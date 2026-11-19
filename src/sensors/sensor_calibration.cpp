#include "sensors/sensor_calibration.h"
#include "config/flight_config.h"
#include "parameters/parameter_service.h"
#include "system/flight_lock.h"
#include "system/device_reboot.h"
#include "system/rtos_sync.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace {
SensorCalibrationSnapshot job{"NONE", "IDLE", "NONE", "NONE", 0};
enum class Job { Idle, Imu, ButtonImu, Airspeed, Rotate, SaveImu, SaveRotate };
Job phase = Job::Idle;
bool safeOutputApplied = false;
bool buttonJob = false;
uint32_t startedAt = 0, sampledAt = 0;
unsigned samples = 0;
double sums[6]{}, squares[6]{};
float result[6]{};
constexpr unsigned IMU_SAMPLES = 2000, AIRSPEED_SAMPLES = 100;
constexpr unsigned BUTTON_IMU_SAMPLES = 12000;
bool finiteValue(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7f800000u) != 0x7f800000u;
}
void finish(const char *error, const char *storage = "NONE") {
  TaskCriticalSection guard;
  job.error = error ? error : "NONE";
  job.status = error ? "FAILED" : "APPLIED";
  job.storage = storage;
  if (!error) job.progress = 100;
  if (buttonJob) {
    const char *notice = error ? "IMU calibration failed\n" : "IMU calibration applied\n";
    queueUsbDiagnosticOutput(notice, strlen(notice));
  }
  phase = Job::Idle;
}
const char *beginJob(Job kind) {
  TaskCriticalSection guard;
  if (phase != Job::Idle || parameterMaintenanceBusy()) return "BUSY";
  if (!isFlightLocked()) return "LOCK_REQUIRED";
  if (parameterRebootRequired() || deviceRebootPending()) return "REBOOT_REQUIRED";
  const char *name = kind == Job::Airspeed ? "AIRSPEED" : kind == Job::Rotate ? "ROTATE" : "IMU";
  job = {name, "RUNNING", "NONE", "NONE", 0};
  phase = kind;
  buttonJob = kind == Job::ButtonImu;
  safeOutputApplied = false;
  samples = 0;
  memset(sums, 0, sizeof(sums));
  memset(squares, 0, sizeof(squares));
  startedAt = sampledAt = millis();
  return nullptr;
}
}

bool sensorCalibrationBusy() { TaskCriticalSection guard; return phase != Job::Idle; }
bool airspeedCalibrationActive() { TaskCriticalSection guard; return phase == Job::Airspeed; }

SensorCalibrationSnapshot sensorCalibrationSnapshot() {
  TaskCriticalSection guard;
  if (phase == Job::Idle && !strcmp(job.storage, "PENDING") && !parameterWritesPending())
    job.storage = "SAVED";
  return job;
}

const char *startSensorCalibration(const char *sensor) {
  if (!strcmp(sensor, "AIRSPEED")) return beginJob(Job::Airspeed);
  if (!strcmp(sensor, "ROTATE")) return beginJob(Job::Rotate);
#if defined(INTIMU) && defined(USE_MPU6050_I2C)
  if (!strcmp(sensor, "IMU") || !strcmp(sensor, "GYRO") || !strcmp(sensor, "ACCEL"))
    return beginJob(Job::Imu);
#endif
  return "UNSUPPORTED_SENSOR";
}

void requestButtonImuCalibration() {
  TaskCriticalSection guard;
  setFlightLocked(true);
#if defined(INTIMU) && defined(USE_MPU6050_I2C)
  if (!beginJob(Job::ButtonImu)) {
    constexpr char notice[] = "BMI088 calibration started; keep level and still\n";
    queueUsbDiagnosticOutput(notice, sizeof(notice) - 1);
  }
#endif
}

void acknowledgeCalibrationLock() {
  TaskCriticalSection guard;
  if (phase != Job::Idle && isFlightLocked()) safeOutputApplied = true;
}

// A job has exactly one sampling task. No bus transaction or save runs inside a critical section.
void pollSensorCalibration(bool imuOwner) {
  Job kind;
  {
    TaskCriticalSection guard;
    kind = phase;
    if (!safeOutputApplied || kind == Job::Idle || kind == Job::SaveRotate || kind == Job::SaveImu)
      return;
    if (imuOwner != (kind == Job::Imu || kind == Job::ButtonImu)) return;
  }
  const uint32_t now = millis();
  if (!isFlightLocked() || parameterRebootRequired() || deviceRebootPending()) {
    finish("CANCELLED");
    return;
  }
  if (now - startedAt > 15000u) { finish("SENSOR_TIMEOUT"); return; }
  if (kind == Job::Rotate) {
    if (!readRotateCalibrationSample(result[0])) { finish("SENSOR_INVALID"); return; }
    TaskCriticalSection guard;
    phase = Job::SaveRotate;
    return;
  }
  float values[6]{};
  if (kind == Job::ButtonImu) {
    // Retain the 12000 raw readings and 100 us settling interval. Bound each burst
    // to eight readings so the locked 2 ms PWM/service cycle keeps running.
    for (unsigned n = 0; n < 8 && samples < BUTTON_IMU_SAMPLES; ++n) {
      if (!readBmi088CalibrationSample(values)) { finish("SENSOR_INVALID"); return; }
      for (unsigned i = 0; i < 6; ++i) sums[i] += values[i];
      ++samples;
      if (n < 7) delayMicroseconds(100);
    }
    TaskCriticalSection guard;
    job.progress = samples * 100u / BUTTON_IMU_SAMPLES;
    if (samples == BUTTON_IMU_SAMPLES) {
      for (unsigned i = 0; i < 6; ++i) result[i] = sums[i] / samples;
      result[2] -= 1.0f;
      phase = Job::SaveImu;
    }
    return;
  }
  const bool airspeedJob = kind == Job::Airspeed;
  if (now - sampledAt < (airspeedJob ? 20u : 2u)) return;
  sampledAt = now;
  if (airspeedJob) {
    if (!readAirspeedCalibrationSample(values[0])) return;
  } else {
    if (!readBmi088CalibrationSample(values)) { finish("SENSOR_INVALID"); return; }
    // Reject disconnected sensors, tilted installation and motion before changing RAM.
    const float norm = values[0]*values[0] + values[1]*values[1] + values[2]*values[2];
    if (!finiteValue(norm) || norm < 0.64f || norm > 1.44f ||
        fabsf(values[0]) > 0.2f || fabsf(values[1]) > 0.2f || values[2] < 0.8f) {
      finish("KEEP_LEVEL");
      return;
    }
    for (unsigned i = 3; i < 6; ++i) {
      if (!finiteValue(values[i]) || fabsf(values[i]) > 20.0f) {
        finish("KEEP_STILL");
        return;
      }
    }
  }
  for (unsigned i = 0; i < (airspeedJob ? 1u : 6u); ++i) {
    sums[i] += values[i];
    squares[i] += static_cast<double>(values[i]) * values[i];
  }
  ++samples;
  const unsigned target = airspeedJob ? AIRSPEED_SAMPLES : IMU_SAMPLES;
  { TaskCriticalSection guard; job.progress = samples * 100u / target; }
  if (samples < target) return;
  float offsets[6]{};
  for (unsigned i = 0; i < (airspeedJob ? 1u : 6u); ++i) {
    offsets[i] = sums[i] / samples;
    const double variance = squares[i] / samples - (sums[i] / samples) * (sums[i] / samples);
    // Pressure is PSI; allow 10 Pa standard deviation during no-wind zeroing.
    const double limit = airspeedJob ? 0.0000021 : i < 3 ? 0.0004 : 0.25;
    if (!finiteValue(offsets[i]) || variance > limit) {
      finish(airspeedJob ? "KEEP_NO_WIND" : "KEEP_STILL");
      return;
    }
  }
  if (airspeedJob) {
    applyAirspeedCalibration(offsets[0]);
    finish(nullptr, "VOLATILE"); // Same lifetime as the existing startup zero.
  } else {
    offsets[2] -= 1.0f;
    TaskCriticalSection guard;
    memcpy(result, offsets, sizeof(result));
    phase = Job::SaveImu;
  }
}

void serviceCalibrationParameters() {
  Job kind;
  {
    TaskCriticalSection guard;
    kind = phase;
  }
  if (kind != Job::SaveImu && kind != Job::SaveRotate) return;
  if (parameterRebootRequired() || deviceRebootPending() || !isFlightLocked()) {
    finish("CANCELLED");
    return;
  }
  const bool saved = kind == Job::SaveImu ? saveImuCalibrationParameters(result)
                                         : saveParameterValue("mt6701_zero", result[0]);
  if (saved) finish(nullptr, "PENDING");
  else finish("SAVE_FAILED");
}
