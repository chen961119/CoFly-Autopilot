#include "sensors/sensor_calibration.h"
#include "sensors/sensor_processing.h"
#include "config/flight_config.h"
#include "parameters/parameter_service.h"
#include "system/flight_lock.h"
#include "system/device_reboot.h"
#include "actuators/actuator_output.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace {
SensorCalibrationSnapshot job{"NONE", "IDLE", "NONE", "NONE", 0};
bool running = false, airspeedJob = false;
uint32_t startedAt = 0, sampledAt = 0;
unsigned samples = 0;
double sums[6]{}, squares[6]{};
constexpr unsigned IMU_SAMPLES = 2000, AIRSPEED_SAMPLES = 100;

bool finiteValue(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7f800000u) != 0x7f800000u;
}
void finish(const char *error, const char *storage = "NONE") {
  job.error = error ? error : "NONE";
  job.status = error ? "FAILED" : "APPLIED";
  job.storage = storage;
  if (!error) job.progress = 100;
  running = false;
}
}

bool sensorCalibrationBusy() { return running; }
SensorCalibrationSnapshot sensorCalibrationSnapshot() {
  if (!running && !strcmp(job.storage, "PENDING") && !parameterWritesPending()) job.storage = "SAVED";
  return job;
}

const char *startSensorCalibration(const char *sensor) {
  if (running) return "BUSY";
  if (!isFlightLocked()) return "LOCK_REQUIRED";
  if (parameterRebootRequired() || deviceRebootPending()) return "REBOOT_REQUIRED";
  const bool imu = !strcmp(sensor, "IMU") || !strcmp(sensor, "GYRO") || !strcmp(sensor, "ACCEL");
#if !defined(INTIMU) || !defined(USE_MPU6050_I2C)
  if (imu) return "UNSUPPORTED_SENSOR";
#endif
  const bool rotate = !strcmp(sensor, "ROTATE");
  if (!imu && !rotate && strcmp(sensor, "AIRSPEED")) return "UNSUPPORTED_SENSOR";
  // GYRO/ACCEL intentionally calibrate the complete integrity-protected IMU group.
  job = {imu ? "IMU" : rotate ? "ROTATE" : "AIRSPEED", "RUNNING", "NONE", "NONE", 0};
  commandSafeActuatorPositions();
  if (rotate) {
    if (ResetRotateSensor()) finish(nullptr, "PENDING");
    else finish(rotateSensorValid() ? "SAVE_FAILED" : "SENSOR_INVALID");
    return nullptr;
  }
  samples = 0;
  memset(sums, 0, sizeof(sums));
  memset(squares, 0, sizeof(squares));
  airspeedJob = !imu;
  startedAt = sampledAt = millis();
  running = true;
  return nullptr;
}

void pollSensorCalibration() {
  if (!running) return;
  const uint32_t now = millis();
  if (!isFlightLocked() || parameterRebootRequired() || deviceRebootPending()) {
    finish("CANCELLED");
    return;
  }
  if (static_cast<uint32_t>(now - startedAt) > 15000) {
    finish("SENSOR_TIMEOUT");
    return;
  }
  if (static_cast<uint32_t>(now - sampledAt) < (airspeedJob ? 20u : 2u)) return;
  sampledAt = now;
  float values[6]{};
  if (airspeedJob) {
    if (!readAirspeedCalibrationSample(values[0])) return;
  } else {
    if (!readBmi088CalibrationSample(values)) {
      finish("SENSOR_INVALID");
      return;
    }
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
  job.progress = samples * 100u / target;
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
    if (!saveImuCalibrationParameters(offsets)) finish("SAVE_FAILED");
    else finish(nullptr, "PENDING");
  }
}
