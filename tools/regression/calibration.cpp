// Host regression of the production calibration state machine and task handoff.
#include "sensors/sensor_calibration.h"
#include "sensors/strain_frame.h"
#include "system/flight_lock.h"
#include "system/flight_clock.h"
#include <cassert>
#include <cmath>
#include <cstring>

static uint32_t nowMs;
static bool locked, pending, maintenance, allowSave = true;
static unsigned imuReads, pressureReads, saves, settlingDelays;
static float gravity = 1.0f, appliedPressure, savedDegrees;
uint32_t testCycles;
static uint32_t nowUs;
uint32_t millis() { return nowMs; }
uint32_t micros() { return nowUs; }
void delayMicroseconds(unsigned us) { assert(us == 100); ++settlingDelays; }
bool isFlightLocked() { return locked; }
void setFlightLocked(bool value) { locked = value; }
bool parameterRebootRequired() { return pending; }
bool deviceRebootPending() { return false; }
bool parameterMaintenanceBusy() { return maintenance; }
bool parameterWritesPending() { return false; }
bool queueUsbDiagnosticOutput(const char *, size_t) { return true; }
bool readBmi088CalibrationSample(float values[6]) {
  ++imuReads;
  values[2] = gravity;
  values[3] = 0.5f;
  return true;
}
bool readAirspeedCalibrationSample(float &value) { ++pressureReads; value = .01f; return true; }
void applyAirspeedCalibration(float value) { appliedPressure = value; }
bool readRotateCalibrationSample(float &value) { value = 310.0f; return true; }
bool saveParameterValue(const char *name, double value) {
  assert(!strcmp(name, "mt6701_zero"));
  savedDegrees = value;
  ++saves;
  return allowSave;
}
bool saveImuCalibrationParameters(const float *values) {
  ++saves;
  assert(std::fabs(values[2]) < 1e-6f && std::fabs(values[3] - .5f) < 1e-6f);
  return allowSave;
}

static void testStrainFrames() {
  const uint8_t check[] = "123456789";
  assert(modbusCrc16(check, 9) == 0x4b37);
  uint8_t frame[25] = {1, 3, 20, 0x12, 0x34, 0xfe, 0xdc};
  const uint16_t crc = modbusCrc16(frame, 23);
  frame[23] = crc & 255;
  frame[24] = crc >> 8;
  StrainFrameDecoder decoder;
  int32_t channels[5] = {5, 4, 3, 2, 1};
  for (unsigned i = 0; i < 1024; ++i) assert(!decoder.push(i & 1, channels));
  for (unsigned i = 0; i < sizeof(frame); ++i)
    assert(!decoder.push(i == 24 ? frame[i] ^ 2 : frame[i], channels));
  assert(channels[0] == 5 && channels[4] == 1);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    for (unsigned i = 0; i < 24; ++i) assert(!decoder.push(frame[i], channels));
    assert(decoder.push(frame[24], channels));
    assert(uint32_t(channels[0]) == 0xfedc1234u && channels[4] == 0);
  }
}

int main() {
  testStrainFrames();
  testCycles = UINT32_MAX - 1200;
  nowUs = UINT32_MAX - 100;
  updateFlightClock();
  testCycles += 1200000;
  nowUs += 2000;
  updateFlightClock();
  assert(std::fabs(dt - .002f) < 1e-7f);
  finishFlightCycle(200, 1200, 0);
  // A masked SysTick can report one elapsed tick while five milliseconds pass.
  testCycles += 3000000;
  nowUs += 1000;
  updateFlightClock();
  finishFlightCycle(300, 2090, 0);
  const auto timing = flightTimingStats();
  assert(timing.cycles == 2 && timing.totalPeriodUs == 7000 && timing.maxPeriodUs == 5000);
  assert(timing.intervalsAtLeast4ms == 1 && timing.executionsOver2ms == 1);
  assert(timing.maxImuToPwmUs == 300 && timing.missedReleases == 0);
  assert(!strcmp(startSensorCalibration("IMU"), "LOCK_REQUIRED"));
  locked = true;
  maintenance = true;
  assert(!strcmp(startSensorCalibration("IMU"), "BUSY"));
  maintenance = false;
  assert(!startSensorCalibration("GYRO"));
  nowMs += 10;
  pollSensorCalibration(true);
  assert(imuReads == 0);
  acknowledgeCalibrationLock();
  assert(!strcmp(startSensorCalibration("AIRSPEED"), "BUSY"));
  for (unsigned i = 0; i < 2000; ++i) {
    pollSensorCalibration(false);
    nowMs += 2;
    pollSensorCalibration(true);
  }
  assert(imuReads == 2000 && pressureReads == 0 && saves == 0 && sensorCalibrationBusy());
  serviceCalibrationParameters();
  serviceCalibrationParameters();
  assert(saves == 1 && !sensorCalibrationBusy());

  locked = false;
  requestButtonImuCalibration();
  assert(locked && sensorCalibrationBusy());
  acknowledgeCalibrationLock();
  for (unsigned i = 0; i < 1500; ++i) { nowMs += 2; pollSensorCalibration(true); }
  assert(imuReads == 14000 && settlingDelays == 1500 * 7);
  serviceCalibrationParameters();
  assert(saves == 2);

  assert(!startSensorCalibration("AIRSPEED"));
  acknowledgeCalibrationLock();
  for (unsigned i = 0; i < 100; ++i) {
    nowMs += 20;
    pollSensorCalibration(true);
    pollSensorCalibration(false);
  }
  assert(pressureReads == 100 && appliedPressure == .01f && !sensorCalibrationBusy());
  assert(!startSensorCalibration("ROTATE"));
  acknowledgeCalibrationLock();
  pollSensorCalibration(false);
  allowSave = false;
  serviceCalibrationParameters();
  assert(savedDegrees == 310 && !strcmp(sensorCalibrationSnapshot().error, "SAVE_FAILED"));

  assert(!startSensorCalibration("IMU"));
  acknowledgeCalibrationLock();
  gravity = .1f;
  nowMs += 2;
  pollSensorCalibration(true);
  assert(!strcmp(sensorCalibrationSnapshot().error, "KEEP_LEVEL"));
  assert(!startSensorCalibration("ROTATE"));
  acknowledgeCalibrationLock();
  pending = true;
  pollSensorCalibration(false);
  assert(!strcmp(sensorCalibrationSnapshot().error, "CANCELLED"));
  pending = false;
  nowMs = UINT32_MAX - 100;
  assert(!startSensorCalibration("AIRSPEED"));
  acknowledgeCalibrationLock();
  nowMs += 15001u;
  pollSensorCalibration(false);
  assert(!strcmp(sensorCalibrationSnapshot().error, "SENSOR_TIMEOUT"));
}
