#pragma once
#include <stdint.h>

struct SensorCalibrationSnapshot {
  const char *sensor;
  const char *status; // IDLE, RUNNING, APPLIED, FAILED
  const char *error;
  const char *storage; // NONE, PENDING (ordinary delayed save), SAVED, VOLATILE
  uint8_t progress;
};

// One aircraft-wide job, shared by USB/X6. nullptr means accepted.
const char *startSensorCalibration(const char *sensor);
void pollSensorCalibration();
bool sensorCalibrationBusy();
SensorCalibrationSnapshot sensorCalibrationSnapshot();

// Sensor-owned raw readings, before offsets/filtering. IMU units: g, deg/s.
bool readBmi088CalibrationSample(float values[6]);
bool readAirspeedCalibrationSample(float &pressure);
void applyAirspeedCalibration(float pressure);
