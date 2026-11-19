#include "sensors/airspeed_sensor.h"

bool AirspeedSensor::readFreshPressure(float &pressure) {
  if (!readMS4525() || lastStatus != 0) return false;
  pressure = MS4525_param.pressure;
  return true;
}

bool AirspeedSensor::calibrateZero() {
  constexpr unsigned requiredSamples = 1000;
  constexpr uint32_t timeoutMs = 2500;
  constexpr unsigned maxConsecutiveShortReads = 3;
  const uint32_t startedAt = millis();
  double sum = 0;
  unsigned samples = 0;
  unsigned shortReads = 0;
  while (static_cast<uint32_t>(millis() - startedAt) < timeoutMs) {
    float pressure;
    if (readFreshPressure(pressure)) {
      shortReads = 0;
      sum += pressure;
      if (++samples == requiredSamples) {
        MS4525_param.pressure_drift = static_cast<float>(sum / samples);
        return true;
      }
    } else {
      // A complete stale frame means the sensor responded. Give it time to update.
      shortReads = lastReadOk ? 0 : shortReads + 1;
      if (shortReads >= maxConsecutiveShortReads) break;
      delay(1);
    }
  }
  return false;
}
