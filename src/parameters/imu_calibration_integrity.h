#pragma once
#include <stdint.h>
#include <string.h>

constexpr unsigned IMU_CALIBRATION_OFFSET_COUNT = 6;

// Fingerprint of the complete calibration group, independent of storage medium.
inline int32_t imuCalibrationFingerprint(const float *offsets, int32_t model) {
  uint32_t hash = 2166136261u;
  for (unsigned i = 0; i <= IMU_CALIBRATION_OFFSET_COUNT; ++i) {
    uint32_t bits;
    if (i < IMU_CALIBRATION_OFFSET_COUNT)
      memcpy(&bits, offsets + i, 4);
    else
      memcpy(&bits, &model, 4);
    for (unsigned byte = 0; byte < 4; ++byte)
      hash = (hash ^ ((bits >> (8 * byte)) & 255u)) * 16777619u;
  }
  int32_t result;
  memcpy(&result, &hash, 4);
  return result;
}
