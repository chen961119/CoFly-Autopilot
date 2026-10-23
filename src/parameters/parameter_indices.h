#pragma once

#include "parameters/parameter_registry.h"
#include "parameters/imu_calibration_integrity.h"
#include <stdio.h>

struct ActuatorParameterIndices {
  size_t minimum, trim, maximum, mode;
};

// Runtime table positions, resolved by name once. Registry rows may be reordered freely.
struct ParameterIndices {
  size_t aircraftId;
  ActuatorParameterIndices actuators[5];
  size_t imuOffsets[IMU_CALIBRATION_OFFSET_COUNT];
  size_t imuValid, imuModel, imuChecksum;
  // Calibration protocol order is MIN/MAX/TRIM/REV, independent of display order.
  size_t radioCalibration[4][4];

  bool initialize(const FlightParameter *table, size_t count) {
    aircraftId = findParameterIndex(table, count, "aircraft_id");
    imuValid = findParameterIndex(table, count, "imu_calibration_valid");
    imuModel = findParameterIndex(table, count, "imu_calibration_model");
    imuChecksum = findParameterIndex(table, count, "imu_calibration_crc");
    if (aircraftId == count || imuValid == count ||
        imuModel == count || imuChecksum == count) return false;
    const char *const offsetNames[] = {
        "imu_accerrorx_6050", "imu_accerrory_6050", "imu_accerrorz_6050",
        "imu_gyroerrorx_6050", "imu_gyroerrory_6050", "imu_gyroerrorz_6050"};
    for (unsigned i = 0; i < IMU_CALIBRATION_OFFSET_COUNT; ++i) {
      imuOffsets[i] = findParameterIndex(table, count, offsetNames[i]);
      if (imuOffsets[i] == count) return false;
    }
    char name[32];
    for (unsigned channel = 0; channel < 5; ++channel) {
      auto &a = actuators[channel];
      snprintf(name, sizeof(name), "pwm_servo%u_min_us", channel + 1);
      a.minimum = findParameterIndex(table, count, name);
      snprintf(name, sizeof(name), "pwm_channel%u_trim", channel + 1);
      a.trim = findParameterIndex(table, count, name);
      snprintf(name, sizeof(name), "pwm_servo%u_max_us", channel + 1);
      a.maximum = findParameterIndex(table, count, name);
      snprintf(name, sizeof(name), "pwm_channel%u_mode", channel + 1);
      a.mode = findParameterIndex(table, count, name);
      if (a.minimum == count || a.trim == count || a.maximum == count || a.mode == count)
        return false;
    }
    const char *const fields[] = {"MIN", "MAX", "TRIM", "REV"};
    for (unsigned channel = 0; channel < 4; ++channel)
      for (unsigned field = 0; field < 4; ++field) {
        snprintf(name, sizeof(name), "RC%u_%s", channel + 1, fields[field]);
        radioCalibration[channel][field] = findParameterIndex(table, count, name);
        if (radioCalibration[channel][field] == count) return false;
      }
    return true;
  }

  bool isImuOffset(size_t index) const {
    for (size_t offset : imuOffsets) if (index == offset) return true;
    return false;
  }
  bool isImuCalibration(size_t index) const {
    return isImuOffset(index) || index == imuValid || index == imuModel || index == imuChecksum;
  }
  size_t imuMember(unsigned member) const {
    if (member < IMU_CALIBRATION_OFFSET_COUNT) return imuOffsets[member];
    if (member == IMU_CALIBRATION_OFFSET_COUNT) return imuValid;
    return member == IMU_CALIBRATION_OFFSET_COUNT + 1 ? imuModel : imuChecksum;
  }
  bool isActuatorLimit(size_t index) const {
    for (const auto &a : actuators)
      if (index == a.minimum || index == a.maximum) return true;
    return false;
  }
};
