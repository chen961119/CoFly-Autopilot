#pragma once

#include "config/flight_config.h"
#include "parameters/parameter_keys.h"
#include "parameters/parameter_registry.h"

// Validate a complete candidate snapshot, after all SD records or a USB edit
// have been collected. Checking live values while loading would depend on row order.
inline bool validActuatorPwmParameters(const FlightParameter *table, size_t count,
                                       const double *values) {
  for (uint16_t key = ParameterKey::ActuatorPwmFirst; key <= ParameterKey::ActuatorPwmLast; key += 2) {
    const unsigned channel = (key - ParameterKey::ActuatorPwmFirst) / 2;
    size_t low = count, high = count, center = count, mode = count;
    for (size_t i = 0; i < count; ++i) {
      if (table[i].id == key) low = i;
      if (table[i].id == key + 1) high = i;
      if (table[i].id == ParameterKey::ActuatorTrimFirst + channel) center = i;
      if (table[i].id == ParameterKey::ActuatorModeFirst + channel) mode = i;
    }
    if (low == count || high == count ||
        !validParameterValue(table[low], values[low]) ||
        !validParameterValue(table[high], values[high]) ||
        !validActuatorPwmLimits(static_cast<int>(values[low]), static_cast<int>(values[high])))
      return false;
    if (center == count || mode == count ||
        !validParameterValue(table[center], values[center]) ||
        !validParameterValue(table[mode], values[mode]))
      return false;
    // All channels have the same rule: only mode 0 uses trim as the center.
    if (values[mode] == 0 && (values[center] < values[low] || values[center] > values[high]))
      return false;
  }
  return true;
}
