#pragma once

#include "config/flight_config.h"
#include "parameters/parameter_keys.h"
#include "parameters/parameter_registry.h"

// Validate a complete candidate snapshot, after all SD records or a USB edit
// have been collected. Checking live values while loading would depend on row order.
inline bool validActuatorPwmParameters(const FlightParameter *table, size_t count,
                                       const double *values) {
  for (uint16_t key = ParameterKey::ActuatorPwmFirst; key <= ParameterKey::ActuatorPwmLast; key += 2) {
    size_t low = count, high = count;
    for (size_t i = 0; i < count; ++i) {
      if (table[i].id == key) low = i;
      if (table[i].id == key + 1) high = i;
    }
    if (low == count || high == count ||
        !validParameterValue(table[low], values[low]) ||
        !validParameterValue(table[high], values[high]) ||
        !validActuatorPwmLimits(static_cast<int>(values[low]), static_cast<int>(values[high])))
      return false;
  }
  return true;
}
