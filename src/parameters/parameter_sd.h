#pragma once

#include "parameters/parameter_registry.h"

// Main-loop-only SD snapshot IO. The caller owns the registry and defaults.
namespace ParameterSD {
struct MigrationStats {
  size_t matched;
  size_t defaultsUsed;
  size_t skipped;
  uint32_t sourceAircraftId;
};

using ValueValidator = bool (*)(size_t index, double value);

void initialize(bool ready, const FlightParameter *parameters, size_t count, const double *defaults,
                ValueValidator validator);
// Primary first, backup on failure. Restores missing values from defaults.
bool load(double *values, bool &needsMigration, MigrationStats &stats);
// The header identifies the airframe whose calibration the snapshot contains.
bool save(const double *values, uint32_t sourceAircraftId);
bool hasRecord(size_t index);
} // namespace ParameterSD
