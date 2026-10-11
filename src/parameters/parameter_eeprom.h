#pragma once
#include "parameters/parameter_registry.h"

// EEPROM 0..127 is unused; retain slot addresses for current-format snapshots.
// Fixed 2048-byte primary/backup slots at 128 and 2176. Shared ASCII V8
// header/records/CRC; capacity depends on actual name and numeric text lengths.
bool initializeEEPROMParameters();
bool readEEPROMParameter(const FlightParameter &parameter, double &value);
bool hasEEPROMMigrationSource(const FlightParameter &parameter);
bool saveEEPROMParameters(const FlightParameter *table, size_t count, const double *values,
                          bool sdVerified = false);
const char *eepromParameterState();
