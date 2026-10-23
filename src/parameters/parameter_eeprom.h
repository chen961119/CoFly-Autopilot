#pragma once
#include "parameters/parameter_registry.h"

// EEPROM 0..127 is unused; retain slot addresses for current-format snapshots.
// Fixed 2048-byte primary/backup slots at 128 and 2176. Shared ASCII
// header/records/CRC; capacity depends on actual name and numeric text lengths.
bool initializeEEPROMParameters(const FlightParameter *table, size_t count);
bool readEEPROMParameter(const FlightParameter &parameter, double &value);
bool hasEEPROMMigrationSource(const FlightParameter &parameter);
bool saveEEPROMParameters(const double *values, bool sdVerified = false);
const char *eepromParameterState();
