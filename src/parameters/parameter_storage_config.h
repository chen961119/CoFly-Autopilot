#pragma once
#include <stddef.h>

// Change this default, or override with -DCOFLY_PARAMETER_NAME_LIMIT=48.
// Text records contain the actual name length, without padding.
#ifndef COFLY_PARAMETER_NAME_LIMIT
#define COFLY_PARAMETER_NAME_LIMIT 27
#endif
constexpr size_t MAX_PARAMETER_NAME = COFLY_PARAMETER_NAME_LIMIT;
constexpr size_t PARAMETER_NAME_COMPAT_LIMIT = 128;
static_assert(MAX_PARAMETER_NAME > 0 && MAX_PARAMETER_NAME <= PARAMETER_NAME_COMPAT_LIMIT,
              "Parameter name limit must be between 1 and 128");
constexpr size_t PARAMETER_VALUE_SIZE = 64;
// Compatible name + value + separators/type/LF/NUL; a buffer limit, not a stride.
constexpr size_t PARAMETER_RECORD_BUFFER_SIZE =
    PARAMETER_NAME_COMPAT_LIMIT + PARAMETER_VALUE_SIZE + 9;

constexpr size_t PARAMETER_COUNT_LIMIT = 512;
constexpr size_t PARAMETER_EEPROM_SLOT_SIZE = 2048;
constexpr int PARAMETER_EEPROM_BASE = 128;
constexpr unsigned PARAMETER_EEPROM_SLOT_COUNT = 2;
