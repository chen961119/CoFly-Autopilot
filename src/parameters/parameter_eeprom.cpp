#include "parameters/parameter_eeprom.h"
#include "parameters/parameter_text.h"
#include "config/aircraft_config.h"
#include <EEPROM.h>

namespace {
constexpr size_t SIZE = PARAMETER_EEPROM_SLOT_SIZE;
constexpr int BASE = PARAMETER_EEPROM_BASE;
uint8_t current[SIZE], next[SIZE];
int active = -1;
const char *state = "DEFAULTS";
const FlightParameter *table;
size_t count, aircraftIdIndex;
bool valid(const uint8_t *data) {
  ParameterText::View view;
  return ParameterText::valid(data, SIZE, view);
}
void readSlot(int slot, uint8_t *buffer) {
  for (size_t i = 0; i < SIZE; ++i)
    buffer[i] = EEPROM.read(BASE + slot * SIZE + i);
}
bool writeSlot(int slot, const uint8_t *data) {
  ParameterText::View view;
  if (!ParameterText::valid(data, SIZE, view))
    return false;
  const int base = BASE + slot * SIZE;
  // Invalidate the ordinary version header while replacing the slot. Restore
  // its first character only after all remaining text/CRC bytes are verified.
  // This uses no extra stored commit field; SD and EEPROM bytes stay identical.
  EEPROM.update(base, 0);
  if (EEPROM.read(base) != 0)
    return false;
  for (size_t i = 1; i < view.used; ++i) {
    EEPROM.update(base + i, data[i]);
    if (EEPROM.read(base + i) != data[i])
      return false;
  }
  EEPROM.update(base, data[0]);
  for (size_t i = 0; i < view.used; ++i)
    if (EEPROM.read(base + i) != data[i])
      return false;
  return true;
}
} // namespace

bool initializeEEPROMParameters(const FlightParameter *parameters, size_t parameterCount) {
  table = parameters;
  count = parameterCount;
  aircraftIdIndex = findParameterIndex(table, count, "aircraft_id");
  active = -1;
  state = "DEFAULTS";
  if (EEPROM.length() < BASE + PARAMETER_EEPROM_SLOT_COUNT * SIZE) {
    state = "CAPACITY_ERROR";
    return false;
  }
  readSlot(0, current);
  if (valid(current))
    active = 0;
  else {
    readSlot(1, next);
    if (valid(next)) {
      memcpy(current, next, SIZE);
      active = 1;
    } else {
      if (!memcmp(current, "COFLY_PARAMS", 12) || !memcmp(next, "COFLY_PARAMS", 12))
        state = "CORRUPT";
      return false;
    }
  }
  state = "LOADED";
  return true;
}
bool readEEPROMParameter(const FlightParameter &p, double &value) {
  if (active < 0 || !validParameterName(p.name))
    return false;
  ParameterText::View view;
  if (!ParameterText::header(current, SIZE, view))
    return false;
  size_t offset = view.records;
  for (size_t i = 0; i < view.rows; ++i) {
    char line[PARAMETER_RECORD_BUFFER_SIZE], *name, *type;
    double decoded;
    if (!ParameterText::line(current, view.used, offset, line, sizeof(line)) ||
        !ParameterText::decode(line, name, type, decoded))
      return false;
    if (!strcmp(name, p.name) && !strcmp(type, p.type()) && validParameterValue(p, decoded)) {
      value = decoded;
      return true;
    }
  }
  return false;
}
bool hasEEPROMMigrationSource(const FlightParameter &p) {
  double value;
  return readEEPROMParameter(p, value);
}
bool saveEEPROMParameters(const double *values, bool sdVerified) {
  if (!table || !values || !count || aircraftIdIndex == count ||
      EEPROM.length() < BASE + PARAMETER_EEPROM_SLOT_COUNT * SIZE)
    return false;
  memset(next, 0, SIZE);
  size_t rows = 0;
  const uint32_t identity = static_cast<uint32_t>(values[aircraftIdIndex]);
  // Count first, since the text COUNT field is variable width.
  for (size_t i = 0; i < count; ++i) {
    if (table[i].storage == ParameterStorage::EEPROM ||
        (!sdVerified && hasEEPROMMigrationSource(table[i])))
      ++rows;
  }
  size_t used = ParameterText::begin(next, SIZE, rows, aircraftCount(), identity);
  if (!used)
    return false;
  for (size_t i = 0; i < count; ++i) {
    const auto &p = table[i];
    double value = values[i];
    if (p.storage != ParameterStorage::EEPROM) {
      if (sdVerified || !readEEPROMParameter(p, value))
        continue;
    }
    for (size_t j = 0; j < i; ++j)
      if (!strcmp(table[j].name, p.name))
        return false;
    const int n =
        ParameterRecord::encode(p, value, reinterpret_cast<char *>(next + used), SIZE - used);
    if (n < 0)
      return false;
    used += n;
  }
  used = ParameterText::finish(next, SIZE, used);
  if (!used || !valid(next))
    return false;
  ParameterText::View old;
  if (active == 0 && ParameterText::header(current, SIZE, old) && used == old.used &&
      !memcmp(next, current, used))
    return true;
  if (active == 0) {
    if (!ParameterText::header(current, SIZE, old))
      return false;
    for (size_t i = 0; i < old.used; ++i)
      if (EEPROM.read(BASE + i) != current[i])
        return false;
    if (!writeSlot(1, current))
      return false;
  }
  if (!writeSlot(0, next)) {
    if (active >= 0)
      active = 1;
    return false;
  }
  memcpy(current, next, SIZE);
  active = 0;
  state = "SAVED";
  return true;
}
const char *eepromParameterState() {
  return state;
}
