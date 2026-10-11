#include "parameters/parameter_service.h"
#include "communication/ground_station_service.h"
#include "communication/serial_ports.h"
#include "config/aircraft_config.h"
#include "parameters/imu_calibration_integrity.h"
#include "parameters/parameter_eeprom.h"
#include "parameters/parameter_keys.h"
#include "parameters/parameter_registry.h"
#include "parameters/parameter_save_debounce.h"
#include "parameters/parameter_sd.h"
#include "parameters/parameter_text.h"
#include "sensors/sensor_processing.h"
#include "system/device_reboot.h"
#include "system/flight_lock.h"
#include <Arduino.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr size_t MAX_PARAMETERS = PARAMETER_COUNT_LIMIT;
constexpr size_t LINE_SIZE = 512;
constexpr size_t VALUE_SIZE = PARAMETER_VALUE_SIZE;

bool eepromLoaded[MAX_PARAMETERS];

const FlightParameter *table;
size_t count;
bool storageReady;
bool rebootRequired = false;
char rxLine[LINE_SIZE];
size_t rxLength;
bool rxOverflow;
char txLine[LINE_SIZE];
size_t txLength;
size_t listIndex;
bool listing;
uint32_t listId;
double snapshot[MAX_PARAMETERS];
double defaults[MAX_PARAMETERS];
// Main-loop-only, non-reentrant service; snapshots stay out of the stack.
double candidate[MAX_PARAMETERS];
ParameterSaveDebounce sdSave, eeSave;
enum class StorageNotice { None, Saved, Failed };
StorageNotice sdNotice = StorageNotice::None, eeNotice = StorageNotice::None;
const char *loadState = "DEFAULTS";

ParameterSD::MigrationStats loadStats;

using ParameterRecord::parseUnsigned;
bool validValue(size_t index, double value) {
  return validParameterValue(table[index], value) &&
         (table[index].id != ParameterKey::AircraftCount || value == 1 || value == 3 ||
          value == 4 || value == 5 || value == 7);
}
bool parseValue(size_t index, const char *text, double &value) {
  return ParameterRecord::parseValue(table[index], text, value) && validValue(index, value);
}
void formatValue(size_t index, double value, char *text, size_t capacity) {
  ParameterRecord::formatValue(table[index], value, text, capacity);
}

size_t parameterIndex(uint16_t key) {
  size_t i = 0;
  while (i < count && table[i].id != key)
    ++i;
  return i;
}

bool retireMigratedSources(const double *values) {
  for (size_t i = 0; i < count; ++i)
    if (table[i].storage == ParameterStorage::SD && hasEEPROMMigrationSource(table[i]))
      return saveEEPROMParameters(table, count, values, true);
  return true;
}

bool persist(const double *values, ParameterStorage medium) {
  if (medium == ParameterStorage::EEPROM)
    return saveEEPROMParameters(table, count, values);
  if (!ParameterSD::save(values))
    return false;
  loadState = "SAVED";
  if (!retireMigratedSources(values)) {
    loadState = "MIGRATION_PENDING";
    return false;
  }
  return true;
}

void applyCandidateParameters() {
  for (size_t i = 0; i < count; ++i) {
    if (table[i].effect == ParameterEffect::Reboot && table[i].read() != candidate[i])
      rebootRequired = true;
    table[i].write(candidate[i]);
  }
  if (rebootRequired)
    setFlightLocked(true);
}

bool flushMedium(ParameterStorage medium) {
  auto &pending = medium == ParameterStorage::SD ? sdSave : eeSave;
  auto &notice = medium == ParameterStorage::SD ? sdNotice : eeNotice;
  if (!pending.dirty)
    return true;
  for (size_t i = 0; i < count; ++i)
    candidate[i] = table[i].read();
  const bool success = persist(candidate, medium);
  pending.completed(success, millis());
  notice = success ? StorageNotice::Saved : StorageNotice::Failed;
  return success;
}

void pollParameterSaves() {
  if (eeSave.due(millis()))
    flushMedium(ParameterStorage::EEPROM);
  if (sdSave.due(millis()))
    flushMedium(ParameterStorage::SD);
}

void markParameterWrites(bool sd, bool ee) {
  const uint32_t now = millis();
  if (sd) {
    sdSave.changed(now);
    sdNotice = StorageNotice::None;
  }
  if (ee) {
    eeSave.changed(now);
    eeNotice = StorageNotice::None;
  }
}

bool requiresFlightLock(const FlightParameter &parameter) {
  return parameter.storage == ParameterStorage::EEPROM ||
         parameter.id >= ParameterKey::AircraftId || parameter.effect == ParameterEffect::Reboot;
}

enum class SaveResult { Rejected, Pending, Saved };
SaveResult saveOne(size_t index, double value) {
  if (index == count || table[index].readOnly || !validValue(index, value))
    return SaveResult::Rejected;
  if (!table[index].integer)
    value = static_cast<float>(value);
  if (requiresFlightLock(table[index]) && !isFlightLocked())
    return SaveResult::Rejected;
  for (size_t i = 0; i < count; ++i)
    candidate[i] = table[i].read();
  candidate[index] = value;
  const size_t idIndex = parameterIndex(ParameterKey::AircraftId),
               countIndex = parameterIndex(ParameterKey::AircraftCount);
  if (idIndex < count && countIndex < count &&
      !validAircraftConfiguration(candidate[idIndex], candidate[countIndex]))
    return SaveResult::Rejected;
  const bool calibrationOffset = table[index].id >= ParameterKey::ImuOffsetFirst &&
                                 table[index].id <= ParameterKey::ImuOffsetLast;
  const size_t marker = parameterIndex(ParameterKey::ImuCalibrationValid);
  if (calibrationOffset && marker == count)
    return SaveResult::Rejected;
  if (table[index].read() == value)
    return (table[index].storage == ParameterStorage::SD ? sdSave.dirty : eeSave.dirty)
               ? SaveResult::Pending
               : SaveResult::Saved;
  // Both immediate values and next-boot settings enter RAM before delayed I/O.
  if (calibrationOffset) {
    candidate[marker] = 0;
  }
  applyCandidateParameters();
  // Initialization enforces one medium for the entire IMU group, including its marker.
  markParameterWrites(table[index].storage == ParameterStorage::SD,
                      table[index].storage == ParameterStorage::EEPROM);
  if (calibrationOffset)
    loadImuCalibration();
  return SaveResult::Pending;
}

void queueReply(uint32_t id, const char *format, ...) {
  int prefix = snprintf(txLine, sizeof(txLine), "@COFLY\t%lu\t", static_cast<unsigned long>(id));
  va_list args;
  va_start(args, format);
  int body = vsnprintf(txLine + prefix, sizeof(txLine) - prefix - 2, format, args);
  va_end(args);
  if (body < 0 || static_cast<size_t>(body) >= sizeof(txLine) - prefix - 2) {
    body = snprintf(txLine + prefix, sizeof(txLine) - prefix - 2,
                    "ERROR\tINTERNAL\tResponse too long");
  }
  txLength = prefix + body;
  txLine[txLength++] = '\n';
  txLine[txLength] = '\0';
}

void error(uint32_t id, const char *code, const char *message) {
  queueReply(id, "ERROR\t%s\t%s", code, message);
}

void handleCommand(char *line) {
  if (!strncmp(line, "GCS ", 4) || !strncmp(line, "GCS\t", 4)) {
    handleGroundStationCommand(line, txLine, sizeof(txLine));
    txLength = strlen(txLine);
    return;
  }
  char *tokens[7];
  size_t tokenCount = 0;
  char *save;
  for (char *p = strtok_r(line, " \t", &save); p; p = strtok_r(nullptr, " \t", &save)) {
    if (tokenCount == 7) {
      error(0, "SYNTAX", "Too many arguments");
      return;
    }
    tokens[tokenCount++] = p;
  }
  if (tokenCount == 0)
    return;
  if (strcmp(tokens[0], "PARAM") || tokenCount < 2) {
    error(0, "UNKNOWN_COMMAND", "Use PARAM HELP");
    return;
  }
  uint32_t id = 0;
  if (tokenCount >= 3 && !parseUnsigned(tokens[2], id)) {
    error(0, "SYNTAX", "Request ID must be uint32");
    return;
  }
  if (!strcmp(tokens[1], "HELP") && tokenCount <= 3) {
    queueReply(id, "HELP\tPARAM READ [id]; PARAM SET id name value; PARAM HELP [id]");
  } else if (!strcmp(tokens[1], "READ") && tokenCount <= 3) {
    for (size_t i = 0; i < count; ++i)
      snapshot[i] = table[i].read();
    listId = id;
    listIndex = 0;
    listing = true;
    queueReply(id, "BEGIN\t%s\t%u\t%s\t%s\t%u\t%u\t%u\t%u", ParameterText::VERSION,
               static_cast<unsigned>(count), storageReady ? "SD_READY" : "NO_SD",
               (sdSave.dirty || eeSave.dirty) ? "PENDING" : loadState,
               static_cast<unsigned>(loadStats.matched),
               static_cast<unsigned>(loadStats.defaultsUsed),
               static_cast<unsigned>(loadStats.skipped), static_cast<unsigned>(MAX_PARAMETER_NAME));
  } else if (!strcmp(tokens[1], "SET") && tokenCount == 5) {
    size_t index = 0;
    while (index < count && strcmp(tokens[3], table[index].name))
      ++index;
    if (index == count) {
      error(id, "UNKNOWN_PARAMETER", "Parameter not registered");
      return;
    }
    double value;
    if (!parseValue(index, tokens[4], value)) {
      error(id, "RANGE", "Value must be finite and within parameter bounds");
      return;
    }
    if (table[index].readOnly) {
      error(id, "READ_ONLY", "Calibration metadata is managed by firmware");
      return;
    }
    if (requiresFlightLock(table[index]) && !isFlightLocked()) {
      error(id, "LOCK_REQUIRED", "Lock aircraft before changing reboot parameters or calibration");
      return;
    }
    if (table[index].storage == ParameterStorage::SD && !storageReady &&
        table[index].effect == ParameterEffect::Reboot) {
      error(id, "NO_SD", "SD card unavailable; RAM unchanged");
      return;
    }
    const SaveResult result = saveOne(index, value);
    if (result == SaveResult::Rejected) {
      error(id, "SAVE_FAILED", "Configuration invalid; parameter unchanged");
      return;
    }
    char formatted[VALUE_SIZE];
    formatValue(index, value, formatted, sizeof(formatted));
    queueReply(id, "OK\t%s\t%s\t%s\t%s", table[index].name, table[index].type(), formatted,
               result == SaveResult::Saved ? "SAVED" : "PENDING");
  } else {
    error(id, "SYNTAX", "Use PARAM READ [id] or PARAM SET id name value");
  }
}
} // namespace

void initializeParameterService(bool sdReady) {
  rebootRequired = false;
  sdSave = {};
  eeSave = {};
  sdNotice = eeNotice = StorageNotice::None;
  table = flightParameterTable(count);
  storageReady = sdReady && count > 0 && count <= MAX_PARAMETERS;
  if (!count || count > MAX_PARAMETERS) {
    count = 0;
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    if (!table[i].id || !validParameterName(table[i].name) ||
        !validValue(i, table[i].defaultValue)) {
      count = 0;
      return;
    }
    for (size_t j = 0; j < i; ++j)
      if (table[j].id == table[i].id || !strcmp(table[j].name, table[i].name)) {
        count = 0;
        return;
      }
  }
  if (parameterIndex(ParameterKey::AircraftId) == count ||
      parameterIndex(ParameterKey::AircraftCount) == count ||
      parameterIndex(ParameterKey::ImuCalibrationValid) == count) {
    count = 0;
    return;
  }
  // A complete calibration is stored in one snapshot. Moving the whole group
  // between SD and EEPROM is supported; splitting it across media is not.
  const auto calibrationStorage = table[parameterIndex(ParameterKey::ImuCalibrationValid)].storage;
  for (uint16_t key = ParameterKey::ImuOffsetFirst; key <= ParameterKey::ImuGroupLast; ++key) {
    const size_t index = parameterIndex(key);
    if (index == count || table[index].storage != calibrationStorage) {
      USBSerial.printf("[PARAM] IMU calibration parameters must share one storage medium\n");
      count = 0;
      storageReady = false;
      return;
    }
  }
  rxLength = txLength = listIndex = 0;
  rxOverflow = listing = false;
  loadState = "DEFAULTS";
  loadStats = {0, count, 0};
  const bool haveEEPROM = initializeEEPROMParameters();
  for (size_t i = 0; i < count; ++i) {
    defaults[i] = table[i].defaultValue;
    eepromLoaded[i] = readEEPROMParameter(table[i], defaults[i]);
    if (!validValue(i, defaults[i])) {
      defaults[i] = table[i].defaultValue;
      eepromLoaded[i] = false;
    }
  }
  for (size_t i = 0; i < count; ++i)
    table[i].write(defaults[i]);
  bool needsMigration = false;
  ParameterSD::initialize(storageReady, table, count, defaults, validValue);
  const bool loadedSD = ParameterSD::load(candidate, needsMigration, loadStats);
  if (!loadedSD)
    memcpy(candidate, defaults, count * sizeof(double));
  bool completeCalibration = true;
  for (size_t i = 0; i < count; ++i) {
    if (table[i].storage == ParameterStorage::EEPROM && eepromLoaded[i])
      candidate[i] = defaults[i];
    if (table[i].id >= ParameterKey::ImuOffsetFirst &&
        table[i].id <= ParameterKey::ImuCalibrationChecksum && !eepromLoaded[i] &&
        !ParameterSD::hasRecord(i))
      completeCalibration = false;
    if (table[i].id >= ParameterKey::ImuOffsetFirst &&
        table[i].id <= ParameterKey::ImuCalibrationChecksum &&
        table[i].storage == ParameterStorage::SD && !ParameterSD::hasRecord(i) &&
        !hasEEPROMMigrationSource(table[i]))
      completeCalibration = false;
  }
  const size_t calibrationMarker = parameterIndex(ParameterKey::ImuCalibrationValid);
  if (completeCalibration) {
    float offsets[ParameterKey::ImuOffsetCount];
    for (unsigned i = 0; i < ParameterKey::ImuOffsetCount; ++i)
      offsets[i] = candidate[parameterIndex(ParameterKey::ImuOffsetFirst + i)];
    if (candidate[parameterIndex(ParameterKey::ImuCalibrationChecksum)] !=
        imuCalibrationFingerprint(offsets,
                                  candidate[parameterIndex(ParameterKey::ImuCalibrationModel)]))
      completeCalibration = false;
  }
  if (!completeCalibration && calibrationMarker < count)
    candidate[calibrationMarker] = 0;
  const size_t idIndex = parameterIndex(ParameterKey::AircraftId),
               countIndex = parameterIndex(ParameterKey::AircraftCount);
  if ((table[idIndex].storage == ParameterStorage::SD && !ParameterSD::hasRecord(idIndex) &&
       !hasEEPROMMigrationSource(table[idIndex])) ||
      (table[countIndex].storage == ParameterStorage::SD && !ParameterSD::hasRecord(countIndex) &&
       !hasEEPROMMigrationSource(table[countIndex])))
    candidate[idIndex] = 0;
  if (!validAircraftConfiguration(candidate[idIndex], candidate[countIndex])) {
    candidate[idIndex] = 0;
    candidate[countIndex] = 3;
  }
  const bool eeSaved = saveEEPROMParameters(table, count, candidate);
  if (!eeSaved) {
    candidate[idIndex] = 0;
    candidate[parameterIndex(ParameterKey::ImuCalibrationValid)] = 0;
    loadState = "EEPROM_ERROR";
  }
  for (size_t i = 0; i < count; ++i)
    table[i].write(candidate[i]);
  if (eeSaved) {
    loadState = (loadedSD || haveEEPROM) ? "LOADED" : "DEFAULTS";
    // A storage move may need SD serialization even if the old schema otherwise matches.
    bool missingSD = false;
    for (size_t i = 0; i < count; ++i)
      if (table[i].storage == ParameterStorage::SD && !ParameterSD::hasRecord(i) && eepromLoaded[i])
        missingSD = true;
    if (storageReady && (needsMigration || missingSD)) {
      if (!ParameterSD::save(candidate, true) || !retireMigratedSources(candidate))
        loadState = "MIGRATION_PENDING";
      else
        loadState = "MIGRATED";
    }
  }
  activateAircraftConfiguration();
  USBSerial.printf("[PARAM] %s; EEPROM %s; aircraft %s/%d\n", loadState, eepromParameterState(),
                   aircraftName(), aircraftCount());
}

bool saveParameterValue(uint16_t id, double value) {
  return saveOne(parameterIndex(id), value) != SaveResult::Rejected;
}
bool flushParameterWrites() {
  if (!table || !count)
    return true;
  // Explicit reboot must not drop queued changes; attempt both media.
  const bool eeSaved = flushMedium(ParameterStorage::EEPROM);
  const bool sdSaved = flushMedium(ParameterStorage::SD);
  return eeSaved && sdSaved;
}
bool parameterRebootRequired() {
  return rebootRequired;
}

bool saveImuCalibrationParameters(const float *offsets) {
  if (!table || !count || !offsets || !isFlightLocked())
    return false;
  for (size_t i = 0; i < count; ++i)
    candidate[i] = table[i].read();
  for (unsigned j = 0; j < ParameterKey::ImuOffsetCount; ++j) {
    const size_t i = parameterIndex(ParameterKey::ImuOffsetFirst + j);
    if (i == count || !validValue(i, offsets[j]))
      return false;
    candidate[i] = offsets[j];
  }
  const size_t marker = parameterIndex(ParameterKey::ImuCalibrationValid),
               model = parameterIndex(ParameterKey::ImuCalibrationModel),
               fingerprint = parameterIndex(ParameterKey::ImuCalibrationChecksum);
  if (marker == count || model == count || fingerprint == count)
    return false;
  candidate[model] = 1;
  candidate[fingerprint] = imuCalibrationFingerprint(offsets, 1);
  candidate[marker] = 1;
  bool sd = false, ee = false;
  for (size_t i = 0; i < count; ++i) {
    if (!validValue(i, candidate[i]))
      return false;
    if (candidate[i] == table[i].read())
      continue;
    sd |= table[i].storage == ParameterStorage::SD;
    ee |= table[i].storage == ParameterStorage::EEPROM;
  }
  // Update the complete group in RAM before scheduling the ordinary snapshot.
  applyCandidateParameters();
  markParameterWrites(sd, ee);
  loadImuCalibration();
  return true;
}

void pollParameterService() {
  if (table && count && count <= MAX_PARAMETERS)
    pollParameterSaves();
  pollDeviceReboot(txLength != 0);
  if (deviceRebootPending() && !txLength)
    return;
  if (!table || count == 0 || count > MAX_PARAMETERS)
    return;
  if (!USBSerial) {
    // A reconnect starts with no stale request/response bytes.
    rxLength = txLength = 0;
    rxOverflow = listing = false;
    return;
  }
  if (txLength) {
    // A single line, one write, only when the USB buffer has enough room.
    if (USBSerial.availableForWrite() >= static_cast<int>(txLength)) {
      USBSerial.write(reinterpret_cast<const uint8_t *>(txLine), txLength);
      txLength = 0;
    }
    return;
  }
  if (listing) {
    if (listIndex < count) {
      const auto &p = table[listIndex];
      char value[VALUE_SIZE], low[VALUE_SIZE], high[VALUE_SIZE];
      formatValue(listIndex, snapshot[listIndex], value, sizeof(value));
      formatValue(listIndex, p.minimum, low, sizeof(low));
      formatValue(listIndex, p.maximum, high, sizeof(high));
      queueReply(listId, "VALUE\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", p.name, p.type(), value,
                 low, high, p.group, p.description,
                 p.storage == ParameterStorage::EEPROM ? "EEPROM" : "SD",
                 p.effect == ParameterEffect::Reboot ? "REBOOT" : "IMMEDIATE",
                 p.readOnly ? "READ_ONLY" : "EDITABLE");
      ++listIndex;
    } else {
      queueReply(listId, "END\t%u", static_cast<unsigned>(count));
      listing = false;
    }
    return;
  }
  if (!USBSerial.available() &&
      (sdNotice != StorageNotice::None || eeNotice != StorageNotice::None)) {
    auto &notice = eeNotice != StorageNotice::None ? eeNotice : sdNotice;
    queueReply(0, "STORAGE\t%s\t%s", eeNotice != StorageNotice::None ? "EEPROM" : "SD",
               notice == StorageNotice::Saved ? "SAVED" : "FAILED");
    notice = StorageNotice::None;
    return;
  }
  // No readStringUntil()/delay(): fragments arrive over successive cycles.
  for (unsigned budget = 0; budget < 64 && USBSerial.available(); ++budget) {
    int c = USBSerial.read();
    if (c == '\r')
      continue;
    if (c == '\n') {
      if (rxOverflow)
        error(0, "LINE_TOO_LONG", "Command exceeds 511 bytes");
      else {
        rxLine[rxLength] = '\0';
        handleCommand(rxLine);
      }
      rxLength = 0;
      rxOverflow = false;
      return;
    }
    if ((c < 32 && c != '\t') || c > 126) {
      rxOverflow = true;
      continue;
    }
    if (!rxOverflow && rxLength < sizeof(rxLine) - 1)
      rxLine[rxLength++] = c;
    else
      rxOverflow = true;
  }
}
