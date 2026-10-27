#include "parameters/parameter_service.h"
#include "ground_station_service/ground_station_service.h"
#include "ground_station_service/sd_file_service.h"
#include "config/serial_config.h"
#include "config/aircraft_config.h"
#include "parameters/imu_calibration_integrity.h"
#include "parameters/parameter_eeprom.h"
#include "parameters/parameter_indices.h"
#include "parameters/parameter_registry.h"
#include "parameters/actuator_parameter_validation.h"
#include "parameters/parameter_save_debounce.h"
#include "parameters/parameter_sd.h"
#include "parameters/parameter_text.h"
#include "sensors/sensor_processing.h"
#include "system/device_reboot.h"
#include "system/flight_lock.h"
#include "radio_comm/radio_comm.h"
#include <Arduino.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr size_t MAX_PARAMETERS = PARAMETER_COUNT_LIMIT;
constexpr size_t LINE_SIZE = 768;
constexpr size_t VALUE_SIZE = PARAMETER_VALUE_SIZE;

bool eepromLoaded[MAX_PARAMETERS];

const FlightParameter *table;
size_t count;
ParameterIndices indices;
bool storageReady;
bool rebootRequired = false;
enum class StorageNotice { None, Saved, Failed };

// Each physical link owns its parser, reply cursor and parameter snapshot.
// No USB traffic is forwarded onto the radio (or vice versa).
struct ParameterSession {
  char rxLine[LINE_SIZE] = {};
  size_t rxLength = 0;
  bool rxOverflow = false;
  uint32_t lastRxTime = 0;
  char txLine[SD_FILE_RESPONSE_SIZE] = {};
  size_t txLength = 0;
  size_t txOffset = 0;
  size_t listIndex = 0;
  bool listing = false;
  uint32_t listId = 0;
  double snapshot[MAX_PARAMETERS] = {};
  StorageNotice sdNotice = StorageNotice::None;
  StorageNotice eeNotice = StorageNotice::None;

  void reset();
  void queueReply(uint32_t id, const char *format, ...);
  void error(uint32_t id, const char *code, const char *message);
  void handleCommand(char *line);
  void poll(Stream &port);
  bool queueDiagnostic(Stream &port, const char *text, size_t length);
};
ParameterSession usbSession, radioSession;

void storageNotice(ParameterStorage medium, StorageNotice notice) {
  ParameterSession *sessions[] = {&usbSession, &radioSession};
  for (auto *session : sessions) {
    if (medium == ParameterStorage::SD) session->sdNotice = notice;
    else session->eeNotice = notice;
  }
}
double defaults[MAX_PARAMETERS];
// Main-loop-only, non-reentrant service; snapshots stay out of the stack.
double candidate[MAX_PARAMETERS];
ParameterSaveDebounce sdSave, eeSave;
const char *loadState = "DEFAULTS";

ParameterSD::MigrationStats loadStats;

using ParameterRecord::parseUnsigned;
bool validValue(size_t index, double value) {
  return validParameterValue(table[index], value);
}
bool parseValue(size_t index, const char *text, double &value) {
  return ParameterRecord::parseValue(table[index], text, value) && validValue(index, value);
}
void formatValue(size_t index, double value, char *text, size_t capacity) {
  ParameterRecord::formatValue(table[index], value, text, capacity);
}

bool retireMigratedSources(const double *values) {
  for (size_t i = 0; i < count; ++i)
    if (table[i].storage == ParameterStorage::SD && hasEEPROMMigrationSource(table[i]))
      return saveEEPROMParameters(values, true);
  return true;
}

bool persist(const double *values, ParameterStorage medium) {
  if (medium == ParameterStorage::EEPROM)
    return saveEEPROMParameters(values);
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
  if (!pending.dirty)
    return true;
  if (medium == ParameterStorage::SD && sdFileSessionActive()) return false;
  for (size_t i = 0; i < count; ++i)
    candidate[i] = table[i].read();
  const bool success = persist(candidate, medium);
  pending.completed(success, millis());
  storageNotice(medium, success ? StorageNotice::Saved : StorageNotice::Failed);
  return success;
}

void pollParameterSaves() {
  if (eeSave.due(millis()))
    flushMedium(ParameterStorage::EEPROM);
  if (!sdFileSessionActive() && sdSave.due(millis()))
    flushMedium(ParameterStorage::SD);
}

void markParameterWrites(bool sd, bool ee) {
  const uint32_t now = millis();
  if (sd) {
    sdSave.changed(now);
    storageNotice(ParameterStorage::SD, StorageNotice::None);
  }
  if (ee) {
    eeSave.changed(now);
    storageNotice(ParameterStorage::EEPROM, StorageNotice::None);
  }
}

bool requiresFlightLock(const FlightParameter &parameter) {
  return parameter.storage == ParameterStorage::EEPROM;
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
  if (!validActuatorPwmParameters(table, count, candidate, indices.actuators))
    return SaveResult::Rejected;
  const size_t idIndex = indices.aircraftId;
  if (idIndex < count && !validAircraftConfiguration(candidate[idIndex]))
    return SaveResult::Rejected;
  const bool calibrationOffset = indices.isImuOffset(index);
  const size_t marker = indices.imuValid;
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

void ParameterSession::queueReply(uint32_t id, const char *format, ...) {
  txOffset = 0;
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

void ParameterSession::error(uint32_t id, const char *code, const char *message) {
  queueReply(id, "ERROR\t%s\t%s", code, message);
}

void ParameterSession::handleCommand(char *line) {
  if (!strncmp(line, "SD ", 3) || !strncmp(line, "SD\t", 3)) {
    if (this != &usbSession) {
      error(0, "USB_REQUIRED", "SD files require USB");
      return;
    }
    handleSdFileCommand(line, txLine, sizeof(txLine));
    txLength = strlen(txLine);
    return;
  }
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
    const size_t index = findParameterIndex(table, count, tokens[3]);
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
      error(id, "LOCK_REQUIRED", "Lock aircraft before changing EEPROM parameters");
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
  usbSession.reset();
  radioSession.reset();
  table = flightParameterTable(count);
  storageReady = sdReady && count > 0 && count <= MAX_PARAMETERS;
  if (!count || count > MAX_PARAMETERS) {
    count = 0;
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    if (!validParameterName(table[i].name) ||
        !validValue(i, table[i].defaultValue)) {
      count = 0;
      return;
    }
    for (size_t j = 0; j < i; ++j)
      if (!strcmp(table[j].name, table[i].name)) {
        count = 0;
        return;
      }
  }
  if (!indices.initialize(table, count)) {
    count = 0;
    storageReady = false;
    return;
  }
  // A complete calibration is stored in one snapshot. Moving the whole group
  // between SD and EEPROM is supported; splitting it across media is not.
  const auto calibrationStorage = table[indices.imuValid].storage;
  // Cached positions remain valid regardless of registry row order.
  for (unsigned member = 0; member < IMU_CALIBRATION_OFFSET_COUNT + 3; ++member) {
    const size_t index = indices.imuMember(member);
    if (index == count || table[index].storage != calibrationStorage) {
      USBSerial.printf("[PARAM] IMU calibration parameters must share one storage medium\n");
      count = 0;
      storageReady = false;
      return;
    }
  }
  loadState = "DEFAULTS";
  loadStats = {0, count, 0};
  const bool haveEEPROM = initializeEEPROMParameters(table, count);
  for (size_t i = 0; i < count; ++i) {
    defaults[i] = table[i].defaultValue;
    eepromLoaded[i] = readEEPROMParameter(table[i], defaults[i]);
    if (!validValue(i, defaults[i])) {
      defaults[i] = table[i].defaultValue;
      eepromLoaded[i] = false;
    }
  }
  for (size_t i = 0; i < count; ++i)
    candidate[i] = table[i].defaultValue;
  if (!validActuatorPwmParameters(table, count, candidate, indices.actuators)) {
    count = 0;
    storageReady = false;
    return;
  }
  for (size_t i = 0; i < count; ++i)
    table[i].write(defaults[i]);
  bool needsMigration = false;
  ParameterSD::initialize(storageReady, table, count, defaults, validValue);
  const bool loadedSD = ParameterSD::load(candidate, needsMigration, loadStats);
  if (!loadedSD)
    memcpy(candidate, defaults, count * sizeof(double));
  // Individual SD values may be in range while a min/max pair is reversed
  // or a servo center falls outside its endpoints.
  // Restore the PWM-limit group only; retain all other compatible parameters.
  if (!validActuatorPwmParameters(table, count, candidate, indices.actuators)) {
    for (size_t i = 0; i < count; ++i)
      if (indices.isActuatorLimit(i))
        candidate[i] = table[i].defaultValue;
    needsMigration = true;
    USBSerial.printf("[PARAM] Invalid actuator PWM limits; restored default limits\n");
  }
  bool completeCalibration = true;
  for (size_t i = 0; i < count; ++i) {
    if (table[i].storage == ParameterStorage::EEPROM && eepromLoaded[i])
      candidate[i] = defaults[i];
    if (indices.isImuCalibration(i) && !eepromLoaded[i] &&
        !ParameterSD::hasRecord(i))
      completeCalibration = false;
    if (indices.isImuCalibration(i) &&
        table[i].storage == ParameterStorage::SD && !ParameterSD::hasRecord(i) &&
        !hasEEPROMMigrationSource(table[i]))
      completeCalibration = false;
  }
  const size_t calibrationMarker = indices.imuValid;
  if (completeCalibration) {
    float offsets[IMU_CALIBRATION_OFFSET_COUNT];
    for (unsigned i = 0; i < IMU_CALIBRATION_OFFSET_COUNT; ++i)
      offsets[i] = candidate[indices.imuOffsets[i]];
    if (candidate[indices.imuChecksum] !=
        imuCalibrationFingerprint(offsets, candidate[indices.imuModel]))
      completeCalibration = false;
  }
  if (!completeCalibration && calibrationMarker < count)
    candidate[calibrationMarker] = 0;
  const size_t idIndex = indices.aircraftId;
  if (table[idIndex].storage == ParameterStorage::SD && !ParameterSD::hasRecord(idIndex) &&
      !hasEEPROMMigrationSource(table[idIndex]))
    candidate[idIndex] = 0;
  if (!validAircraftConfiguration(candidate[idIndex])) {
    candidate[idIndex] = 0;
  }
  const bool eeSaved = saveEEPROMParameters(candidate);
  if (!eeSaved) {
    candidate[idIndex] = 0;
    candidate[indices.imuValid] = 0;
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

bool saveParameterValue(const char *name, double value) {
  return saveOne(findParameterIndex(table, count, name), value) != SaveResult::Rejected;
}
bool saveRadioCalibrationParameters(const char *const *values) {
  if (sdFileSessionActive() || !table || !count || !storageReady || !values || !isFlightLocked() || !radioInputValid())
    return false;
  for (size_t i = 0; i < count; ++i) candidate[i] = table[i].read();
  for (unsigned channel = 0; channel < 4; ++channel) {
    double group[4];
    for (unsigned field = 0; field < 4; ++field) {
      const size_t index = indices.radioCalibration[channel][field];
      if (index == count || table[index].readOnly || table[index].storage != ParameterStorage::SD ||
          !parseValue(index, values[channel * 4 + field], group[field])) return false;
      candidate[index] = group[field];
    }
    const float deadzone = radioChannelParameters[channel].deadzone;
    if (group[1] - group[0] < 400 || group[2] - group[0] <= deadzone ||
        group[1] - group[2] <= deadzone) return false;
  }
  // Write and read back the complete snapshot before changing live values.
  if (!ParameterSD::save(candidate)) return false;
  applyCandidateParameters();
  sdSave.completed(true, millis());
  storageNotice(ParameterStorage::SD, StorageNotice::Saved);
  loadState = "SAVED";
  updateRadioNormalized();
  return true;
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

bool parameterWritesPending() { return sdSave.dirty || eeSave.dirty; }

bool saveImuCalibrationParameters(const float *offsets) {
  if (!table || !count || !offsets || !isFlightLocked())
    return false;
  for (size_t i = 0; i < count; ++i)
    candidate[i] = table[i].read();
  for (unsigned j = 0; j < IMU_CALIBRATION_OFFSET_COUNT; ++j) {
    const size_t i = indices.imuOffsets[j];
    if (i == count || !validValue(i, offsets[j]))
      return false;
    candidate[i] = offsets[j];
  }
  const size_t marker = indices.imuValid, model = indices.imuModel,
               fingerprint = indices.imuChecksum;
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

namespace {
void ParameterSession::reset() {
  rxLength = txLength = txOffset = listIndex = 0;
  rxOverflow = listing = false;
  sdNotice = eeNotice = StorageNotice::None;
}

bool ParameterSession::queueDiagnostic(Stream &port, const char *text, size_t length) {
  // A reply may span multiple control cycles. Share its cursor so a diagnostic
  // can never be inserted in the middle of a protocol frame. Incoming commands,
  // parameter listings and storage notices take priority over new diagnostics.
  if (txLength || listing || rxLength || rxOverflow || port.available() ||
      sdNotice != StorageNotice::None || eeNotice != StorageNotice::None)
    return false;
  memcpy(txLine, text, length);
  txLength = length;
  txOffset = 0;
  return true;
}

void ParameterSession::poll(Stream &port) {
  if (!table || count == 0 || count > MAX_PARAMETERS) return;
  // Drop abandoned partial commands after a radio interruption. Complete
  // commands retain the same validation/locking as the USB service.
  if ((rxLength || rxOverflow) && static_cast<uint32_t>(millis() - lastRxTime) > 2000) {
    rxLength = 0;
    rxOverflow = false;
  }
  if (txLength) {
    // UART's normal TX buffer is smaller than a DATA frame. Never wait for
    // an entire line to fit, and never block the 500 Hz loop in write().
    const int available = port.availableForWrite();
    if (available > 0) {
      size_t amount = txLength - txOffset;
      if (amount > static_cast<size_t>(available)) amount = available;
      const size_t budget = this == &usbSession && sdFileSessionActive() ? 2048 : 64;
      if (amount > budget) amount = budget;
      txOffset += port.write(reinterpret_cast<const uint8_t *>(txLine) + txOffset, amount);
      if (txOffset == txLength) txLength = txOffset = 0;
    }
    return;
  }
  if (deviceRebootPending()) return;
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
  if (!port.available() &&
      (sdNotice != StorageNotice::None || eeNotice != StorageNotice::None)) {
    auto &notice = eeNotice != StorageNotice::None ? eeNotice : sdNotice;
    queueReply(0, "STORAGE\t%s\t%s", eeNotice != StorageNotice::None ? "EEPROM" : "SD",
               notice == StorageNotice::Saved ? "SAVED" : "FAILED");
    notice = StorageNotice::None;
    return;
  }
  // No readStringUntil()/delay(): fragments arrive over successive cycles.
  for (unsigned budget = 0; budget < 64 && port.available(); ++budget) {
    int c = port.read();
    if (c < 0) break;
    lastRxTime = millis();
    if (c == '\r')
      continue;
    if (c == '\n') {
      if (rxOverflow)
        error(0, "LINE_TOO_LONG", "Command exceeds 767 bytes");
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

} // namespace

void pollParameterService() {
  pollSdFileService(static_cast<bool>(USBSerial));
  if (table && count && count <= MAX_PARAMETERS) pollParameterSaves();
  if (!USBSerial) usbSession.reset();
  // A radio reply must drain before the existing delayed reboot starts.
  pollDeviceReboot(usbSession.txLength != 0 || radioSession.txLength != 0);
  if (USBSerial) usbSession.poll(USBSerial);
}

void pollTelemetryParameterService() {
  radioSession.poll(TelemetrySerial);
}

bool queueDiagnosticOutput(const char *text, size_t length) {
  if (!table || !count || count > MAX_PARAMETERS || !text || !length ||
      length >= LINE_SIZE || text[length - 1] != '\n' || deviceRebootPending())
    return false;
  const bool usb = USBSerial && usbSession.queueDiagnostic(USBSerial, text, length);
  const bool radio = radioSession.queueDiagnostic(TelemetrySerial, text, length);
  return usb || radio;
}
