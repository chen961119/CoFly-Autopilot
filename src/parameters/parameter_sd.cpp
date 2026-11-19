#include "parameters/parameter_sd.h"
#include "logging/flight_storage.h"
#include "parameters/parameter_text.h"
#include "config/aircraft_config.h"
#include <SD.h>
#include <string.h>

namespace {
const char *const primaryPath = "params.cfg";
const char *const backupPath = "params_backup.cfg";
const FlightParameter *table;
size_t count;
size_t aircraftIdIndex;
const double *defaults;
bool storageReady;
const char *activePath;
bool fileSeen[PARAMETER_COUNT_LIMIT];
// Boot and storage task use these fixed buffers in sequence.
uint8_t fileData[ParameterText::FILE_CAPACITY];
double previous[PARAMETER_COUNT_LIMIT];
double verified[PARAMETER_COUNT_LIMIT];
ParameterSD::ValueValidator validValue;
using ParameterSD::MigrationStats;
// Restore shared text records by name/type; missing entries use defaults.
bool readParameterFile(const char *path, double *values, bool *needsMigration = nullptr,
                       MigrationStats *stats = nullptr) {
  if (needsMigration)
    *needsMigration = false;
  MigrationStats parsedStats{0, count, 0};
  File file = SD.open(path, FILE_READ);
  if (!file || !file.size() || file.size() >= sizeof(fileData))
    return false;
  const size_t length = file.size();
  const size_t read = file.read(fileData, length);
  file.close();
  ParameterText::View view;
  if (read != length || !ParameterText::valid(fileData, length, view) || view.used != length)
    return false;
  memcpy(values, defaults, count * sizeof(double));
  memset(fileSeen, 0, sizeof(fileSeen));
  size_t sdCount = 0;
  for (size_t i = 0; i < count; ++i)
    if (table[i].storage == ParameterStorage::SD)
      ++sdCount;
  if (needsMigration && (view.rows != sdCount || view.legacy ||
                         view.aircraftCount != static_cast<uint32_t>(aircraftCount())))
    *needsMigration = true;
  size_t offset = view.records;
  for (size_t row = 0; row < view.rows; ++row) {
    char line[PARAMETER_RECORD_BUFFER_SIZE], *name, *type;
    double value;
    if (!ParameterText::line(fileData, length, offset, line, sizeof(line)) ||
        !ParameterText::decode(line, name, type, value))
      return false;
    size_t index = 0;
    while (index < count && strcmp(name, table[index].name))
      ++index;
    if (index == count) {
      ++parsedStats.skipped;
      if (needsMigration)
        *needsMigration = true;
      continue;
    }
    if (strcmp(table[index].type(), type) || !validValue(index, value)) {
      ++parsedStats.skipped;
      if (needsMigration)
        *needsMigration = true;
    } else {
      fileSeen[index] = true;
      values[index] = value;
      ++parsedStats.matched;
      --parsedStats.defaultsUsed;
    }
  }
  if (needsMigration && aircraftIdIndex < count &&
      view.aircraftId != static_cast<uint32_t>(values[aircraftIdIndex]))
    *needsMigration = true;
  if (stats)
    *stats = parsedStats;
  return true;
}

bool sameSDValues(const double *a, const double *b) {
  for (size_t i = 0; i < count; ++i)
    if (table[i].storage == ParameterStorage::SD && a[i] != b[i])
      return false;
  return true;
}

// The Arduino File wrapper discards flush/close errors. Check the native
// SdFat result before reopening for schema, CRC and value verification.
bool writeFileData(const char *path, size_t length) {
  FsFile file = SD.sdfs.open(path, O_RDWR | O_CREAT);
  if (!file) return false;
  if (!file.truncate(0)) {
    file.close();
    return false;
  }
  const bool written = file.write(fileData, length) == length;
  const bool synced = written && file.sync();
  const bool closed = file.close();
  if (!synced || !closed) SD.remove(path);
  return synced && closed;
}

bool writeSnapshot(const char *path, const double *values) {
  size_t sdCount = 0;
  const uint32_t identity = aircraftIdIndex < count
      ? static_cast<uint32_t>(values[aircraftIdIndex]) : 0;
  for (size_t i = 0; i < count; ++i) {
    if (table[i].storage == ParameterStorage::SD)
      ++sdCount;
  }
  size_t used = ParameterText::begin(fileData, sizeof(fileData), sdCount, aircraftCount(), identity);
  if (!used)
    return false;
  for (size_t i = 0; i < count; ++i) {
    if (table[i].storage != ParameterStorage::SD)
      continue;
    const int n = ParameterRecord::encode(
        table[i], values[i], reinterpret_cast<char *>(fileData + used), sizeof(fileData) - used);
    if (n < 0)
      return false;
    used += n;
  }
  used = ParameterText::finish(fileData, sizeof(fileData), used);
  if (!used)
    return false;
  if (!writeFileData(path, used)) return false;
  if (!readParameterFile(path, verified) || !sameSDValues(values, verified)) {
    // Remove an incomplete replacement; the other validated file remains intact.
    SD.remove(path);
    return false;
  }
  return true;
}

// Keep the exact validated old file as a rollback copy during schema migration.
bool copyValidatedSnapshot(const char *sourcePath, const char *destinationPath,
                           const double *expectedValues) {
  File source = SD.open(sourcePath, FILE_READ);
  if (!source || source.size() == 0 || source.size() > sizeof(fileData))
    return false;
  const size_t length = source.size();
  const size_t read = source.read(reinterpret_cast<uint8_t *>(fileData), length);
  source.close();
  if (read != length)
    return false;
  if (!writeFileData(destinationPath, length)) return false;
  if (!readParameterFile(destinationPath, verified) ||
      memcmp(expectedValues, verified, count * sizeof(double))) {
    SD.remove(destinationPath);
    return false;
  }
  return true;
}

bool saveSnapshot(const double *values, bool preserveStoredFile) {
  if (!storageReady)
    return false;
  // Preserve the last successfully persisted values before replacing the primary.
  // When boot recovered from backup, leave that backup intact.
  if (activePath) {
    if (!readParameterFile(activePath, previous))
      return false;
    if (activePath != backupPath &&
        !(preserveStoredFile ? copyValidatedSnapshot(activePath, backupPath, previous)
                             : writeSnapshot(backupPath, previous)))
      return false;
  }
  if (!writeSnapshot(primaryPath, values)) {
    // The primary may have been truncated; retry from the preserved backup.
    if (activePath)
      activePath = backupPath;
    return false;
  }
  activePath = primaryPath;
  return true;
}

} // namespace

namespace ParameterSD {
void initialize(bool ready, const FlightParameter *parameters, size_t size,
                const double *defaultValues, ValueValidator validator) {
  table = parameters;
  count = size;
  aircraftIdIndex = findParameterIndex(table, count, "aircraft_id");
  defaults = defaultValues;
  validValue = validator;
  storageReady = ready && table && defaults && validValue && count <= PARAMETER_COUNT_LIMIT;
  activePath = nullptr;
  memset(fileSeen, 0, sizeof(fileSeen));
}

bool load(double *values, bool &needsMigration, MigrationStats &stats) {
  if (!storageReady)
    return false;
  if (readParameterFile(primaryPath, values, &needsMigration, &stats))
    activePath = primaryPath;
  else if (readParameterFile(backupPath, values, &needsMigration, &stats))
    activePath = backupPath;
  return activePath != nullptr;
}

bool save(const double *values, bool preserveStoredFile) {
  struct Request { const double *values; bool preserve; } request{values, preserveStoredFile};
  return runStorageOperation([](void *context) {
    auto &r = *static_cast<Request *>(context);
    return saveSnapshot(r.values, r.preserve);
  }, &request);
}

bool hasRecord(size_t index) {
  return index < count && fileSeen[index];
}
} // namespace ParameterSD
