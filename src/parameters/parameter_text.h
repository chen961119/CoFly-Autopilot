#pragma once
#include "parameters/parameter_record.h"

// Identical ASCII/LF container for SD, EEPROM and PC. No generation or profile.
namespace ParameterText {
constexpr const char *VERSION = "COFLY_PARAMS";
constexpr const char *LEGACY_VERSION = "COFLY_PARAMS_V8";
constexpr size_t MAX_ROWS = PARAMETER_COUNT_LIMIT;
constexpr size_t FOOTER = 15;
constexpr size_t FILE_CAPACITY = MAX_ROWS * PARAMETER_RECORD_BUFFER_SIZE + 256;
struct View {
  size_t rows, records, used;
  uint32_t aircraftCount, aircraftId;
  bool legacy; // V8 or the older three-line aircraft header; migrate on save.
};
bool validAircraftMetadata(uint32_t aircraftCount, uint32_t aircraftId);
uint32_t checksum(const uint8_t *data, size_t size);
bool line(const uint8_t *data, size_t available, size_t &offset, char *out, size_t capacity);
bool header(const uint8_t *data, size_t available, View &view);
bool decode(char *line, char *&name, char *&type, double &value);
bool valid(const uint8_t *data, size_t available, View &view);
size_t begin(uint8_t *data, size_t capacity, size_t rows,
             uint32_t aircraftCount, uint32_t aircraftId);
size_t finish(uint8_t *data, size_t capacity, size_t body);
} // namespace ParameterText
