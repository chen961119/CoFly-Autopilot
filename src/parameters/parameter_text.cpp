#include "parameters/parameter_text.h"
#include <stdio.h>

namespace ParameterText {
bool validAircraftMetadata(uint32_t aircraftCount, uint32_t aircraftId) {
  return (aircraftCount == 1 || aircraftCount == 3 || aircraftCount == 4 ||
          aircraftCount == 5 || aircraftCount == 7) && aircraftId <= aircraftCount;
}
uint32_t checksum(const uint8_t *data, size_t size) {
  uint32_t value = UINT32_MAX;
  for (size_t i = 0; i < size; ++i) {
    value ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      value = value >> 1 ^ (0xedb88320u & (0u - (value & 1u)));
  }
  return ~value;
}
bool line(const uint8_t *data, size_t available, size_t &offset, char *out, size_t capacity) {
  size_t n = 0;
  while (offset < available && data[offset] != '\n') {
    const uint8_t c = data[offset++];
    if (c < 32 || c > 126 || n + 1 >= capacity)
      return false;
    out[n++] = c;
  }
  if (offset == available)
    return false;
  ++offset;
  out[n] = 0;
  return true;
}
bool header(const uint8_t *data, size_t available, View &view) {
  size_t offset = 0;
  char text[PARAMETER_RECORD_BUFFER_SIZE];
  if (!line(data, available, offset, text, sizeof(text)))
    return false;
  const bool v8 = !strcmp(text, LEGACY_VERSION);
  // Both older layouts are readable and rewritten with a single-line header.
  const bool legacy = v8 || !strcmp(text, VERSION);
  uint32_t aircraftCount = 0, aircraftId = 0;
  if (!legacy) {
    constexpr char prefix[] = "COFLY_PARAMS AIRCRAFT_COUNT=";
    if (strncmp(text, prefix, sizeof(prefix) - 1))
      return false;
    char *total = text + sizeof(prefix) - 1;
    char *identity = strstr(total, " AIRCRAFT_ID=");
    if (!identity)
      return false;
    *identity = '\0';
    if (!ParameterRecord::parseUnsigned(total, aircraftCount) ||
        !ParameterRecord::parseUnsigned(identity + 13, aircraftId))
      return false;
  } else if (!v8) {
    if (!line(data, available, offset, text, sizeof(text)) || strncmp(text, "AIRCRAFT_COUNT=", 15) ||
        !ParameterRecord::parseUnsigned(text + 15, aircraftCount) ||
        !line(data, available, offset, text, sizeof(text)) || strncmp(text, "AIRCRAFT_ID=", 12) ||
        !ParameterRecord::parseUnsigned(text + 12, aircraftId))
      return false;
  }
  if (!v8 && !validAircraftMetadata(aircraftCount, aircraftId))
    return false;
  uint32_t rows;
  if (!line(data, available, offset, text, sizeof(text)) || strncmp(text, "COUNT=", 6) ||
      !ParameterRecord::parseUnsigned(text + 6, rows) || rows > MAX_ROWS)
    return false;
  view = {rows, offset, 0, aircraftCount, aircraftId, legacy};
  for (size_t i = 0; i < rows; ++i)
    if (!line(data, available, offset, text, sizeof(text)))
      return false;
  if (available - offset < FOOTER || memcmp(data + offset, "CRC32=", 6) ||
      data[offset + 14] != '\n')
    return false;
  for (size_t i = 6; i < 14; ++i) {
    const auto c = data[offset + i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')))
      return false;
  }
  view.used = offset + FOOTER;
  return true;
}
bool decode(char *line, char *&name, char *&type, double &value) {
  char *text;
  if (!ParameterRecord::split(line, name, type, text))
    return false;
  float floating = 0;
  int32_t integer = 0;
  FlightParameter p(0, name, &floating, -3.4028234663852886e38, 3.4028234663852886e38, "", "");
  if (!strcmp(type, "int")) {
    p.integer = &integer;
    p.minimum = INT32_MIN;
    p.maximum = INT32_MAX;
  }
  return ParameterRecord::parseValue(p, text, value);
}
bool valid(const uint8_t *data, size_t available, View &view) {
  if (!header(data, available, view))
    return false;
  const size_t footer = view.used - FOOTER;
  uint32_t expected = 0;
  for (size_t i = footer + 6; i < footer + 14; ++i) {
    const char c = data[i];
    const unsigned digit = c <= '9' ? c - '0' : (c <= 'F' ? c - 'A' + 10 : c - 'a' + 10);
    expected = expected << 4 | digit;
  }
  if (checksum(data, footer) != expected)
    return false;
  size_t offset = view.records;
  for (size_t i = 0; i < view.rows; ++i) {
    char record[PARAMETER_RECORD_BUFFER_SIZE], *name, *type;
    double value;
    if (!line(data, footer, offset, record, sizeof(record)) || !decode(record, name, type, value))
      return false;
    size_t previous = view.records;
    for (size_t j = 0; j < i; ++j) {
      char other[PARAMETER_RECORD_BUFFER_SIZE], *n, *t, *v;
      if (!line(data, footer, previous, other, sizeof(other)) ||
          !ParameterRecord::split(other, n, t, v) || !strcmp(n, name))
        return false;
    }
  }
  return offset == footer;
}
size_t begin(uint8_t *data, size_t capacity, size_t rows,
             uint32_t aircraftCount, uint32_t aircraftId) {
  if (rows > MAX_ROWS || !validAircraftMetadata(aircraftCount, aircraftId))
    return 0;
  const int n = snprintf(reinterpret_cast<char *>(data), capacity,
                         "%s AIRCRAFT_COUNT=%u AIRCRAFT_ID=%u\nCOUNT=%u\n", VERSION,
                         static_cast<unsigned>(aircraftCount), static_cast<unsigned>(aircraftId),
                         static_cast<unsigned>(rows));
  return n > 0 && static_cast<size_t>(n) < capacity ? n : 0;
}
size_t finish(uint8_t *data, size_t capacity, size_t body) {
  if (body + FOOTER + 1 > capacity)
    return 0;
  const int n = snprintf(reinterpret_cast<char *>(data + body), capacity - body, "CRC32=%08lx\n",
                         static_cast<unsigned long>(checksum(data, body)));
  return n == FOOTER ? body + FOOTER : 0;
}
} // namespace ParameterText
