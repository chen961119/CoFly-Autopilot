#include "parameters/parameter_text.h"
#include <stdio.h>

namespace ParameterText {
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
  if (!line(data, available, offset, text, sizeof(text)) || strcmp(text, VERSION))
    return false;
  uint32_t rows;
  if (!line(data, available, offset, text, sizeof(text)) || strncmp(text, "COUNT=", 6) ||
      !ParameterRecord::parseUnsigned(text + 6, rows) || rows > MAX_ROWS)
    return false;
  view = {rows, offset, 0};
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
size_t begin(uint8_t *data, size_t capacity, size_t rows) {
  if (rows > MAX_ROWS)
    return 0;
  const int n = snprintf(reinterpret_cast<char *>(data), capacity, "%s\nCOUNT=%u\n", VERSION,
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
