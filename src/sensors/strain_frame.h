#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

inline uint16_t modbusCrc16(const uint8_t *data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc & 1u) ? (crc >> 1) ^ 0xa001u : crc >> 1;
  }
  return crc;
}

// Serial7 returns five signed 32-bit values, low Modbus register first.
class StrainFrameDecoder {
public:
  bool push(uint8_t byte, int32_t (&channels)[5]) {
    bytes[used++] = byte;
    while (used && !validPrefix()) {
      --used;
      memmove(bytes, bytes + 1, used);
    }
    if (used != sizeof(bytes)) return false;
    for (unsigned i = 0; i < 5; ++i) {
      const uint8_t *p = bytes + 3 + i * 4;
      const uint32_t raw = (uint32_t(p[2]) << 24) | (uint32_t(p[3]) << 16) |
                           (uint32_t(p[0]) << 8) | p[1];
      memcpy(&channels[i], &raw, sizeof(raw));
    }
    used = 0;
    return true;
  }

private:
  bool validPrefix() const {
    return bytes[0] == 1 && (used < 2 || bytes[1] == 3) && (used < 3 || bytes[2] == 20) &&
           (used < sizeof(bytes) || modbusCrc16(bytes, 23) == (bytes[23] | (bytes[24] << 8)));
  }
  uint8_t bytes[25]{};
  size_t used = 0;
};
