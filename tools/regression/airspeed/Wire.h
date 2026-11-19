#pragma once

#include <Arduino.h>
#include <array>
#include <cassert>
#include <vector>

class TwoWire {
public:
  struct Reply {
    uint16_t counts;
    uint8_t status;
    uint8_t length;
  };
  std::vector<Reply> replies;
  unsigned starts = 0, requests = 0;
  unsigned readTimeMs = 1;

  void begin() { ++starts; }
  uint8_t requestFrom(uint8_t address, uint8_t size) {
    assert(address == 0x28 && size == 4 && requests < 10000);
    const Reply reply = requests < replies.size() ? replies[requests] : Reply{};
    ++requests;
    delay(readTimeMs);
    bytes = {static_cast<uint8_t>((reply.status << 6) | (reply.counts >> 8)),
             static_cast<uint8_t>(reply.counts), 0, 0};
    offset = 0;
    length = reply.length;
    return length;
  }
  int available() const { return length - offset; }
  int read() {
    assert(available() > 0);
    return bytes[offset++];
  }

private:
  std::array<uint8_t, 4> bytes{};
  unsigned offset = 0, length = 0;
};
