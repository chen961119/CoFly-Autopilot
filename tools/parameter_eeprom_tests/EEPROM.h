#pragma once
#include <array>
#include <stdint.h>
#include <stdexcept>

struct PowerCut {};
struct MockEEPROM {
  std::array<uint8_t,4284> bytes;
  int writes=0, cut=-1, fail=-1, capacity=4284;
  int length() const { return capacity; }
  uint8_t read(int address) const { return bytes.at(address); }
  void update(int address,uint8_t value) {
    if (writes==cut) throw PowerCut{};
    if (writes++==fail) return;
    bytes.at(address)=value;
  }
  void resetFaults() { writes=0;cut=fail=-1; }
};
extern MockEEPROM EEPROM;
