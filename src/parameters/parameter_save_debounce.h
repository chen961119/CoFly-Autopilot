#pragma once
#include <stdint.h>

// One instance per medium; uint32 subtraction also handles millis() wraparound.
struct ParameterSaveDebounce {
  static constexpr uint32_t delayMs = 2000;
  bool dirty = false;
  uint32_t since = 0;
  void changed(uint32_t now) {
    dirty = true;
    since = now;
  }
  bool due(uint32_t now) const { return dirty && static_cast<uint32_t>(now - since) >= delayMs; }
  void completed(bool success, uint32_t now) {
    dirty = !success;
    since = now; // A failed attempt retries after two seconds, never every loop.
  }
};
