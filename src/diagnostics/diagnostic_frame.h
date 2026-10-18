#pragma once

#include <Arduino.h>
#include "parameters/parameter_service.h"

// Build one complete diagnostic before offering it to the shared USB/X6 TX
// cursors. Oversized frames are dropped whole, never emitted as partial lines.
class DiagnosticFrame : public Print {
public:
  using Print::write;
  size_t write(uint8_t value) override {
    if (length >= sizeof(text) - 1) {
      overflow = true;
      return 0;
    }
    text[length++] = static_cast<char>(value);
    return 1;
  }
  bool send() {
    if (overflow || !length) return false;
    if (text[length - 1] != '\n') write('\n');
    return !overflow && queueDiagnosticOutput(text, length);
  }

private:
  char text[768];
  size_t length = 0;
  bool overflow = false;
};
