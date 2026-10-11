#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
extern uint32_t testTime;
inline uint32_t millis() { return testTime; }
struct MockSerial {
  bool connected = false;
  std::string input, output;
  explicit operator bool() const { return connected; }
  int available() { return static_cast<int>(input.size()); }
  int availableForWrite() { return 4096; }
  int read() { const char c=input.front(); input.erase(0,1); return c; }
  size_t write(const uint8_t *data, size_t size) { output.append(reinterpret_cast<const char *>(data),size); return size; }
  void printf(const char *, ...) {}
};
extern MockSerial Serial;
