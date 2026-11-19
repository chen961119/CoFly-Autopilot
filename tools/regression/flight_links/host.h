#pragma once
// The replay owns simulated time, serial FIFO contents and recorded PWM edges.
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <vector>

inline uint32_t replayUs = 0;
inline uint32_t micros() { return replayUs; }
inline uint32_t millis() { return replayUs / 1000; }
inline void delay(unsigned long ms) { replayUs += ms * 1000; }
constexpr double PI = 3.141592653589793;
constexpr int SERIAL_8E2_RXINV_TXINV = 1;
#define constrain(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))
inline int digitalRead(int) { return 0; }
class Print {
public:
  virtual size_t write(uint8_t) = 0;
  template<typename... T> void print(T...) {}
  template<typename... T> void println(T...) {}
  template<typename... T> void printf(T...) {}
};
class Stream {
public:
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  int freeBytes = 256;
  int available() const { return static_cast<int>(rx.size()); }
  int read() { assert(!rx.empty()); int b = rx.front(); rx.pop_front(); return b; }
  int availableForWrite() const { return freeBytes; }
  size_t write(const uint8_t *data, size_t size) {
    assert(size <= static_cast<size_t>(freeBytes));
    tx.insert(tx.end(), data, data + size);
    return size;
  }
  void begin(unsigned long, int = 0) {}
  void addMemoryForRead(void *, size_t) {}
  void addMemoryForWrite(void *, size_t) {}
};
using HardwareSerial = Stream;
inline HardwareSerial Serial2, Serial3, Serial5;
inline std::array<int, 7> pwm{};
class Servo {
  unsigned pin = 0;
public:
  void attach(unsigned p, int, int) { pin = p; }
  void writeMicroseconds(int value) { pwm.at(pin) = value; }
};
class SPISettings {};
