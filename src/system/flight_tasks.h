#pragma once
#include <stdint.h>

// Minimum unused stack since boot, in 32-bit words. Read from diagnostics only.
struct FlightStackMargins {
  uint32_t control, links, auxiliary, parameters, storage;
};
FlightStackMargins flightStackMargins();
