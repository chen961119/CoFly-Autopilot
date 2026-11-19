#pragma once

#include <MS4525.h>

class AirspeedSensor final : public MS4525read_Task {
public:
  explicit AirspeedSensor(TwoWire &bus) : MS4525read_Task(nullptr) {
    // The driver calls begin() for a non-null constructor argument. Startup owns this bus.
    MS4525_wire = &bus;
  }

  bool readFreshPressure(float &pressure);
  bool calibrateZero();
};
