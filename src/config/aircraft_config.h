#pragma once
#include <stdint.h>

// 总数量是编译期配置，可直接作为数组长度；同一构型各机共用固件，仅编号进参数表。
#ifndef COFLY_AIRCRAFT_COUNT
#define COFLY_AIRCRAFT_COUNT 3
#endif
static_assert(COFLY_AIRCRAFT_COUNT == 1 || COFLY_AIRCRAFT_COUNT == 3 ||
                  COFLY_AIRCRAFT_COUNT == 4 || COFLY_AIRCRAFT_COUNT == 5 ||
                  COFLY_AIRCRAFT_COUNT == 7,
              "COFLY_AIRCRAFT_COUNT must be 1, 3, 4, 5 or 7");

// Identity is an EEPROM boot parameter; active identity is latched.
extern int32_t aircraftIdParameter;
void activateAircraftConfiguration();
bool aircraftConfigurationValid();
int aircraftId();
constexpr int aircraftCount() { return COFLY_AIRCRAFT_COUNT; }
constexpr bool aircraftIsSingle() { return COFLY_AIRCRAFT_COUNT == 1; }
bool aircraftIsLeft();
bool aircraftIsRight();
bool validAircraftConfiguration(int id);
const char *aircraftName();
