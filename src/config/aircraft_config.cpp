#include "config/aircraft_config.h"

int32_t aircraftIdParameter = 0; // Unconfigured, never silently boot as A.
namespace { int activeId = 0; }
bool validAircraftConfiguration(int id) {
  return id >= 0 && id <= COFLY_AIRCRAFT_COUNT;
}
void activateAircraftConfiguration() {
  activeId = validAircraftConfiguration(aircraftIdParameter) ? aircraftIdParameter : 0;
}
bool aircraftConfigurationValid() { return activeId > 0; }
int aircraftId() { return activeId; }
bool aircraftIsLeft() { return activeId == 2 || activeId == 4 || activeId == 6; }
bool aircraftIsRight() { return activeId == 3 || activeId == 5 || activeId == 7; }
const char *aircraftName() {
  static const char *const names[] = {"UNCONFIGURED", "A", "B", "C", "D", "E", "F", "G"};
  return names[activeId];
}
