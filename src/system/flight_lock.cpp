#include "config/aircraft_config.h"
#include "sensors/sensor_processing.h"
#include "sensors/sensor_calibration.h"
#include "system/flight_lock.h"
#include "parameters/parameter_service.h"
#include "system/device_reboot.h"

namespace {
bool flightLocked = true;
}

bool isFlightLocked() { return flightLocked || parameterRebootRequired() || deviceRebootPending(); }
void setFlightLocked(bool locked) {
  flightLocked = locked || parameterRebootRequired() || deviceRebootPending() ||
    !aircraftConfigurationValid() || !imuCalibrationValid() || sensorCalibrationBusy();
}
