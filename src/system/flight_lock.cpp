#include "config/aircraft_config.h"
#include "sensors/sensor_processing.h"
#include "sensors/sensor_calibration.h"
#include "system/flight_lock.h"
#include "parameters/parameter_service.h"
#include "system/device_reboot.h"
#include "system/rtos_sync.h"

namespace {
bool flightLocked = true;
}

bool isFlightLocked() {
  TaskCriticalSection guard;
  return flightLocked || parameterRebootRequired() || deviceRebootPending();
}
void setFlightLocked(bool locked) {
  TaskCriticalSection guard;
  flightLocked = locked || parameterRebootRequired() || deviceRebootPending() ||
    !aircraftConfigurationValid() || !imuCalibrationValid() || sensorCalibrationBusy() || parameterMaintenanceBusy();
}
