#include "control/control_output.h"
#include "control/roll_damping_controller.h"
#include "parameters/control_parameters.h"
#include "sensors/sensor_processing.h"
#include "config/flight_config.h"
using namespace ControlParameters;

void RollDampingController::update() const {
  rollDampingControlOutput.rateTarget = 0.0f;
  rollDampingControlOutput.dampingCommand = - k_Clp * ang_vel[0];
}
