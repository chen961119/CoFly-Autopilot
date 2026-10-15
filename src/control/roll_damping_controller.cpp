#include "control/control_output.h"
#include "control/roll_damping_controller.h"
#include "parameters/control_parameters.h"
#include "sensors/sensor_processing.h"
#include "config/flight_config.h"
using namespace ControlParameters;

void RollDampingController::update() const {
  const float Clp_PID = .01 * k_Clp * (0 - ang_vel[0]);
  rollDampingControlOutput.command = Clp_PID * 125.0;
}
