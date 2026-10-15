#include "control/throttle_controller.h"
#include "control/control_output.h"

void ThrottleController::update(float throttleTarget) const {
  throttleControlOutput.command = throttleTarget;
}
