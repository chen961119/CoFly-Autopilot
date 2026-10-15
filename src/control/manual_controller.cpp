#include "control/manual_controller.h"
#include "control/control_output.h"

void ManualController::update(const MasterControlTargets &targets) const {
  manualControlOutput.roll = targets.roll / 2.0f;
  manualControlOutput.yaw = targets.yaw / 2.0f;
  manualControlOutput.throttle = targets.throttle;
  manualControlOutput.pitch = targets.pitch / 2.0f;
}
