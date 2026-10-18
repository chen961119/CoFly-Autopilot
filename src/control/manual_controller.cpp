#include "control/manual_controller.h"
#include "control/control_output.h"

void ManualController::update(const MasterControlTargets &targets) const {
  manualControlOutput.rollCommand = targets.roll / 2.0f;
  manualControlOutput.yawCommand = targets.yawRate / 2.0f;
  manualControlOutput.throttleCommand = targets.throttle;
  manualControlOutput.pitchCommand = targets.pitch / 2.0f;
}
