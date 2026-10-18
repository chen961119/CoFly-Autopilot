#include "control/manual_controller.h"
#include "control/control_output.h"
#include "control/control_modes.h"
#include "math/units.h"

void ManualController::update(const MasterControlTargets &targets) const {
  // The inactive manual result is still logged in stabilized mode. Preserve its
  // legacy numeric scale while shared physical targets are now SI.
  manualControlOutput.rollCommand = (isStabilizedMode(currentFlightMode) ? Units::toDegrees(targets.roll) : targets.roll) / 2.0f;
  manualControlOutput.yawCommand = (isStabilizedMode(currentFlightMode) ? Units::toDegrees(targets.yawRate) : targets.yawRate) / 2.0f;
  manualControlOutput.throttleCommand = targets.throttle;
  manualControlOutput.pitchCommand = (isStabilizedMode(currentFlightMode) ? Units::toDegrees(targets.pitch) : targets.pitch) / 2.0f;
}
