#include "math/units.h"
#include "control/slave_controller.h"
#include "control/control_output.h"
#include "control/command_generation.h"
#include "interface/human_interface.h"
#include "control/controller_instances.h"
#include "control/control_modes.h"
#include "parameters/control_parameters.h"

void runSlaveController() {
  auto &controllers = controlControllers();
  controllers.pitchPid.prepare(ControlParameters::integralControlEnabled != 0, controlModeChanged);
  if (controlModeChanged) controllers.pitchIndi.reset(Units::toDegrees(targets.pitch));

  const bool stabilized = isStabilizedMode(currentFlightMode);
  controllers.roll.update(0.0f, false);
  controllers.yawRate.update(0.0f, 0.0f, 0.0f, false);
  controllers.throttle.update(0.0f, 0.0f, false, false, false, 0.0f);
  controllers.pitchPid.update(Units::toDegrees(targets.pitch), stabilized && activePitchController == PitchController::PID);
  controllers.pitchIndi.update(Units::toDegrees(targets.pitch), stabilized && activePitchController == PitchController::INDI);
  controllers.configuration.update(targets.configuration);
}
