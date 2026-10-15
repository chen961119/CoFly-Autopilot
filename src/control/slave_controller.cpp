#include "control/slave_controller.h"
#include "control/control_output.h"
#include "control/command_generation.h"
#include "interface/human_interface.h"
#include "control/controller_instances.h"
#include "control/control_modes.h"

void runSlaveController() {
  auto &controllers = controlControllers();
  controllers.pitchPid.prepare(integralEnabled, changed);
  if (changed) controllers.pitchIndi.reset(targets.pitch);

  const bool stabilized = isStabilizedMode(currentMode);
  controllers.roll.update(0.0f, false);
  controllers.yaw.update(0.0f, 0.0f, 0.0f, false);
  controllers.throttle.update(0.0f);
  controllers.pitchPid.update(targets.pitch, stabilized && pitchController == PitchController::PID);
  controllers.pitchIndi.update(targets.pitch, stabilized && pitchController == PitchController::INDI);
  controllers.configuration.update(targets.configuration);
}
