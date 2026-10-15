#include "control/master_controller.h"
#include "control/control_output.h"
#include "control/command_generation.h"
#include "control/controller_instances.h"
#include "control/control_modes.h"
#include "config/aircraft_config.h"

void runMasterController()
{
  auto &controllers = controlControllers();
  controllers.rollDamping.update();
  controllers.roll.prepare(integralEnabled, changed);
  controllers.yaw.prepare(integralEnabled, changed);
  controllers.configuration.prepare(integralEnabled);
  controllers.pitchPid.prepare(integralEnabled, changed);
  if (changed) controllers.pitchIndi.reset(targetPitchDegrees());

  const bool stabilized = isStabilizedMode(currentMode);
  controllers.roll.update(targets.roll, stabilized);
  controllers.yaw.update(targets.yaw, targets.roll, 0.0f, stabilized);
  controllers.throttle.update(targets.throttle);
  controllers.pitchPid.update(targets.pitch, stabilized && pitchController == PitchController::PID);
  controllers.pitchIndi.update(targets.pitch, stabilized && pitchController == PitchController::INDI);
  controllers.manual.update(targets);
  controllers.configuration.update(targets.configuration);
}
