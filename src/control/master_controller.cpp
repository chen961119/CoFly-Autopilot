#include "control/master_controller.h"
#include "control/control_output.h"
#include "control/command_generation.h"
#include "control/controller_instances.h"
#include "control/control_modes.h"
#include "config/aircraft_config.h"
#include "parameters/control_parameters.h"

void runMasterController()
{
  auto &controllers = controlControllers();
  const bool integralEnabled = ControlParameters::integralControlEnabled != 0;
  controllers.rollDamping.update();
  controllers.roll.prepare(integralEnabled, controlModeChanged);
  controllers.yawRate.prepare(integralEnabled, controlModeChanged);
  controllers.configuration.prepare(integralEnabled);
  controllers.pitchPid.prepare(integralEnabled, controlModeChanged);
  if (controlModeChanged) controllers.pitchIndi.reset(targetPitchDegrees());

  const bool stabilized = isStabilizedMode(currentFlightMode);
  controllers.roll.update(targets.roll, stabilized);
  controllers.yawRate.update(targets.yawRate, targets.roll, 0.0f, stabilized);
  controllers.throttle.update(targets.throttle);
  controllers.pitchPid.update(targets.pitch, stabilized && activePitchController == PitchController::PID);
  controllers.pitchIndi.update(targets.pitch, stabilized && activePitchController == PitchController::INDI);
  controllers.manual.update(targets);
  controllers.configuration.update(targets.configuration);
}
