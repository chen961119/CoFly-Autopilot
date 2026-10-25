#include "parameters/control_parameters.h"
#include "control/controller_instances.h"
#include "control/control_output.h"
#include "allocation/slave_mixer.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"

void mixSlaveCommands() {

  commands.local = {};
  const float elevator = currentFlightMode == MANUAL_MODE ? slaveCommand.masterContribution.elevator :
      (activePitchController == PitchController::INDI ?
       -pitchIndiControlOutput.indiOutput :
       -pitchPidControlOutput.pidOutput);
  const float compensation = ControlParameters::elevator_aileron_gain * elevator;
  commands.local.aileron1 += compensation + rollDampingControlOutput.dampingCommand;
  commands.local.aileron2 += compensation - rollDampingControlOutput.dampingCommand;
  commands.local.elevator = currentFlightMode == MANUAL_MODE ? 0.0f : elevator;
}
