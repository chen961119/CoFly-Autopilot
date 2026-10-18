#include "parameters/control_parameters.h"
#include "control/controller_instances.h"
#include "control/control_output.h"
#include "allocation/slave_mixer.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"

void mixSlaveCommands() {
  controlControllers().rollDamping.update();
  commands.local = {};
  const float elevator = currentFlightMode == MANUAL_MODE ? slaveCommand.masterContribution.elevator :
      (activePitchController == PitchController::INDI ?
       pitchIndiControlOutput.indiOutput :
       -pitchPidControlOutput.pidOutput);
  // 交联只在从分配生成；主机升降舵前馈不重复作为本机俯仰贡献。
  commands.local.aileron1 = commands.local.aileron2 = -elevator / 15.0f + rollDampingControlOutput.dampingCommand;
  if (isStabilizedMode(currentFlightMode)) commands.local.elevator = elevator;
}
