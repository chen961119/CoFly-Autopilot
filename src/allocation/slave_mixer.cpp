#include "parameters/control_parameters.h"
#include "control/controller_instances.h"
#include "control/control_output.h"
#include "allocation/slave_mixer.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"

void mixSlaveCommands() {

  commands.local = {};
  // 上级贡献已为下偏正；本机PID/INDI为抬头正，选用时取负转换为舵面量。
  const float elevator = currentFlightMode == MANUAL_MODE ? slaveCommand.masterContribution.elevator :
      (activePitchController == PitchController::INDI ?
       -pitchIndiControlOutput.indiOutput :
       -pitchPidControlOutput.pidOutput);
  // 交联只在从分配生成；主机升降舵前馈不重复作为本机俯仰贡献。
  const float compensation = ControlParameters::elevator_aileron_gain * elevator;
  commands.local.aileron1 = commands.local.aileron2 = compensation + rollDampingControlOutput.dampingCommand;
  commands.local.elevator = currentFlightMode == MANUAL_MODE ? 0.0f : elevator;
}
