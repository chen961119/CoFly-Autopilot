#include "parameters/control_parameters.h"
#include "control/controller_instances.h"
#include "control/control_output.h"
#include "allocation/slave_mixer.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"

void mixSlaveCommands() {

  commands.local = {};
  // 上级贡献已经由主机换算，只供手动交联/后续融合直接使用。
  // 本机 PID 贡献单独乘 5；INDI 已为执行器尺度，不再乘倍数。
  const float elevator = currentFlightMode == MANUAL_MODE ? slaveCommand.masterContribution.elevator :
      (activePitchController == PitchController::INDI ?
       -pitchIndiControlOutput.indiOutput :
       -pitchPidControlOutput.pidOutput * 5.0f);
  // 用已换算的本机舵面贡献生成交联，不对交联/阻尼再乘 2，也不计入上级前馈。
  const float compensation = ControlParameters::elevator_aileron_gain * elevator;
  commands.local.aileron1 += compensation + rollDampingControlOutput.dampingCommand*5;
  commands.local.aileron2 += compensation - rollDampingControlOutput.dampingCommand*5;
  commands.local.elevator = currentFlightMode == MANUAL_MODE ? 0.0f : elevator;
}
