#include "parameters/control_parameters.h"
#include "control/controller_instances.h"
#include "control/control_output.h"
#include "allocation/slave_mixer.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"

float allocatePitchCommand() {
  if (currentMode == MANUAL_MODE) return manualControlOutput.pitch;
  if (pitchController == PitchController::INDI) {
    // 原 INDI 输出按实测物理 PWM 标定；只在此适配回统一逻辑方向，输出时恢复一次。
    const int pwm = pitchIndiControlOutput.pwm;
    return (pwm - PWM_CENTER_US - pwm_channel3_trim) / (1000.0f * pwm_channel3_rev);
  }
  // PID 直接输出逻辑控制量；PWM 缩放由执行器层完成。
  return pitchPidControlOutput.command;
}

void mixSlaveCommands() {
  controlControllers().rollDamping.update();
  commands.local = {};
  const float elevator = currentMode == MANUAL_MODE ? slaveCommand.masterContribution.elevator :
      allocatePitchCommand();
  // 交联只在从分配生成；主机升降舵前馈不重复作为本机俯仰贡献。
  commands.local.aileron1 = commands.local.aileron2 = elevator / 15.0f + rollDampingControlOutput.command / 1000.0f;
  if (isStabilizedMode(currentMode)) commands.local.elevator = elevator;
}
