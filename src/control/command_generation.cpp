#include "math/units.h"
#include "control/control_output.h"
#include "control/command_generation.h"
#include "control/control_pipeline.h"
#include "control/control_modes.h"
#include "config/aircraft_config.h"
#include <cmath>
#include "parameters/control_parameters.h"
using namespace ControlParameters;

ControlTargets targets{};
SlaveControlTargets slaveTargets[CONTROL_AIRCRAFT_SLOTS]{};
SlaveCommand slaveCommands[CONTROL_AIRCRAFT_SLOTS]{};

// 分配与日志仍使用角度参考；不再保存重复的全局目标。
float targetRollDegrees() {
  return aircraftId() == 1 && currentFlightMode == MANUAL_MODE ? targets.roll * maxRoll : Units::toDegrees(targets.roll);
}

float targetPitchDegrees() {
  return aircraftId() == 1 && currentFlightMode == MANUAL_MODE
      ? targets.pitch * maxPitch + Trim_pitch_angle : Units::toDegrees(targets.pitch);
}

// yawRate 目标的统一角速度尺度，单位 deg/s；手动归一化输入按最大偏航角速度换算。
float targetYawRateDegreesPerSec() {
  return aircraftId() == 1 && currentFlightMode == MANUAL_MODE ? targets.yawRate * maxYawRate : Units::toDegrees(targets.yawRate);
}


// 根据本周期运动目标和原始分配结果生成从机目标、作动器贡献和待发送命令。
void generateSlaveCommand() {
  for (unsigned i = 0; i < CONTROL_AIRCRAFT_SLOTS; ++i) {
    slaveTargets[i].pitch = i < static_cast<unsigned>(aircraftCount() - 1) ? Units::toRadians(targetPitchDegrees()) : 0;
    slaveCommands[i] = {};
  }

  const float roll = targetRollDegrees();
  const float rollCommand = currentFlightMode == MANUAL_MODE ? targets.roll : rollControlOutput.pidOutput;
  const bool strategy1 = mixerStrategyCommand > 0.2f;
  const bool strategy2 = mixerStrategyCommand < 0.2f && mixerStrategyCommand > -0.2f;
  if (aircraftCount() == 3) {
    if (strategy1) {
      // slaveTargets[0].pitch = targets.pitch;
      // slaveTargets[1].pitch =  targets.pitch;
    } else if (strategy2) {
      slaveTargets[0].pitch += Units::toRadians(20.0 * rollControlOutput.filteredPidOutput);
      slaveTargets[1].pitch += Units::toRadians(-20.0 * rollControlOutput.filteredPidOutput);
    } else {
      const float coeab = fabsf(Units::toDegrees(configurationControlOutput.angleErrors[0])) > 20
          ? fabsf(Units::toDegrees(configurationControlOutput.angleErrors[0])) / 20.0f : 0.8f;
      const float coeac = 1.0f;
      const float coeroll = 0.2f;
      const float ab = configurationControlOutput.pidOutputs[0], ac = configurationControlOutput.pidOutputs[1];
      // 与原 mixer 的两路原始副翼公式一致，不从缩放/刹车后的输出反推。
      const float b1 = 1.016 * coeroll * rollCommand + coeab * 0.864 * ab - coeac * 0.62 * ac;
      const float b2 = 0.593 * coeroll * rollCommand - coeab * 0.27 * ab - coeac * 0.08 * ac;
      const float c1 = -0.593 * coeroll * rollCommand + coeab * 0.08 * ab + coeac * 0.27 * ac;
      const float c2 = -1.016 * coeroll * rollCommand + coeab * 0.62 * ab - coeac * 0.864 * ac;
      slaveTargets[0].pitch += Units::toRadians(30.0 * rollControlOutput.filteredPidOutput + 6.0 * (b1 + b2));
      slaveTargets[1].pitch += Units::toRadians(-30.0 * rollControlOutput.filteredPidOutput + 6.0 * (c1 + c2));
    }
  } else if (aircraftCount() == 4) {
    if (strategy1) {
      slaveTargets[2].pitch += Units::toRadians(0.13 * 1.5 * roll);
      slaveTargets[0].pitch += Units::toRadians(0.07 * 1.5 * roll);
      slaveTargets[1].pitch += Units::toRadians(-0.13 * 1.5 * roll);
    } else if (strategy2) {
      slaveTargets[2].pitch += Units::toRadians(70.0 * rollCommand);
      slaveTargets[0].pitch += Units::toRadians(30.0 * rollCommand);
      slaveTargets[1].pitch += Units::toRadians(-70.0 * rollCommand);
    } else {
      const float ab = configurationControlOutput.pidOutputs[0], ac = configurationControlOutput.pidOutputs[1], bd = configurationControlOutput.pidOutputs[2];
      slaveTargets[2].pitch += Units::toRadians(70.0 * rollCommand + 35.0 * ab - 20.0 * ac + 20.0 * bd);
      slaveTargets[0].pitch += Units::toRadians(30.0 * rollCommand - 40.0 * ab + 15.0 * ac - 45.0 * bd);
      slaveTargets[1].pitch += Units::toRadians(-70.0 * rollCommand + 35.0 * ab - 20.0 * ac + 20.0 * bd);
    }
  } else if (aircraftCount() == 5) {
    slaveTargets[2].pitch += Units::toRadians(0.3 * 1.2 * roll);
    slaveTargets[0].pitch += Units::toRadians(0.15 * 1.2 * roll);
    slaveTargets[1].pitch += Units::toRadians(-0.15 * 1.2 * roll);
    slaveTargets[3].pitch += Units::toRadians(-0.3 * 1.2 * roll);
  } else if (aircraftCount() == 7) {
    slaveTargets[4].pitch += Units::toRadians(0.5 * 1.8 * roll);
    slaveTargets[2].pitch += Units::toRadians(0.3 * 1.8 * roll);
    slaveTargets[0].pitch += Units::toRadians(0.15 * 1.8 * roll);
    slaveTargets[1].pitch += Units::toRadians(-0.15 * 1.8 * roll);
    slaveTargets[3].pitch += Units::toRadians(-0.3 * 1.8 * roll);
    slaveTargets[5].pitch += Units::toRadians(-0.5 * 1.8 * roll);
  }

  const float brakeOffset = ::brakeEnabled ? 0.1f : 0.0f;
  for (unsigned i = 0; i < CONTROL_AIRCRAFT_SLOTS; ++i) {
    ActuatorCommand &child = commands.slaves[i];
    // 手动交联先读取原始副翼值，再原位缩放，避免把刹车和缩放计入交联。
    if (currentFlightMode == MANUAL_MODE) {
      child.elevator = commands.local.elevator -
          0.5f * (child.aileron1 + child.aileron2) / 2;
    }
    child.aileron1 = child.aileron1 + brakeOffset;
    child.aileron2 = child.aileron2 + brakeOffset;
  }
  for (unsigned i = 0; i < CONTROL_AIRCRAFT_SLOTS; ++i) {
    slaveCommands[i].mode = currentFlightMode;
    slaveCommands[i].target = slaveTargets[i];
    slaveCommands[i].masterContribution = commands.slaves[i];
  }
}
