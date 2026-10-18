#pragma once
#include "control/control_types.h"

extern ControlTargets targets;
extern SlaveControlTargets slaveTargets[CONTROL_AIRCRAFT_SLOTS];
extern SlaveCommand slaveCommands[CONTROL_AIRCRAFT_SLOTS];
// 主机手动归一化目标换算为角度；全局从机 pitch 为 rad；这些兼容函数返回度制供既有控制器使用。
float targetRollDegrees();
float targetPitchDegrees();
float targetYawRateDegreesPerSec();
// 在主机分配之后生成从机目标、作动器贡献及独立发送命令。
void generateSlaveCommand();
