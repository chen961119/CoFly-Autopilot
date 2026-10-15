#pragma once
#include "control/control_types.h"

extern ControlTargets targets;
extern SlaveControlTargets slaveTargets[CONTROL_AIRCRAFT_SLOTS];
extern SlaveCommand slaveCommands[CONTROL_AIRCRAFT_SLOTS];
// 主机手动归一化目标换算为角度；从机 pitch 已是角度。
float targetRollDegrees();
float targetPitchDegrees();
float targetYawDegrees();
// 在主机分配之后生成从机目标、作动器贡献及独立发送命令。
void generateSlaveCommand();
