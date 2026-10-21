#pragma once
#include "control/control_types.h"

extern ControlTargets targets;
extern SlaveControlTargets slaveTargets[CONTROL_AIRCRAFT_SLOTS];
extern SlaveCommand slaveCommands[CONTROL_AIRCRAFT_SLOTS];
// 在主机分配之后生成从机目标、作动器贡献及独立发送命令。
void generateSlaveCommand();
