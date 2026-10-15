#pragma once
#include "control/control_types.h"

// 第 6 层：从机俯仰分配及交联，输出从作动器指令-2（逻辑增量）。
void mixSlaveCommands();
// A 机也复用相同的俯仰分配，保留 PID/INDI 公式与尺度。
float allocatePitchCommand();
