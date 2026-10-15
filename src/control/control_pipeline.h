#pragma once
#include "control/control_types.h"

// 本周期主从逻辑控制结果，由 runControlCycle() 统一更新。
extern ControlCycleOutput commands;
// 主从两条并列流程的模式编排。
void runControlCycle();
