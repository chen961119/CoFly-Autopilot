#pragma once
#include "control/control_types.h"

// 最终写给舵机的物理 PWM，单位 μs：副翼1/副翼2/升降舵/油门/方向舵。
extern float ActuatorOutput[5];

void attachActuators();
void commandSafeActuatorPositions();
// 本机最终逻辑指令转物理 PWM。
void convertControlCommandsToPWM(const ControlCycleOutput &cycle);
// 仅打包已有子机指令，不运行控制器、mixer 或交联。
void prepareActuatorCommands();
void applyAndTransmitActuatorCommands();
