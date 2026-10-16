#pragma once
#include "control/control_types.h"

void initializeInitialControlMode();
extern FlightMode currentMode;
extern FlightMode lastMode;
extern PitchController pitchController;
extern PitchController lastPitchController;
extern SlaveCommand slaveCommand;
extern bool changed;
extern bool integralEnabled;
// 主机读取遥控模式；从机接收上级命令；共同准备本机俯仰算法和模式状态。
extern float mixerStrategy;
extern float configurationCoeff; // 构型控制输出系数，固定为 1.0f。
extern float configurationAngle; // CH7 构型角（度）：上反 +10，下反 -10，平直 0。
extern bool brake;
void prepareControlMode();
bool isStabilizedMode(FlightMode mode);
