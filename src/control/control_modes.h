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
extern float coeffconfiguration; // 构型控制输出系数，主机模式准备阶段更新。
extern bool brake;
void prepareControlMode();
bool isStabilizedMode(FlightMode mode);
