#pragma once
#include "control/control_types.h"

void initializeInitialControlMode();
extern FlightMode currentFlightMode;
extern FlightMode previousFlightMode;
extern PitchController activePitchController;
extern PitchController previousPitchController;
extern SlaveCommand slaveCommand;
extern bool controlModeChanged;
// 主机读取遥控模式；从机接收上级命令；共同准备本机俯仰算法和模式状态。
extern float mixerStrategyCommand;
extern float configurationControlScale; // 构型控制输出系数，固定为 1.0f。
extern float configurationAngle; // CH7 构型角（rad）：上反 +π/18，下反 -π/18，平直 0。
extern bool brakeEnabled;
void prepareControlMode();
bool isStabilizedMode(FlightMode mode);
