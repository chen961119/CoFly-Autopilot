#pragma once
#include "control/control_types.h"

void initializeInitialControlMode();
extern FlightMode currentFlightMode;
extern FlightMode previousFlightMode;
extern PitchController activePitchController;
extern PitchController previousPitchController;
extern SlaveCommand slaveCommand;
extern bool controlModeChanged;
// 主机遥控直通输入：[roll, pitch, throttle, yaw]；[0/1/3] 为 -1..1，[2] 为 0..1。
extern float manual_input[4];
// 主机读取遥控模式；从机接收上级命令；共同准备本机俯仰算法和模式状态。
extern float mixerStrategyCommand;
extern float configurationAngle; // 构型角（rad），固定为平直 0，不再绑定 CH7。
extern bool brakeEnabled;
void prepareControlMode();
bool isStabilizedMode(FlightMode mode);
