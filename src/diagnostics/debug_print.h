#pragma once
#include <stdint.h>

int32_t &localThrottleDebugSetting();
int32_t &actuatorOutputDebugSetting();
int32_t &controlModeDebugSetting();

// enabled=1 时以 10 Hz 打印最终物理 PWM，其他值不打印。
void printActuatorOutput(int enabled);
// enabled=1 时以 10 Hz 打印 initializeInitialControlMode 涉及变量的当前值。
void printControlMode(int enabled);

// 串口诊断输出；打印节流计时由 debug_print.cpp 独立维护。
void printRadioData();
void printDesiredState();
void printConfigurationData();
void printPIDoutput();
void printServoCommands();
void printLoopRate();
void printLocalThrottle();
void printGyroData();
void printAccelData();
void printRollPitchYaw();
void printQuaternion();
