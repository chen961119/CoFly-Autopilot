#pragma once
#include <stdint.h>

int32_t &localThrottleDebugSetting();
int32_t &actuatorOutputDebugSetting();
int32_t &controlModeDebugSetting();
int32_t &airspeedDebugSetting();

// enabled=1 时以 10 Hz 打印最终物理 PWM，其他值不打印。
void printActuatorOutput(int enabled);
// enabled=1 时以 10 Hz 打印 initializeInitialControlMode 涉及变量的当前值。
void printControlMode(int enabled);
// enabled=1 时以独立的 10 Hz 节流打印本机空速（m/s）。
void printAirspeed(int enabled);

// USB/X6 双链路非阻塞诊断输出；打印节流计时由 debug_print.cpp 独立维护。
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
