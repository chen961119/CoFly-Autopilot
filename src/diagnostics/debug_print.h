#pragma once
#include <stdint.h>

int32_t &actuatorOutputDebugSetting();
int32_t &controlModeDebugSetting();
int32_t &airspeedDebugSetting();
int32_t &radioDebugSetting();
int32_t &desiredStateDebugSetting();
int32_t &configurationDebugSetting();
int32_t &pidOutputDebugSetting();
int32_t &servoCommandsDebugSetting();
int32_t &loopRateDebugSetting();
int32_t &gyroDebugSetting();
int32_t &accelDebugSetting();
int32_t &attitudeDebugSetting();
int32_t &quaternionDebugSetting();

// enabled=1 时以 10 Hz 打印最终物理 PWM，其他值不打印。
void printActuatorOutput(int enabled = 0);
// enabled=1 时以 10 Hz 打印 initializeInitialControlMode 涉及变量的当前值。
void printControlMode(int enabled = 0);
// enabled=1 时以独立的 10 Hz 节流打印本机空速（m/s）。
void printAirspeed(int enabled = 0);

// USB/X6 双链路非阻塞诊断输出；默认关闭，仅 enabled=1 时按各自 10 Hz 节流打印。
void printRadioData(int enabled = 0);
void printDesiredState(int enabled = 0);
void printConfigurationData(int enabled = 0);
void printPIDoutput(int enabled = 0);
void printServoCommands(int enabled = 0);
void printLoopRate(int enabled = 0);
void printGyroData(int enabled = 0);
void printAccelData(int enabled = 0);
void printRollPitchYaw(int enabled = 0);
void printQuaternion(int enabled = 0);

// 每周期在最终执行器输出之后调用，按各 Debug 参数选择打印项。
void runDebugPrints();
