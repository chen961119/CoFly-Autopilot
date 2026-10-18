#pragma once

// PWM 逻辑中位统一为 1500 μs；机械中位由各路 trim 修正。
constexpr int PWM_CENTER_US = 1500;

// 各通道软件上下限为 SD 参数 pwm_servo1_min_us/max_us 至 pwm_servo5_min_us/max_us，
// 声明与默认值见 parameters/control_parameters.h/.cpp。

// 驱动范围独立于软件限幅，覆盖全部合法锁定位置（1500 + trim）。
constexpr int PWM_SERVO_ATTACH_MIN_US = 1000;
constexpr int PWM_SERVO_ATTACH_MAX_US = 2000;
constexpr bool validActuatorPwmLimits(int minimum, int maximum) {
  return PWM_SERVO_ATTACH_MIN_US <= minimum && minimum <= maximum &&
         maximum <= PWM_SERVO_ATTACH_MAX_US;
}

// Identity is an EEPROM boot parameter; count is COFLY_AIRCRAFT_COUNT in aircraft_config.h.
// 俯仰算法由 SD 参数 pitch_controller 选择：0 PID，1 INDI。

// MT6701 is measurement/logging only. Wing control uses adjacent IMU rolls.
// #define expensive

// #define EXTIMU
#define INTIMU

// Storage profile label only; COFLY_AIRCRAFT_COUNT=1 selects single-aircraft operation.
#define TEAM


#define USE_SBUS_RX

// IMU 选择宏同时决定机间状态发送使用的数据源。
#define USE_MPU6050_I2C

enum FlightMode {
  MANUAL_MODE = 0,
  STABILIZE_MODE = 1,
};

// 陀螺仪量程只能启用一项，单位 deg/s。
#define GYRO_250DPS // 当前默认量程
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// 加速度计量程只能启用一项，单位 g。
#define ACCEL_2G // 当前默认量程
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

// #define USE_BAROMETER
// 启用气压计前需确认板上焊点已短接，并评估读取耗时对循环周期的影响。
