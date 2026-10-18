#pragma once

#include <stdint.h>

// 控制器可调参数；运行状态和积分/滤波历史不放在此处。
// 默认值在 control_parameters.cpp，注册元数据在 parameter_registry.cpp。
namespace ControlParameters {
extern int32_t integralControlEnabled; // 0 关闭 PID 积分，1 开启；各机独立设置。
extern int32_t pitch_controller; // 0 PID，1 INDI；各机独立设置。
extern float Trim_pitch_angle;
extern float k_Clp;
extern float i_limit;
extern float maxRoll;
extern float maxPitch;
extern float maxYawRate;
extern float V_cruise; // 偏航协调转弯计算使用的巡航空速，m/s。
extern int32_t airspeed_control_enabled; // 0 油门直通，1 增稳时空速 PI。
extern float airspeed_min, airspeed_max; // 油门杆映射的目标空速范围，m/s。
extern float throttle_cruise; // 空速 PI 的基础油门，0～1。
extern float Kp_airspeed, Ki_airspeed; // 油门/(m/s)、油门/(m/s*s)。
extern float airspeed_lpf_hz; // 控制器反馈一阶低通截止频率；0 直通。
extern float Kp_roll_angle;
extern float Ki_roll_angle;
extern float Kd_roll_angle;
extern float B_loop_roll;
extern float Kp_pitch_angle;
extern float Ki_pitch_angle;
extern float Kd_pitch_angle;
extern float B_loop_pitch;
extern float Kp_Flap;
extern float B_loop_FLAP;
extern float indi_pitch_q_gain;
extern float indi_pitch_effectiveness;
extern float indi_pitch_pwm_to_deg_k;
extern float indi_pitch_servo_delay_s;
extern float indi_pitch_servo_tau_s;
extern float indi_pitch_rate_limit_deg_s;
extern float indi_pitch_cmd_lpf_fc_hz;
extern float Kp_roll_rate;
extern float Ki_roll_rate;
extern float Kd_roll_rate;
extern float Kp_pitch_rate;
extern float Ki_pitch_rate;
extern float Kd_pitch_rate;
extern float Kff_roll_rate;
extern float Kff_pitch_rate;
extern float Kff_yaw_rate;
extern float Kff_FLAP_RATE;
extern float Kp_FLAP_RATE;
extern float Ki_FLAP_RATE;
extern float Kp_yaw_rate;
extern float Ki_yaw_rate;
extern float Kd_yaw_rate;
extern float roll_pid_lpf_fc;
extern float pitch_des_local_rate_lpf_fc;
extern float roll_pid_dot_lpf_fc;
} // namespace ControlParameters

// 执行器软件脉宽上下限（us），本机 SD 参数；方向和 trim 不改变其物理含义。
extern int32_t pwm_servo1_min_us, pwm_servo1_max_us;
extern int32_t pwm_servo2_min_us, pwm_servo2_max_us;
extern int32_t pwm_servo3_min_us, pwm_servo3_max_us;
extern int32_t pwm_servo4_min_us, pwm_servo4_max_us;
extern int32_t pwm_servo5_min_us, pwm_servo5_max_us;

// 执行器方向和本机安装偏置，供控制、输出与日志模块共享。
extern int32_t pwm_channel1_rev, pwm_channel2_rev, pwm_channel3_rev, pwm_channel4_rev,
    pwm_channel5_rev;
extern float pwm_channel1_trim, pwm_channel2_trim, pwm_channel3_trim, pwm_channel4_trim,
    pwm_channel5_trim;
