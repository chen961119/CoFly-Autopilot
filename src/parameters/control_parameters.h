#pragma once

#include <stdint.h>

// 控制器可调参数；运行状态和积分/滤波历史不放在此处。
// 默认值在 control_parameters.cpp，注册元数据在 parameter_registry.cpp。
namespace ControlParameters {
extern float Trim_pitch_angle;
extern float k_Clp;
extern float i_limit;
extern float maxRoll;
extern float maxPitch;
extern float maxYaw;
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
extern float indi_pitch_pwm_to_deg_b;
extern float indi_pitch_servo_delay_s;
extern float indi_pitch_servo_tau_s;
extern float indi_pitch_defl_min;
extern float indi_pitch_defl_max;
extern float indi_pitch_rate_limit_deg_s;
extern float indi_pitch_cmd_lpf_fc_hz;
extern float indi_pitch_pwm_min;
extern float indi_pitch_pwm_max;
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
extern float Kp_yaw;
extern float Ki_yaw;
extern float Kd_yaw;
extern float roll_pid_lpf_fc;
extern float pitch_des_local_rate_lpf_fc;
extern float roll_pid_dot_lpf_fc;
} // namespace ControlParameters

// 执行器方向和本机安装偏置，供控制、输出与日志模块共享。
extern int32_t pwm_channel1_rev, pwm_channel2_rev, pwm_channel3_rev, pwm_channel4_rev,
    pwm_channel5_rev;
extern float pwm_channel1_trim, pwm_channel2_trim, pwm_channel3_trim, pwm_channel4_trim,
    pwm_channel5_trim;
