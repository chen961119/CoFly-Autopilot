#include "parameters/control_parameters.h"
//#include "config/flight_config.h"

namespace ControlParameters {
int32_t integralControlEnabled = 1; // SD 参数；控制滚转/俯仰/构型/空速积分，偏航积分独立。
int32_t pitch_controller = 0; // 保留原默认 PID 选择，用于增稳模式。
float Trim_pitch_angle = 3.0;
float k_Clp = 0.05;
// 滚转/俯仰/构型沿用原公共 i_limit 的误差积分限幅；偏航改为限制 I 项输出。
float i_limit_roll_angle = 45.0f;
float i_limit_roll_rate = 45.0f;
float i_limit_pitch_angle = 45.0f;
float i_limit_pitch_rate = 45.0f;
float i_limit_yaw_rate = 0.1f; // 乘 Ki 后的积分控制量上限。
float i_limit_FLAP_RATE = 45.0f; // 六路构型角速度环使用同一组构型参数。
float maxRoll = 80.0;        // 最大滚转期望；角度模式为 °，角速度模式为 °/s。
float maxPitch = 60.0;       // 最大俯仰期望；单位随控制模式变化。
float maxYawRate = 40.0;        // 最大偏航角速度期望，单位 °/s。
float V_cruise = 13.0f;      // 巡航空速（m/s）。
int32_t airspeed_control_enabled = 0;
float airspeed_min = 8.0f;
float airspeed_max = 22.0f;
float throttle_cruise = 0.5f;
float Kp_airspeed = 0.04f;
float Ki_airspeed = 0.01f;
float airspeed_lpf_hz = 2.0f;
float Kp_roll_angle = 0.25;  // 滚转角比例增益。
float Ki_roll_angle = 0.0;   // 滚转角积分增益。
float Kd_roll_angle = 0.0;   // 滚转角微分增益；串级滚转 PID 不使用该参数。
float B_loop_roll = 1.0;     // 滚转外环阻尼系数，范围 0～1。
float Kp_pitch_angle = 0.12; // 俯仰角比例增益。
float Ki_pitch_angle = 0.0;  // 俯仰角积分增益。
float Kd_pitch_angle = 0.00; // 俯仰角微分增益；串级俯仰 PID 不使用该参数。
float B_loop_pitch = 1.0;    // 俯仰外环阻尼系数，范围 0～1。
float Kp_Flap = 0.2;         // 襟翼角度比例增益。
float B_loop_FLAP = 0.0;
// INDI 的 q、dq 抬头为正；升降舵偏角 delta_e 下偏为正，单位 deg。
float indi_pitch_q_gain = 25.0f;           // 单位 1/s；dq_des = Kq * (q_des - q_filtered)。
float indi_pitch_effectiveness = -113.65f; // 局部舵效 G，单位 (deg/s^2)/deg；负值表示下偏使俯仰角加速度减小。
// 零舵角 PWM：模式 0 为 trim，模式 1 为端点中点；delta_e=(PWM-零点)*rev*k。
float indi_pitch_pwm_to_deg_k = 0.090909f; // 正斜率大小，deg/us；安装方向由本机rev承担，不包含在k中。
float indi_pitch_servo_delay_s = 0.03f; // 估计模型的纯延迟，s；按当前 dt 换算并取最近整数周期，最多 63 周期。
float indi_pitch_servo_tau_s = 0.015f;  // 估计模型的一阶时间常数，s；延迟后的命令还需经过此响应过程。
float indi_pitch_rate_limit_deg_s = 800.0f; // 舵偏命令限速，deg/s；每周期允许变化量为该值乘 dt。
float indi_pitch_cmd_lpf_fc_hz = 6.0f;      // 舵偏命令的一阶低通截止频率，Hz；非正值关闭此低通。
float Kp_roll_rate = 0.15;   // 滚转角速度比例增益。
float Ki_roll_rate = 0.1;    // 滚转角速度积分增益。
float Kd_roll_rate = 0.0002; // 滚转角速度微分增益；增大前应检查输出噪声。
float Kp_pitch_rate = 0.11;  // 俯仰角速度比例增益。
float Ki_pitch_rate = 0.10;  // 俯仰角速度积分增益。
float Kd_pitch_rate = 0.000; // 俯仰角速度微分增益；当前设为零。
float Kff_roll_rate = 0.12;  // 滚转角速度前馈增益。
float Kff_pitch_rate = 0.20;
float Kff_yaw_rate = 0.03;
float Kff_FLAP_RATE = 0.1;
float Kp_FLAP_RATE = 0.2;     // 0.09
float Ki_FLAP_RATE = 0.20;    // 0.15
float Kp_yaw_rate = 0.1;           // 偏航角速度比例增益。
float Ki_yaw_rate = 0.03;          // 偏航角速度积分增益。
float Kd_yaw_rate = 0.00000;       // 偏航角速度微分增益；当前设为零。
float roll_pid_lpf_fc = 7.0f; // 7hz指令滤波
float pitch_des_local_rate_lpf_fc = 5.0f; // INDI 目标角差分前馈的一阶低通截止频率，Hz；非正值直通，不是 q 反馈截止频率。
float roll_pid_dot_lpf_fc = 10.0f; // 先给个截止频率参数，后面可调
} // namespace ControlParameters

// PWM——OUTPUT
