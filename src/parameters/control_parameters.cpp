#include "parameters/control_parameters.h"
//#include "config/flight_config.h"

namespace ControlParameters {
int32_t integralControlEnabled = 1; // SD 参数，下一周期生效；关闭时清除 PID 积分历史。
int32_t pitch_controller = 1; // 保留原默认 INDI 选择，用于增稳模式。
float Trim_pitch_angle = 3.0;
float k_Clp = 0.05;
float i_limit = 45.0;        // 积分项限幅，防止长时间误差导致指令过大。
float maxRoll = 80.0;        // 最大滚转期望；角度模式为 °，角速度模式为 °/s。
float maxPitch = 60.0;       // 最大俯仰期望；单位随控制模式变化。
float maxYaw = 160.0;        // 最大偏航角速度期望，单位 °/s。
float V_cruise = 13.0f;      // 巡航空速（m/s）。
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
float indi_pitch_q_gain = 25.0f;           // q误差转dq_des的比例
float indi_pitch_effectiveness = -113.65f; // 升降舵舵效: ang_acc[1] / delta_e, 单位 deg/s^2 per deg
float indi_pitch_pwm_to_deg_k = 0.090909f; // delta_e = k * pwm + b, 1500PWM中位, 1720PWM下偏20deg
float indi_pitch_pwm_to_deg_b = -136.3636f;
float indi_pitch_servo_delay_s = 0.03f; // 舵机纯延迟
float indi_pitch_servo_tau_s = 0.015f;  // 舵机一阶时间常数
float indi_pitch_defl_min = -20.0f;     // 最大最小可用偏角 20deg
float indi_pitch_defl_max = 20.0f;
float indi_pitch_rate_limit_deg_s = 800.0f; // 舵偏角速度限幅
float indi_pitch_cmd_lpf_fc_hz = 6.0f;      // INDI输出舵偏指令低通截止频率 13.0
float indi_pitch_pwm_min = 1100.0f;
float indi_pitch_pwm_max = 1920.0f;
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
float Kp_yaw = 0.2;           // 偏航角速度比例增益。
float Ki_yaw = 0.05;          // 偏航角速度积分增益。
float Kd_yaw = 0.00000;       // 偏航角速度微分增益；当前设为零。
float roll_pid_lpf_fc = 7.0f; // 7hz指令滤波
float pitch_des_local_rate_lpf_fc = 5.0f;
float roll_pid_dot_lpf_fc = 10.0f; // 先给个截止频率参数，后面可调
} // namespace ControlParameters

// PWM——OUTPUT

int32_t pwm_channel1_rev = -1;
int32_t pwm_channel2_rev = 1;
int32_t pwm_channel3_rev = -1;
int32_t pwm_channel4_rev = 1;
int32_t pwm_channel5_rev = -1;

// 本机安装偏置（μs），默认零；开机从本机 SD 卡恢复。
// trim 不随 rev 反向，各机在各自 USB 参数表中校准并保存。
float pwm_channel1_trim = 0;
float pwm_channel2_trim = 0;
float pwm_channel3_trim = 0;
float pwm_channel4_trim = 0;
float pwm_channel5_trim = 0;
