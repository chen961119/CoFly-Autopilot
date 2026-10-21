#include "math/units.h"
#include "control/pitch_indi_controller.h"
#include "math/math_constants.h"
#include "control/control_output.h"
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
#include "sensors/sensor_processing.h"
#include "config/flight_config.h"
#include "actuators/actuator_pwm_mapping.h"
using namespace ControlParameters;

// 单机俯仰 INDI（Incremental Nonlinear Dynamic Inversion，增量非线性动态逆）。
// 主机和从机都复用本控制器；它只求本机升降舵，不在这里做机间混控。
//
// 一次 update 的计算链：
//   theta_ref_deg -> theta_dot_cmd_deg_s -> q_ref_deg_s -> q_dot_ref_deg_s2
//   -> delta_delta_e_deg -> delta_e_cmd_deg -> indiOutput（抬头为正的归一化俯仰控制量）。
//   q_dot_ref 是虚拟控制量（目标俯仰角加速度）；delta_delta_e 是 Delta(delta_e)，即舵偏增量。
// 命名采用 INDI 文献常用符号，工程后缀规则和旧名对照见 docs/indi-variable-naming.md。
// ref=参考值，cmd=命令，f=滤波，hat=估计，dot=时间导数，prev=上一周期，meas=测量。
//
// 本文件的符号与单位：
//   theta_ref_deg 保持 deg；全局 att 为 rad，读入时转为 deg，抬头为正。
//   全局 ang_vel 为 rad/s、ang_acc 为 rad/s²；内部计算转回 deg/s、deg/s²。
//   这里 q 是俯仰角速度，不是四元数，也不是动压；r 是机体 Z 轴角速度。
//   delta_e_*：升降舵偏角，deg，下偏为正，上偏为负。
//   当前标定的 indi_pitch_effectiveness < 0：下偏产生负俯仰角加速度。
//   pwm_*_us：内部物理PWM，us，用于命令换算和延迟队列。
//   indiOutput：通道逆映射后取负；模式 0 为抬头正的有符号控制量，不是角加速度。
//   模式 1 取负后为 -1..0，不具备以配平为零的有符号语义；当前升降舵应使用模式 0。
//   参数对应：K_q = indi_pitch_q_gain，G_qe = indi_pitch_effectiveness。
//   K_q 单位 1/s，G_qe 单位 (deg/s^2)/deg；参数存储名和参数值保持原约定。
//
// 模式 0 的零舵角 PWM=trim；模式 1 的零舵角 PWM=(min+max)/2。
// delta_e = (PWM - 配平PWM) * rev * k，角度标定仍使用同一个物理斜率。
// 当前交联/前馈融合使用有符号贡献，通道3应保持模式0；模式1不自动转换整套混控坐标。

PitchIndiController::PitchIndiController() {
  // 本对象独立的二阶低通：q 反馈截止 40 Hz，Z 轴耦合项截止 100 Hz。
  // 两者按 500 Hz 采样频率设计系数；这不是把实际控制周期 dt 固定为 1/500。
  q_filter.set_cutoff_frequency(500, 40);
  r_filter.set_cutoff_frequency(500, 100);
}
void PitchIndiController::reset(float theta_ref_deg) {
  // 模式/俯仰算法切换时由外部调用。这里只标记重新初始化，
  // 下次有效 update 才重填延迟队列、重置 Y 轴滤波及舵机估计。
  // Z 轴耦合滤波器不在此重置，仍保留自己的历史，这是当前实现的行为。
  initialized = false;
  pwm_delay_write_index = 0;
  theta_ref_dot_filter_initialized = false;
  // 用切换时的目标作为差分起点，避免首周期把整个目标角当成目标变化率。
  // 这只消除切换造成的目标斜率跳变，角度误差本身仍会进入后面的控制律。
  theta_ref_prev_deg = theta_ref_deg;
  theta_ref_dot_f_deg_s = 0;
}
void PitchIndiController::update(float theta_ref_deg, bool enabled) {
  // 1. 每周期先清空发布结果。未启用时不推进滤波/队列，也不清除对象历史；
  //    正常切换流程会另外调用 reset()，不能把 enabled=false 当成一次 reset。
  pitchIndiControlOutput = {};
  if (!enabled) return;
  float q_f_deg_s = 0.0f;
  // 2. 只从通道3的物理范围、配平中位和rev推导角度范围，不另设INDI边界参数。
  //    物理PWM始终以配平中位为零舵角；rev=-1时两个角度端点的顺序会反转。
  const float pwm_trim_us = actuatorNeutralPwm(pwm_servo3_min_us, pwm_channel3_trim,
      pwm_servo3_max_us, pwm_channel3_mode);
  const float delta_e_at_pwm_min_deg =
      (pwm_servo3_min_us - pwm_trim_us) * pwm_channel3_rev * indi_pitch_pwm_to_deg_k;
  const float delta_e_at_pwm_max_deg =
      (pwm_servo3_max_us - pwm_trim_us) * pwm_channel3_rev * indi_pitch_pwm_to_deg_k;
  const float delta_e_min_deg = fminf(delta_e_at_pwm_min_deg, delta_e_at_pwm_max_deg);
  const float delta_e_max_deg = fmaxf(delta_e_at_pwm_min_deg, delta_e_at_pwm_max_deg);
  if (!initialized)
  {
    // q 滤波器从当前测量稳态开始，避免从零起滤产生人为的角速度误差。
    q_filter.reset(Units::toDegrees(ang_vel[1]));
    // 配平零舵角为0 deg；命令/估计从0开始，物理PWM历史填充配平中位。
    // 这里只假定舵机处于配平点，并没有读取舵面位置传感器。
    for (int i = 0; i < pwm_delay_buffer_length; i++)
    {
      pwm_delay_buffer_us[i] = pwm_trim_us;
    }
    delta_e_hat_deg = 0.0f;
    delta_e_cmd_prev_deg = 0.0f;
    delta_e_cmd_f_prev_deg = 0.0f;
    initialized = true;
  }
  q_f_deg_s = q_filter.apply(Units::toDegrees(ang_vel[1]));
  // 3. 俯仰角目标变化率前馈，单位 deg/s。
  //    对 theta_ref_deg 差分，不是对测量俯仰角差分；dt 为秒。
  //    dt 太小时本次差分置零，目标斜率随后限到 +/-10 deg/s。
  float theta_ref_dot_raw_deg_s;
  if (dt > 1.0e-6f)
  {
    theta_ref_dot_raw_deg_s = (theta_ref_deg - theta_ref_prev_deg) / dt;
  }
  else
  {
    theta_ref_dot_raw_deg_s = 0.0f;
  }
  theta_ref_prev_deg = theta_ref_deg;
  theta_ref_dot_raw_deg_s = constrain(theta_ref_dot_raw_deg_s, -10.0, 10.0);
  if (!theta_ref_dot_filter_initialized)
  {
    theta_ref_dot_f_deg_s = theta_ref_dot_raw_deg_s;
    theta_ref_dot_filter_initialized = true;
  }
  else if (pitch_des_local_rate_lpf_fc > 0.0f && dt > 1.0e-6f)
  {
    // 目标斜率的一阶低通：tau_theta_ref_dot_s=1/(2*pi*fc)，alpha=dt/(tau_theta_ref_dot_s+dt)。
    // 初次直接采用本次斜率，之后才按该公式递推；fc 非正则走下面的直通分支。
    float tau_theta_ref_dot_s =
    1.0f / ((2 * PI) * pitch_des_local_rate_lpf_fc);
    float alpha_theta_ref_dot = dt / (tau_theta_ref_dot_s + dt);
    theta_ref_dot_f_deg_s = theta_ref_dot_f_deg_s +
    alpha_theta_ref_dot * (theta_ref_dot_raw_deg_s -
        theta_ref_dot_f_deg_s);
  }
  else
  {
    theta_ref_dot_f_deg_s = theta_ref_dot_raw_deg_s;
  }
  // 4. 角度外环：theta_dot_cmd = 3 * (theta_ref-theta) + theta_ref_dot_f。
  //    固定系数 3.0 的单位为 1/s，表示每差 1 deg 给 3 deg/s 的修正；
  //    后一项让控制器主动跟随移动目标。这一环不使用俯仰 PID 的积分或增益。
  float e_theta_deg = theta_ref_deg - Units::toDegrees(att[1]);
  float theta_dot_cmd_deg_s = e_theta_deg * 3.0f + theta_ref_dot_f_deg_s;
  // 欧拉俯仰角变化率不总等于机体 q。按当前坐标关系：
  //   theta_dot = q*cos(phi) - r*sin(phi)
  // 所以 q_ref=(theta_dot_cmd+r_f*sin(phi))/cos(phi)，r_f 来自独立 Z 轴低通。
  // /57.3 将滚转角由 deg 换为 rad；当前式直接除以 cos(phi)，没有最小分母保护。
  float q_ref_deg_s = (theta_dot_cmd_deg_s + sin(Units::toDegrees(att[0]) / 57.3) *
      r_filter.apply(Units::toDegrees(ang_vel[2]))) /cos(Units::toDegrees(att[0]) / 57.3);
  // 限制的是机体 q 目标（+/-120 deg/s），不是前面的 +/-10 deg/s 目标斜率。
  q_ref_deg_s = constrain(q_ref_deg_s, -120, 120);
  // 5. 角速度误差转为角加速度参考：q_dot_ref=K_q*(q_ref-q_f)。
  //    q_dot_ref 表示角加速度参考值，与反馈 q_dot_meas 区分。
  //    K_q 单位 1/s；例如默认 25 时，1 deg/s 的误差要求 25 deg/s^2 的修正。
  float q_dot_ref_deg_s2 = indi_pitch_q_gain * (q_ref_deg_s - q_f_deg_s);
  pitchIndiControlOutput.q_dot_ref = Units::toRadians(q_dot_ref_deg_s2);
  pitchIndiControlOutput.q_ref = Units::toRadians(q_ref_deg_s);
  pitchIndiControlOutput.q_f = Units::toRadians(q_f_deg_s);
  pitchIndiControlOutput.isValid = true;
  // 6. INDI 核心：用实测角加速度作为当前响应，求“还差多少舵偏”。
  //    局部增量模型：q_dot_new ~= q_dot_meas + G_qe*(delta_e_new-delta_e_hat)。
  //    反演得到：delta_delta_e = (q_dot_ref-q_dot_meas)/G_qe；代码取其 70%。
  //    ang_acc[1] 已由传感器层差分、滤波，不是在这里对 q_ref 求导。
  //    delta_e_hat_deg 是舵机模型的实际舵偏估计，不是上次发出的目标舵偏。
  //    例：希望增加正俯仰角加速度、且当前响应不足，误差为正；
  //    默认 G=-113.65，求得负舵偏增量，即升降舵向上修正，方向为抬头。
  //    0.7 是修正幅度系数，不是积分开关，也不是低通截止频率。
  float delta_delta_e_deg =
  0.7f * (q_dot_ref_deg_s2 - Units::toDegrees(ang_acc[1])) / indi_pitch_effectiveness;
  float delta_e_cmd_deg = delta_e_hat_deg + delta_delta_e_deg;
  // 7. 约束舵面命令。先相对“上次最终命令”按 deg/s 限速，
  //    防止反演误差一次生成过大的动作。delta_e_step_max_deg=deg/s*秒，单位 deg。
  //    默认 800 deg/s、dt=0.002 s 时，每周期最多改变 1.6 deg。
  float delta_e_step_max_deg = indi_pitch_rate_limit_deg_s * dt;
  float delta_e_step_deg = delta_e_cmd_deg - delta_e_cmd_prev_deg;
  delta_e_step_deg = constrain(delta_e_step_deg, -delta_e_step_max_deg, delta_e_step_max_deg);
  delta_e_cmd_deg = delta_e_cmd_prev_deg + delta_e_step_deg;
  // 角度范围由通道3推导；先约束低通目标，再进行一次命令低通。
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, delta_e_min_deg, delta_e_max_deg);
  if (indi_pitch_cmd_lpf_fc_hz > 0.0f && dt > 1.0e-6f)
  {
    // 只在角度域低通一次，保持舵面运动的物理含义。
    float tau_delta_e_cmd_s = 1.0f / ((2 * PI) * indi_pitch_cmd_lpf_fc_hz);
    float alpha_delta_e_cmd = dt / (tau_delta_e_cmd_s + dt);
    delta_e_cmd_deg = delta_e_cmd_f_prev_deg +
        alpha_delta_e_cmd * (delta_e_cmd_deg - delta_e_cmd_f_prev_deg);
  }
  // 低通前限制目标；低通后确保 trim 在线修改导致范围变化时，旧滤波状态也不会越界。
  // 边界固定且上一状态合法时，一阶低通为凸组合，这第二次限幅通常不改变结果。
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, delta_e_min_deg, delta_e_max_deg);
  delta_e_cmd_prev_deg = delta_e_cmd_deg;
  delta_e_cmd_f_prev_deg = delta_e_cmd_deg;
  // 恢复物理PWM计算；k始终是正斜率大小，安装方向由rev负责。
  // rev只允许+/-1，因此这里除以rev与乘以rev等价。
  float pwm_cmd_float_us = pwm_trim_us +
      delta_e_cmd_deg / (pwm_channel3_rev * indi_pitch_pwm_to_deg_k);
  // 与推导角度边界使用同一通道3范围，也消除浮点换算可能出现的微小越界。
  pwm_cmd_float_us = constrain(pwm_cmd_float_us, pwm_servo3_min_us, pwm_servo3_max_us);
  // 8. 估计舵机实际到达的位置，供下一个控制周期作为增量反演基准。
  //    模型是“纯延迟 + 一阶响应”，并非读取了实际舵角，也不等同于命令瞬时到位。
  //    队列保存限幅后的浮点物理PWM，不在控制器内部量化为整数us。
  pwm_delay_buffer_us[pwm_delay_write_index] = pwm_cmd_float_us;
  pwm_delay_write_index = (pwm_delay_write_index + 1) % pwm_delay_buffer_length;
  int actuator_delay_steps = 0;
  if (dt > 1.0e-6f)
  {
    // 将延迟秒数折算为最近整数个采样周期；默认 0.03 s、dt=0.002 s 时为15。
    actuator_delay_steps = (int)(indi_pitch_servo_delay_s / dt + 0.5f);
  }
  // 固定64个队列元素最多保留63个周期的延迟，实际时间随 dt 改变。
  actuator_delay_steps = constrain(actuator_delay_steps, 0, pwm_delay_buffer_length - 1);
  // 写入后pwm_delay_write_index指向下个写位置：减1回到本次命令，再减延迟步数。
  // actuator_delay_steps=0 时读本次命令；大于0时读更早的命令。
  int pwm_delay_read_index = pwm_delay_write_index - 1 - actuator_delay_steps;
  if (pwm_delay_read_index < 0)
  {
    pwm_delay_read_index += pwm_delay_buffer_length;
  }
  const float pwm_delayed_us = pwm_delay_buffer_us[pwm_delay_read_index];
  const float delta_e_delayed_deg =
      (pwm_delayed_us - pwm_trim_us) * pwm_channel3_rev * indi_pitch_pwm_to_deg_k;
  // 一阶舵机模型：估计舵偏逐渐追随“延迟后的目标舵偏”。
  // tau_actuator_s 很小时按即时跟随处理；否则 alpha=dt/(tau_actuator_s+dt)，tau_actuator_s 越大响应越慢。
  float tau_actuator_s = indi_pitch_servo_tau_s;
  if (tau_actuator_s < 1.0e-4f)
  {
    delta_e_hat_deg = delta_e_delayed_deg;
  }
  else
  {
    float alpha_actuator = dt / (tau_actuator_s + dt);
    delta_e_hat_deg += alpha_actuator * (delta_e_delayed_deg - delta_e_hat_deg);
  }
  // 模型只使用INDI自身的PWM命令，不包含稍后Mixer叠加的主机前馈。
  // 9. 发布控制过程量。delta_e_cmd 是命令，delta_e_hat 是模型估计，
  //     正常情况下二者会因延迟和舵机响应不同，不应当作同一个量理解。
  // 角速度目标、反馈及角加速度目标已在步骤 5 发布，不再重复保存一份。
  pitchIndiControlOutput.q_dot_meas = ang_acc[1];
  pitchIndiControlOutput.delta_e_cmd = Units::toRadians(delta_e_cmd_deg);
  pitchIndiControlOutput.delta_e_hat = Units::toRadians(delta_e_hat_deg);
  // 出口取负以统一为抬头正；主/从 mixer 选用时再取负，恢复下偏正的舵面坐标。
  // 不在控制器内量化到整数 us；混控后统一应用 rev、trim 和最终物理 PWM 限幅。
  pitchIndiControlOutput.indiOutput =
      -actuatorPwmToCommand(pwm_cmd_float_us, pwm_channel3_rev,
          pwm_servo3_min_us, pwm_channel3_trim, pwm_servo3_max_us, pwm_channel3_mode);
}
