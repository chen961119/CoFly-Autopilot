#pragma once
#include "control/control_types.h"
#include "sensors/filters.h"
// 单机俯仰 INDI：保存跨周期状态，使用全局姿态/角速度/角加速度和 dt 计算。
// 内部用物理PWM计算；模式0的indiOutput为抬头正归一化控制量，mixer取负后作为elevator。
class PitchIndiController {
public:
  PitchIndiController();
  // theta_ref_deg 单位 deg、抬头为正；记录目标差分起点，下一次有效更新重建舵机状态。
  void reset(float theta_ref_deg);
  // enabled=false 只清空发布结果，不推进内部状态；切换时的历史重置由 reset 负责。
  void update(float theta_ref_deg, bool enabled);
private:
  // 延迟队列保存浮点物理 PWM，初始化为映射零点（模式 0 为 trim，模式 1 为端点中点）。
  static constexpr int pwm_delay_buffer_length = 64;
  // 是否已用零舵角初始化队列、舵偏估计及命令历史。
  bool initialized = false;
  float pwm_delay_buffer_us[pwm_delay_buffer_length] = {};
  // 下次写入的环形队列位置；不是最旧命令的固定下标。
  int pwm_delay_write_index = 0;
  // 均为 deg、下偏为正：模型估计舵偏（增量基准）、上次最终命令舵偏（限速基准）。
  float delta_e_hat_deg = 0, delta_e_cmd_prev_deg = 0;
  // 上次舵偏命令低通状态，deg；命令只在角度域低通一次。
  float delta_e_cmd_f_prev_deg = 0;
  // 目标角变化率的一阶低通状态，与机体角速度反馈滤波相互独立。
  bool theta_ref_dot_filter_initialized = false;
  // 上次目标角（deg）和滤波后的目标角变化率（deg/s）。
  float theta_ref_prev_deg = 0, theta_ref_dot_f_deg_s = 0;
  // 本对象的二阶低通：Y 轴用于 q 反馈，Z 轴用于欧拉角变化率转换的耦合项。
  // 独立于传感器模块的全局滤波器；reset() 不重置此处 r_filter 的历史。
  PX4LowPassFilter2p q_filter, r_filter;
};
