#pragma once
// 偏航角速度 PID：控制机体 Z 轴角速度，不控制偏航角。
// 原控制律及历史由本对象保存；参数继续从 ControlParameters 读取。
class YawRatePidController {
public:
  void prepare(bool controlModeChanged);
  // yawRateTarget 为 rad/s；rollTarget、pitchTarget 为 rad，用于计算协调转弯角速度前馈。
  void update(float yawRateTarget, float rollTarget, float pitchTarget, bool enabled);

private:
  bool derivativeInitialized = false;
  float error_yaw_rate = 0;
  float error_yaw_rate_prev = 0;
  float integral_yaw_rate = 0;
  float integral_yaw_rate_prev = 0;
  float derivative_yaw_rate = 0;
  float proportion_yaw_rate = 0;
  float feedforward_yaw_rate = 0;
  float yaw_rate_PID = 0;
};
