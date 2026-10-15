#pragma once
// 原控制律及历史由本对象保存；参数继续从 ControlParameters 读取。
class YawPidController {
public:
  void prepare(bool integralEnabled, bool changed);
  void update(float yawTarget, float rollTarget, float pitchTarget, bool enabled);

private:
  bool ModeChange = false;
  float i_valid = 0;
  float error_yaw = 0;
  float error_yaw_prev = 0;
  float integral_yaw = 0;
  float integral_yaw_prev = 0;
  float derivative_yaw = 0;
  float yaw_PID = 0;
};
