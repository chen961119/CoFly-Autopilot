#pragma once
// 原控制律及历史由本对象保存；参数继续从 ControlParameters 读取。
class RollPidController {
public:
  void prepare(bool integralControlEnabled, bool controlModeChanged);
  void update(float rollTarget, bool enabled);
private:
  bool ModeChange = false;
  float i_valid = 0;
  bool roll_pid_dot_initialized = false;
  float roll_eq = 0;
  float previous_roll_att = 0;
  float error_roll = 0;
  float error_roll_prev = 0;
  float roll_des_prev = 0;
  float integral_roll_ol = 0;
  float integral_roll_prev_ol = 0;
  float integral_roll_il = 0;
  float integral_roll_prev_il = 0;
  float derivative_roll = 0;
  float roll_PID = 0;
  float roll_PID_lpf = 0;
  float roll_PID_prev = 0;
  float roll_PID_dot = 0;
  float roll_PID_dot_lpf = 0;
};
