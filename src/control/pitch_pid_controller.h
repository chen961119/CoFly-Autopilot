#pragma once
// 原控制律及历史由本对象保存；参数继续从 ControlParameters 读取。
class PitchPidController {
public:
  void prepare(bool integralControlEnabled, bool controlModeChanged);
  void update(float pitchTarget, bool enabled);

private:
  bool ModeChange = false;
  float i_valid = 0;
  float previous_pitch_att = 0;
  float error_pitch = 0;
  float error_pitch_prev = 0;
  float pitch_des_prev = 0;
  float integral_pitch_ol = 0;
  float integral_pitch_prev_ol = 0;
  float integral_pitch_il = 0;
  float integral_pitch_prev_il = 0;
  float derivative_pitch = 0;
  float pitch_PID = 0;
};
