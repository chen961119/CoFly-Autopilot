#pragma once

// mode=0: min/trim/max 三点映射；mode=1: min/max 两点映射，trim 仅为安全位置。
// mode=0 接收 -1..1，mode=1 接收 0..1；超范围混控量由输出层最终限幅。
inline float actuatorCommandToPwm(float command, int reverse, float minimum,
                                  float trim, float maximum, int mode) {
  if (mode == 1) {
    const float u = reverse < 0 ? 1.0f - command : command;
    return minimum + u * (maximum - minimum);
  }
  const float u = reverse * command;
  return trim + u * (u >= 0.0f ? maximum - trim : trim - minimum);
}

// INDI 的物理零舵角：模式 1 取端点中点，对应 command=0.5，trim 仅为安全位置。
inline float actuatorNeutralPwm(float minimum, float trim, float maximum, int mode) {
  return mode == 1 ? (minimum + maximum) * 0.5f : trim;
}
// INDI 的物理 PWM 逆变换，零行程返回 0，避免除以零。
inline float actuatorPwmToCommand(float pwm, int reverse, float minimum,
                                  float trim, float maximum, int mode) {
  if (mode == 1) {
    const float span = maximum - minimum;
    if (span <= 0.0f) return 0.0f;
    const float u = (pwm - minimum) / span;
    return reverse < 0 ? 1.0f - u : u;
  }
  const float offset = pwm - trim;
  const float span = offset >= 0.0f ? maximum - trim : trim - minimum;
  return span > 0.0f ? reverse * offset / span : 0.0f;
}

