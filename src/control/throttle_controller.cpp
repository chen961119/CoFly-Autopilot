#include "control/throttle_controller.h"
#include "control/control_output.h"
#include "parameters/control_parameters.h"
#include <stdint.h>
#include <string.h>

namespace {
// 发布版本启用 -ffast-math，使用位检查避免 isfinite 被优化为恒真。
bool finiteFloat(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7f800000u) != 0x7f800000u;
}
float clamp(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}
} // namespace

void ThrottleController::update(float throttleTarget, float measuredAirspeed,
                                bool feedbackValid, bool enabled,
                                bool integralEnabled, float dt) {
  using namespace ControlParameters;
  auto &output = throttleControlOutput;
  output = {};
  const float stick = finiteFloat(throttleTarget) ? clamp(throttleTarget, 0, 1) : 0;
  output.throttleCommand = stick; // 手动、禁用或无有效反馈时直通。
  const bool validDt = finiteFloat(dt) && dt > 0 && dt <= 0.1f;
  const bool validFeedback = feedbackValid && finiteFloat(measuredAirspeed) &&
      measuredAirspeed >= 0 && measuredAirspeed <= 100 && validDt;
  const bool validParameters = finiteFloat(airspeed_min) && finiteFloat(airspeed_max) &&
      airspeed_min > 0 && airspeed_max > airspeed_min && airspeed_max <= 100 &&
      finiteFloat(airspeed_lpf_hz) && airspeed_lpf_hz >= 0 && airspeed_lpf_hz <= 20 &&
      finiteFloat(Kp_airspeed) && Kp_airspeed >= 0 && Kp_airspeed <= 1 &&
      finiteFloat(Ki_airspeed) && Ki_airspeed >= 0 && Ki_airspeed <= 1 &&
      finiteFloat(throttle_cruise) && throttle_cruise >= 0 && throttle_cruise <= 1;

  if (validFeedback && validParameters) {
    // 50 Hz 空速在控制周期之间零阶保持；按实际控制 dt 做一阶低通。
    if (!filterInitialized || airspeed_lpf_hz == 0) filteredAirspeed = measuredAirspeed;
    else {
      const float tau = 1.0f / (6.2831853f * airspeed_lpf_hz);
      filteredAirspeed += dt / (tau + dt) * (measuredAirspeed - filteredAirspeed);
    }
    filterInitialized = true;
    output.filteredAirspeed = filteredAirspeed;
  } else {
    filterInitialized = false; // 恢复反馈时重新初始化，避免使用中断前的滤波历史。
  }

  if (!enabled || airspeed_control_enabled == 0 || !validFeedback || !validParameters ||
      stick <= 0.02f) {
    integral = 0;
    if (enabled && airspeed_control_enabled != 0 && stick <= 0.02f)
      output.throttleCommand = 0;
    return;
  }

  output.speedControlActive = true;
  output.airspeedTarget = airspeed_min + stick * (airspeed_max - airspeed_min);
  output.airspeedError = output.airspeedTarget - filteredAirspeed;
  const float proportional = Kp_airspeed * output.airspeedError;
  if (!integralEnabled || Ki_airspeed == 0) integral = 0;
  else {
    const float candidate = clamp(integral + Ki_airspeed * output.airspeedError * dt, -0.3f, 0.3f);
    const float candidateOutput = throttle_cruise + proportional + candidate;
    // 饱和时只允许积分往退出饱和的方向变化。
    if ((candidateOutput <= 1 || candidate < integral) &&
        (candidateOutput >= 0 || candidate > integral)) integral = candidate;
  }
  output.integralTerm = integral;
  output.throttleCommand = clamp(throttle_cruise + proportional + integral, 0, 1);
}
