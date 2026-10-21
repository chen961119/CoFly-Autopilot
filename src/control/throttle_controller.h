#pragma once

class ThrottleController {
public:
  // 油门杆 0～1；空速 m/s；dt 秒。主机按增稳模式传入 enabled，不包含锁定判断。
  void update(float throttleTarget, float measuredAirspeed, bool feedbackValid,
              bool enabled, bool integralEnabled, float dt);

private:
  float filteredAirspeed = 0;
  float integral = 0; // 直接保存油门尺度的 I 项，限制为 ±0.3。
  bool filterInitialized = false;
};
