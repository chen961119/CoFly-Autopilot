#pragma once

class ThrottleController {
public:
  // 油门杆 0～1；空速 m/s；dt 秒。仅主机在增稳且解锁时启用闭环。
  void update(float throttleTarget, float measuredAirspeed, bool feedbackValid,
              bool enabled, bool integralEnabled, float dt);

private:
  float filteredAirspeed = 0;
  float integral = 0; // 直接保存油门尺度的 I 项，限制为 ±0.3。
  bool filterInitialized = false;
};
