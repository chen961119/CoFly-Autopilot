#include "control/throttle_controller.h"
#include "control/control_output.h"
#include "parameters/control_parameters.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

using namespace ControlParameters;

static void near(float actual, float expected, float tolerance = 1e-5f) {
  assert(std::fabs(actual - expected) < tolerance);
}

int main() {
  auto &out = throttleControlOutput;
  ThrottleController controller;
  auto step = [&](float speed, float stick = 0.5f, bool valid = true,
                  bool enabled = true, bool integralEnabled = true, float dt = 0.02f) {
    controller.update(stick, speed, valid, enabled, integralEnabled, dt);
  };

  // 杆量映射、速度误差方向，禁用积分时检查纯 P。
  airspeed_lpf_hz = 0;
  step(15, 0.5f, true, true, false);
  near(out.airspeedTarget, 15);
  near(out.throttleCommand, 0.5f);
  step(10, 0.5f, true, true, false);
  near(out.throttleCommand, 0.7f);
  step(20, 0.5f, true, true, false);
  near(out.throttleCommand, 0.3f);
  step(22, 1, true, true, false);
  near(out.airspeedTarget, 22);

  // 长时间低速不积累饱和积分；恢复目标速度后不保持最大油门。
  controller = {};
  for (int i = 0; i < 10000; ++i) step(0, 1);
  near(out.throttleCommand, 1);
  near(out.integralTerm, 0);
  step(22, 1);
  near(out.throttleCommand, 0.5f);
  for (int i = 0; i < 10000; ++i) step(100);
  near(out.throttleCommand, 0);
  near(out.integralTerm, 0);

  // I 项限制、总积分开关、手动/锁定和反馈故障时清状态。
  for (int i = 0; i < 10000; ++i) step(14);
  near(out.integralTerm, 0.3f);
  step(15, 0.5f, true, true, false);
  near(out.integralTerm, 0);
  for (int i = 0; i < 10000; ++i) step(16);
  near(out.integralTerm, -0.3f);
  step(10, 0.4f, true, false);
  near(out.throttleCommand, 0.4f);
  assert(!out.speedControlActive);
  step(15);
  near(out.integralTerm, 0);
  step(10);
  step(10, 0.4f, false);
  near(out.throttleCommand, 0.4f);
  assert(!out.speedControlActive);
  step(15);
  near(out.integralTerm, 0);

  // 近零杆不因低速自动加油；参数开关可恢复增稳油门直通。
  step(0, 0.02f);
  near(out.throttleCommand, 0);
  assert(!out.speedControlActive);
  airspeed_control_enabled = 0;
  step(10, 0.4f);
  near(out.throttleCommand, 0.4f);
  airspeed_control_enabled = 1;

  // 非法反馈、周期和参数不进入 PI；涵盖发布版本 -ffast-math。
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (float speed : {nan, inf, -1.0f, 101.0f}) {
    step(speed);
    near(out.throttleCommand, 0.5f);
    assert(!out.speedControlActive);
  }
  for (float dt : {0.0f, -1.0f, 0.2f, nan, inf}) {
    step(10, 0.5f, true, true, true, dt);
    near(out.throttleCommand, 0.5f);
    assert(!out.speedControlActive);
  }
  step(10, nan);
  near(out.throttleCommand, 0);
  airspeed_max = airspeed_min;
  step(10);
  assert(!out.speedControlActive);
  airspeed_max = 22;

  // 2 Hz 低通削弱一次测量跳变；故障恢复重新初始化反馈。
  controller = {};
  airspeed_lpf_hz = 2;
  step(10);
  near(out.filteredAirspeed, 10);
  step(20);
  assert(out.filteredAirspeed > 10 && out.filteredAirspeed < 13);
  step(20, 0.5f, false);
  step(20);
  near(out.filteredAirspeed, 20);

  // 简化一阶飞机模型（3 s 时常）+ 恒定阻力扰动，验证 PI 消除稳态误差。
  controller = {};
  float speed = 12;
  float sample = speed;
  for (int i = 0; i < 60000; ++i) {
    if (i % 10 == 0) sample = speed; // 50 Hz 采样，500 Hz 控制。
    step(sample, 0.5f, true, true, true, 0.002f);
    assert(out.throttleCommand >= 0 && out.throttleCommand <= 1);
    speed += 0.002f / 3 * (30 * out.throttleCommand - 3 - speed);
  }
  near(speed, 15, 0.02f);
  near(out.throttleCommand, 0.6f, 0.002f);
  puts("Throttle controller checks passed (mapping, PI, filtering, fallback, reset, simulation).");
}
