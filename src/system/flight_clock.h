#pragma once
#include <stdint.h>

struct FlightTimingStats {
  uint32_t cycles, missedReleases, maxPeriodUs, maxExecutionUs;
  uint32_t maxImuToPwmUs, lastImuToPwmUs;
  uint32_t intervalsAtLeast4ms, executionsOver2ms;
  uint64_t totalPeriodUs;
};
FlightTimingStats flightTimingStats();
void finishFlightCycle(uint32_t imuToPwmUs, uint32_t executionUs, uint32_t missed);

// current_time 为本轮起点的 micros() 时间戳（μs），dt 为相邻两轮间隔（秒）。
extern float dt;
extern unsigned long current_time;

void updateFlightClock(); // 使用无符号差值计算，兼容 micros() 回绕。

// 使用 current_time 忙等到本轮达到目标周期；freq 单位为 Hz。
// 若本轮已超时则立即返回，无法补偿超时。
void loopRate(int freq);
