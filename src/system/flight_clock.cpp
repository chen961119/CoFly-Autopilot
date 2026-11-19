#include "system/flight_clock.h"
#include "system/rtos_sync.h"
#include <Arduino.h>

float dt = 0.0f;
unsigned long current_time = 0;
namespace {
FlightTimingStats timing{};
uint32_t periodUs = 2000;
bool clockStarted = false;
uint32_t previousCycles;
}

// 记录本周期起始时间，并计算相邻两周期的时间差 dt（秒）。
void updateFlightClock() {
  // micros() 的无符号减法可跨计数器回绕计算间隔，dt 单位为秒。
  const uint32_t nowCycles = ARM_DWT_CYCCNT;
  const uint32_t previous = current_time;
  current_time = micros();
  // The cycle counter continues while a flash operation masks SysTick.
  // Use the Arduino clock for intervals longer than one DWT revolution.
  const uint32_t clockDelta = current_time - previous;
  if (clockStarted) {
    const uint32_t cyclesPerUs = F_CPU_ACTUAL / 1000000u;
    periodUs = clockDelta >= UINT32_MAX / cyclesPerUs
        ? clockDelta : (nowCycles - previousCycles) / cyclesPerUs;
  }
  previousCycles = nowCycles;
  clockStarted = true;
  dt = periodUs * 1.0e-6f;
}

void finishFlightCycle(uint32_t imuToPwmUs, uint32_t executionUs, uint32_t missed) {
  TaskCriticalSection guard;
  ++timing.cycles;
  timing.totalPeriodUs += periodUs;
  timing.missedReleases += missed;
  if (periodUs >= 4000u) ++timing.intervalsAtLeast4ms;
  if (executionUs > 2000u) ++timing.executionsOver2ms;
  if (periodUs > timing.maxPeriodUs) timing.maxPeriodUs = periodUs;
  if (executionUs > timing.maxExecutionUs) timing.maxExecutionUs = executionUs;
  timing.lastImuToPwmUs = imuToPwmUs;
  if (imuToPwmUs > timing.maxImuToPwmUs) timing.maxImuToPwmUs = imuToPwmUs;
}

FlightTimingStats flightTimingStats() {
  TaskCriticalSection guard;
  return timing;
}

// 从飞行时钟 current_time 起忙等到目标周期时长；freq 单位为 Hz。
// 若处理已超时则立即返回，不会补偿遗漏的周期。
void loopRate(int freq) {
  // micros() 的无符号差值可处理计数器回绕；忙等期间 CPU 不执行其他循环任务。
  const float intervalMicros = 1000000.0f / freq;
  unsigned long checker = micros();
  while (intervalMicros > (checker - current_time)) {
    checker = micros();
  }
}
