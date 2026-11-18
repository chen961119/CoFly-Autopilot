#include "math/units.h"
#include "sensors/filters.h"
#include "config/flight_config.h"
#include "sensors/sensor_processing.h"
#include "sensors/imu_internal.h"
#include "math/math_utils.h"
#include "math/math_constants.h"
#include "system/flight_clock.h"

namespace {
PX4LowPassFilter2p gyroLowpass[3];

struct GyroNotch {
  void configure(float center_hz, float bandwidth_hz) {
    const float beta = tanf(PI * bandwidth_hz / CONTROL_LOOP_RATE_HZ);
    gain = beta / (1.0f + beta);
    a1 = -2.0f * cosf(2.0f * PI * center_hz / CONTROL_LOOP_RATE_HZ) *
         (1.0f - gain);
    a2 = 1.0f - 2.0f * gain;
  }

  float apply(unsigned int axis, float sample, bool reset) {
    State &s = state[axis];
    if (reset) {
      // 将带通支路置于当前输入的直流稳态。
      s.z1 = s.z2 = -gain * sample;
      return sample;
    }
    // 输入减去二阶带通的输出，形成单位直流增益的陷波。
    const float drive = gain * sample;
    const float band = drive + s.z1;
    s.z1 = s.z2 - a1 * band;
    s.z2 = -drive - a2 * band;
    return sample - band;
  }

  struct State { float z1, z2; };
  State state[3]{};
  float gain{0.0f}, a1{0.0f}, a2{0.0f};
};

GyroNotch gyroNotches[3];
bool gyroNotchesInitialized = false;
}

// 按采样频率和截止频率（Hz）计算二阶低通系数；非正参数时改为直接透传。
void PX4LowPassFilter2p::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
  if (sample_freq <= 0.0f || cutoff_freq <= 0.0f) {
    _b0 = 1.0f; _b1 = 0.0f; _b2 = 0.0f;
    _a1 = 0.0f; _a2 = 0.0f;
    return;
  }
  const float fr = sample_freq / cutoff_freq;
  const float ohm = tanf(PI / fr);
  const float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
  _b0 = ohm * ohm / c;
  _b1 = 2.0f * _b0;
  _b2 = _b0;
  _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  _a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
}

// 输入一个新样本，更新二阶滤波器内部状态并返回滤波结果。
float PX4LowPassFilter2p::apply(float sample) {
  float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
  float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
  _delay_element_2 = _delay_element_1;
  _delay_element_1 = delay_element_0;
  return output;
}

// 将滤波器延迟状态重置到指定样本对应的稳态值。
void PX4LowPassFilter2p::reset(float sample) {
  _delay_element_1 = _delay_element_2 = sample / (1.0f + _a1 + _a2);
}

// 各机在本地采样频率下生成所有控制器共用的角速度反馈。
void initializeControlFilters() {
  constexpr float feedbackCutoffHz = 40.0f;
  // 2026-11-12 刚接柔性翼试飞，按 A 机日志的 current_time 校正报告峰频。
  // 参数依次为中心频率和单节 -3 dB 带宽，单位 Hz。
  gyroNotches[0].configure(5.80f, 0.55f);
  gyroNotches[1].configure(7.72f, 0.55f);
  gyroNotches[2].configure(9.64f, 0.65f);
  gyroNotchesInitialized = false;
  for (auto &lowpass : gyroLowpass) {
    lowpass.set_cutoff_frequency(CONTROL_LOOP_RATE_HZ, feedbackCutoffHz);
    lowpass.reset(0.0f);
  }
  for (float &value : ang_vel_filtered) value = 0.0f;
  initializeAngularAccelerationFilters();
}

// 每次传感器测量发布时推进一次，控制器仅读取结果，不重复推进滤波状态。
void updateAngularVelocityFilters() {
  const bool resetNotches = !gyroNotchesInitialized || dt <= 0.0f || dt > 0.01f;
  for (unsigned int axis = 0; axis < 3; ++axis) {
    float rate = ang_vel[axis];
    for (auto &notch : gyroNotches) {
      rate = notch.apply(axis, rate, resetNotches);
    }
    ang_vel_filtered[axis] = gyroLowpass[axis].apply(rate);
  }
  gyroNotchesInitialized = true;
}

float B_madgwick_adaptive = 0.04; // 动态权重
float base_B_madgwick = 0.04;     // 基准权重

float att[3] = {};

float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // W/X/Y/Z，初始机体水平。


// 内置 IMU 使用六轴 Madgwick 姿态融合；外置 IMU 直接提供姿态。
void Madgwick() {
#if defined INTIMU && !defined EXTIMU
  Madgwick6DOF(Units::toDegrees(GyroX_6050), Units::toDegrees(GyroY_6050), Units::toDegrees(GyroZ_6050),
               Units::gravityUnits(AccX_6050), Units::gravityUnits(AccY_6050),
               Units::gravityUnits(AccZ_6050), dt);
#endif
}

// 六轴姿态融合，加速度计修正姿态，陀螺仪积分预测姿态。
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,
                  float invSampleFreq) {
  // 六轴姿态融合：无磁力计时只用角速度与重力方向修正姿态。
  // 加速度模长偏离 1 g 越多，越降低加速度反馈权重。
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // 陀螺仪由 deg/s 换算为 rad/s。
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // 根据角速度计算四元数变化率。
  qDot1 = 0.5f * (-quat[1] * gx - quat[2] * gy - quat[3] * gz);
  qDot2 = 0.5f * (quat[0] * gx + quat[2] * gz - quat[3] * gy);
  qDot3 = 0.5f * (quat[0] * gy - quat[1] * gz + quat[3] * gx);
  qDot4 = 0.5f * (quat[0] * gz + quat[1] * gy - quat[2] * gx);

  // 加速度计全零时不计算反馈，避免除以零。
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 1. 计算加速度向量的模
    float acc_norm = sqrt(ax * ax + ay * ay + az * az);

    // 2. 计算与 1.0g 的偏离值
    float acc_error = abs(acc_norm - 1.0f);

    // 3. 动态调整权重：如果偏离超过 0.1g，开始线性减小权重
    // 当偏离达到 0.5g 时，权重降至极低，几乎完全信任陀螺仪积分
    if (acc_error < 0.1f) {
      B_madgwick_adaptive = base_B_madgwick;
    } else {
      // 线性插值：偏离 0.1g~0.5g 之间，权重从 0.04 降到 0.001
      B_madgwick_adaptive =
          base_B_madgwick *
          (1.0f - constrain((acc_error - 0.1f) / 0.4f, 0.0f, 0.98f));
    }
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 缓存重复使用的四元数乘积。
    _2q0 = 2.0f * quat[0];
    _2q1 = 2.0f * quat[1];
    _2q2 = 2.0f * quat[2];
    _2q3 = 2.0f * quat[3];
    _4q0 = 4.0f * quat[0];
    _4q1 = 4.0f * quat[1];
    _4q2 = 4.0f * quat[2];
    _8q1 = 8.0f * quat[1];
    _8q2 = 8.0f * quat[2];
    q0q0 = quat[0] * quat[0];
    q1q1 = quat[1] * quat[1];
    q2q2 = quat[2] * quat[2];
    q3q3 = quat[3] * quat[3];

    // 梯度下降得到重力方向误差。
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * quat[1] - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * quat[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * quat[3] - _2q1 * ax + 4.0f * q2q2 * quat[3] - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // 归一化校正梯度。
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 用自适应权重修正陀螺仪积分结果。

    qDot1 -= B_madgwick_adaptive * s0;
    qDot2 -= B_madgwick_adaptive * s1;
    qDot3 -= B_madgwick_adaptive * s2;
    qDot4 -= B_madgwick_adaptive * s3;
  }

  // 积分并归一化四元数。
  quat[0] += qDot1 * invSampleFreq;
  quat[1] += qDot2 * invSampleFreq;
  quat[2] += qDot3 * invSampleFreq;
  quat[3] += qDot4 * invSampleFreq;

  recipNorm = invSqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
  quat[0] *= recipNorm;
  quat[1] *= recipNorm;
  quat[2] *= recipNorm;
  quat[3] *= recipNorm;

  // 输出角度；滚转角按当前安装方向取反。
  att[0] = -atan2(quat[0] * quat[1] + quat[2] * quat[3], 0.5f - quat[1] * quat[1] - quat[2] * quat[2]) ; // rad。
  att[1] =
      asin(constrain(-2.0f * (quat[1] * quat[3] - quat[0] * quat[2]), -0.999999, 0.999999)) ; // rad。
  att[2] = - atan2(quat[1] * quat[2] + quat[0] * quat[3], 0.5f - quat[2] * quat[2] - quat[3] * quat[3]) ; // rad。
}
