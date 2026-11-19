#pragma once

#include "config/flight_config.h"
#include <Arduino.h>
void radioSetup();
// 链路任务独占接收器，控制任务在周期末锁存一帧，供下一周期使用。
void pollRadioInput();
void latchRadioInput();

// Eight SD-backed normalization parameter groups. PWM values are in microseconds.
struct RadioChannelParameters {
  float minimum, maximum, trim, reverse, deadzone;
};
extern RadioChannelParameters radioChannelParameters[8];
extern int radio_channel1_raw, radio_channel2_raw, radio_channel3_raw, radio_channel4_raw;
extern int radio_channel5_raw, radio_channel6_raw, radio_channel7_raw, radio_channel8_raw;
extern float radio_channel1_normalized, radio_channel2_normalized, radio_channel3_normalized, radio_channel4_normalized;
extern float radio_channel5_normalized, radio_channel6_normalized, radio_channel7_normalized, radio_channel8_normalized;
void updateRadioNormalized();
// Four primary channels have recent, plausible receiver measurements.
bool radioInputValid();
