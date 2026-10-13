#pragma once

#include "config/flight_config.h"
#include <Arduino.h>
#include <SBUS.h>
#include <DSMRX.h>

// 接收机硬件实例由 radio_comm.cpp 持有；接收类型由 flight_config.h 统一选择。
#if defined USE_SBUS_RX
extern SBUS sbus;
extern uint16_t sbusChannels[16];
extern bool sbusFailSafe, sbusLostFrame;
#endif
#if defined USE_DSM_RX
extern DSM1024 DSM;
#endif

void radioSetup();
// 读取接收机通道并更新滤波后的 radio_channelN_raw，供下一控制周期使用。
void getCommands();
unsigned long getRadioPWM(int ch_num);

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
