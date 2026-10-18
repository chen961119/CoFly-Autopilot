#include "config/serial_config.h"
// 遥控接收模块：支持编译配置选定的 PPM、PWM 或 SBUS 接收机。
// 原始实现来自 Nicholas Rehm 的 dRehmFlight 项目，后续按本项目硬件接线调整。
#include <Arduino.h>
#include "radio_comm/radio_comm.h"
#include <SBUS.h>  // SBUS 接收机驱动。
// 以下引脚用于 PWM 接收机；SBUS 实际使用 SbusSerial。
const int ch1Pin = 15; // 通道 1：滚转
const int ch2Pin = 16; // 通道 2：俯仰
const int ch3Pin = 17; // 通道 3：油门
const int ch4Pin = 20; // 通道 4：偏航
const int ch5Pin = 21; // 通道 5：油门切断
const int ch6Pin = 22; // 通道 6：辅助功能
const int PPM_Pin = 23;

static unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
static volatile unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
static int radio_channel1_raw_prev, radio_channel2_raw_prev, radio_channel3_raw_prev,
    radio_channel4_raw_prev;

static int ppm_counter = 0;
static unsigned long time_ms = 0;
static volatile uint32_t radioReceivedAt[4];
static volatile bool radioReceived[4];

static void markRadioReceived(unsigned channel) {
  radioReceivedAt[channel] = micros();
  radioReceived[channel] = true;
}

bool radioInputValid() {
  const uint32_t now = micros();
  for (unsigned i = 0; i < 4; ++i)
    if (!radioReceived[i] || now - radioReceivedAt[i] > 250000u) return false;
#if defined USE_SBUS_RX
  if (sbusFailSafe || sbusLostFrame) return false;
#endif
  const int values[] = {radio_channel1_raw, radio_channel2_raw, radio_channel3_raw, radio_channel4_raw};
  for (int value : values) if (value < 800 || value > 2200) return false;
  return true;
}


#if defined USE_SBUS_RX
SBUS sbus(SbusSerial);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;
#endif

// 按编译配置初始化 PPM、PWM 或 SBUS 接收机及相应中断。
void radioSetup() {
  // PPM：单引脚中断，按相邻上升沿的间隔分离各通道。
  #if defined USE_PPM_RX
    // 配置中断输入引脚。
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    // 每次边沿变化由中断服务函数解析。
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

  // PWM：每个通道分别测量高电平脉宽。
  #elif defined USE_PWM_RX
    // 配置各通道输入引脚。
    pinMode(ch1Pin, INPUT_PULLUP);
    pinMode(ch2Pin, INPUT_PULLUP);
    pinMode(ch3Pin, INPUT_PULLUP);
    pinMode(ch4Pin, INPUT_PULLUP);
    pinMode(ch5Pin, INPUT_PULLUP);
    pinMode(ch6Pin, INPUT_PULLUP);
    delay(20);
    // 为每个通道绑定边沿中断。
    attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
    delay(20);

  // SBUS 接收机。
  #elif defined USE_SBUS_RX
    sbus.begin();

  #else
    // 未配置接收机类型时不执行初始化。
  #endif
}

// Names retain the original PWM-equivalent values and existing filtering.
int radio_channel1_raw, radio_channel2_raw, radio_channel3_raw, radio_channel4_raw;
int radio_channel5_raw, radio_channel6_raw, radio_channel7_raw, radio_channel8_raw;
float radio_channel1_normalized, radio_channel2_normalized, radio_channel3_normalized, radio_channel4_normalized;
float radio_channel5_normalized, radio_channel6_normalized, radio_channel7_normalized, radio_channel8_normalized;
RadioChannelParameters radioChannelParameters[8] = {
    {1000, 2000, 1500, 1, 10}, {1000, 2000, 1500, 1, 10},
    {1000, 2000, 1500, 1, 10}, {1000, 2000, 1500, 1, 10},
    {1000, 2000, 1500, 1, 10}, {1000, 2000, 1500, 1, 10},
    {1000, 2000, 1500, 1, 10}, {1000, 2000, 1500, 1, 10}};

static float normalizeRadioChannel(int raw, const RadioChannelParameters &p) {
  const float delta = raw - p.trim;
  float value = 0;
  if (delta > p.deadzone) {
    const float span = p.maximum - p.trim - p.deadzone;
    if (span > 0) value = (delta - p.deadzone) / span;
  } else if (delta < -p.deadzone) {
    const float span = p.trim - p.minimum - p.deadzone;
    if (span > 0) value = (delta + p.deadzone) / span;
  }
  return constrain(value * p.reverse, -1.0f, 1.0f);
}

void updateRadioNormalized() {
  radio_channel1_normalized = normalizeRadioChannel(radio_channel1_raw, radioChannelParameters[0]);
  radio_channel2_normalized = normalizeRadioChannel(radio_channel2_raw, radioChannelParameters[1]);
  radio_channel3_normalized = normalizeRadioChannel(radio_channel3_raw, radioChannelParameters[2]);
  radio_channel4_normalized = normalizeRadioChannel(radio_channel4_raw, radioChannelParameters[3]);
  radio_channel5_normalized = normalizeRadioChannel(radio_channel5_raw, radioChannelParameters[4]);
  radio_channel6_normalized = normalizeRadioChannel(radio_channel6_raw, radioChannelParameters[5]);
  radio_channel7_normalized = normalizeRadioChannel(radio_channel7_raw, radioChannelParameters[6]);
  radio_channel8_normalized = normalizeRadioChannel(radio_channel8_raw, radioChannelParameters[7]);
}

// 按接收机类型读取各遥控通道并进行低通；更新的值供下一控制周期使用。
void getCommands() {
  // 读取当前接收机通道；PWM/PPM 由中断更新，SBUS 由库解析。
  // 对关键通道做一阶低通后，供下一次控制周期使用。

#if defined USE_PPM_RX || defined USE_PWM_RX
  radio_channel1_raw = getRadioPWM(1);
  radio_channel2_raw = getRadioPWM(2);
  radio_channel3_raw = getRadioPWM(3);
  radio_channel4_raw = getRadioPWM(4);
  radio_channel5_raw = getRadioPWM(5);
  radio_channel6_raw = getRadioPWM(6);

#elif defined USE_SBUS_RX
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {
    // 以下比例换算对应 Taranis-Plus 与 X4R-SB 的 SBUS 输出范围。
    float scale = 0.615;
    float bias = 895.0;
    radio_channel1_raw = sbusChannels[0] * scale + bias;
    radio_channel2_raw = sbusChannels[1] * scale + bias;
    radio_channel3_raw = sbusChannels[2] * scale + bias;
    radio_channel4_raw = sbusChannels[3] * scale + bias;
    radio_channel5_raw = sbusChannels[4] * scale + bias;
    radio_channel6_raw = sbusChannels[5] * scale + bias;
    radio_channel7_raw = sbusChannels[6] * scale + bias;
    radio_channel8_raw = sbusChannels[7] * scale + bias;
    for (unsigned i = 0; i < 4; ++i) markRadioReceived(i);
  }

#endif

  // 对关键遥控通道做一阶低通，并保存本次值。
  float b = 0.7; // 系数越小平滑越强，响应越慢。
  radio_channel1_raw = (1.0 - b) * radio_channel1_raw_prev + b * radio_channel1_raw;
  radio_channel2_raw = (1.0 - b) * radio_channel2_raw_prev + b * radio_channel2_raw;
  radio_channel3_raw = (1.0 - b) * radio_channel3_raw_prev + b * radio_channel3_raw;
  radio_channel4_raw = (1.0 - b) * radio_channel4_raw_prev + b * radio_channel4_raw;
  radio_channel1_raw_prev = radio_channel1_raw;
  radio_channel2_raw_prev = radio_channel2_raw;
  radio_channel3_raw_prev = radio_channel3_raw;
  radio_channel4_raw_prev = radio_channel4_raw;
}

// 读取 PWM/PPM 中断缓存的指定通道脉宽；通道号为 1～6，返回值单位为 μs。
unsigned long getRadioPWM(int ch_num) {
  // 返回中断采集的原始通道脉宽，单位 μs；无效通道返回 0。
  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  
  return returnPWM;
}

// PPM/PWM 中断服务函数：只记录边沿时间和脉宽，不执行控制计算。

// PPM 上升沿中断：用帧间长间隔同步，再把后续脉宽写入各通道缓存。
void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig==1) { // 仅在上升沿测量相邻脉冲间隔。
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { // 长间隔表示新的一帧开始。
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { // 第 1 通道。
      channel_1_raw = dt_ppm;
      markRadioReceived(0);
    }
  
    if (ppm_counter == 2) { // 第 2 通道。
      channel_2_raw = dt_ppm;
      markRadioReceived(1);
    }
  
    if (ppm_counter == 3) { // 第 3 通道。
      channel_3_raw = dt_ppm;
      markRadioReceived(2);
    }
  
    if (ppm_counter == 4) { // 第 4 通道。
      channel_4_raw = dt_ppm;
      markRadioReceived(3);
    }
  
    if (ppm_counter == 5) { // 第 5 通道。
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { // 第 6 通道。
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

// 通道 1 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
    markRadioReceived(0);
  }
}

// 通道 2 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
    markRadioReceived(1);
  }
}

// 通道 3 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
    markRadioReceived(2);
  }
}

// 通道 4 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh4() {
  int trigger = digitalRead(ch4Pin);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
    markRadioReceived(3);
  }
}

// 通道 5 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

// 通道 6 PWM 边沿中断：记录上升沿，并在下降沿保存脉宽（μs）。
void getCh6() {
  int trigger = digitalRead(ch6Pin);
  if(trigger == 1) {
    rising_edge_start_6 = micros();
  }
  else if(trigger == 0) {
    channel_6_raw = micros() - rising_edge_start_6;
  }
}
