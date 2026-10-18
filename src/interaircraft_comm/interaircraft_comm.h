#pragma once

#include <Arduino.h>
#include <type_traits>
#include "control/control_types.h"

// 每个对象管理一条命令链；A 使用 left/right，从机只使用所在侧对象。
class AircraftCommandLink {
public:
  AircraftCommandLink() = default;
  // 对象绑定一个下游输出并独立限频；链路层级：A=1，B/C=2，D/E=3，F/G=4。
  void configure(Stream *output, Stream *input, unsigned int ownAircraftLevel = 1);
  // 输入为 PWM 缩放前的逻辑控制输出；目标角为 rad，线上步长为 0.001 rad。
  bool setCommandLink(unsigned int index, const SlaveCommand &command);
  const SlaveCommand &getCommandLink();
  void sendPreparedChildCommands();
  void receiveCommandData();
  void forwardReceivedChildCommands();
  bool hasReceivedData() const { return dataReceived; }

private:
  static constexpr unsigned int CHILD_COUNT = 3;
  static constexpr size_t HEADER_SIZE = 2;
  static constexpr size_t CHECKSUM_SIZE = 1;
  static constexpr uint32_t SEND_INTERVAL_US = 10000; // 100 Hz，每条链独立限频。

  static constexpr float CONTROL_OUTPUT_SCALE = 4096.0f; // int16 定点，分辨率 1/4096，范围约 ±8。

  // 仅包含线上字段，按声明顺序紧密排列；各机使用相同定义和小端编码。
  // 增删或修改字段时，同步更新 setCommandLink、getCommandLink 和 printReceivedData。
#pragma pack(push, 1)
  struct CommandPayload {
    static constexpr float ANGLE_SCALE = 1000.0f;
    uint8_t controlMode = MANUAL_MODE;
    // 每槽位依次为副翼1、副翼2、油门、方向舵、升降舵，int16 定点逻辑量（真实值 ×4096）；副翼和升降舵均下偏为正。
    // 原构造函数的低油门默认值移到载荷初始化，不在构造函数中处理协议。
    int16_t controlOutputs[CHILD_COUNT * 5] = {
        0, 0, -2048, 0, 0, 0, 0, -2048, 0, 0, 0, 0, -2048, 0, 0};
    int16_t pitchAngles[CHILD_COUNT] = {}; // 0.001 rad；范围 [-32.768, 32.767] rad。
  };
#pragma pack(pop)
  static_assert(std::is_trivially_copyable<CommandPayload>::value,
                "Command payload must support byte copying");
  static constexpr size_t FRAME_SIZE = HEADER_SIZE + sizeof(CommandPayload) + CHECKSUM_SIZE;
  CommandPayload commandPayload{};
  // 解码结果与链路对象同寿命，getCommandLink() 返回其稳定引用。
  SlaveCommand parsedCommand{MANUAL_MODE, {}, {0, 0, 0, -0.5f, 0}};
  static_assert(sizeof(int16_t) == 2 && sizeof(CommandPayload) == 37, "Command wire layout changed");

  bool dataReceived = false;
  Stream *transmitPort = nullptr;
  Stream *receivePort = nullptr;
  unsigned int localAircraftLevel = 1; // 链路层级：A=1，B/C=2，D/E=3，F/G=4；减 2 得到链内槽位。
  uint8_t receiveBuffer[FRAME_SIZE] = {};
  size_t receivePosition = 0;
  uint32_t lastTransmitTimeUs = 0;
  uint32_t receivedFrameCount = 0;

  void printReceivedData(int enabled);
};

namespace CommandLinks {
extern AircraftCommandLink *left;
extern AircraftCommandLink *right;
}
// SD 参数绑定到此设置；1 打印有效命令帧，0 关闭，修改立即生效。
int32_t &commandLinkDebugSetting();
// 身份参数加载后绑定串口；单机保持命令链禁用。
void initializeCommandLinks();
void receiveCommandData();
void forwardReceivedChildCommands();
bool hasReceivedCommandData();

// 每个对象管理一条状态链；A 使用 left/right，从机只使用所在侧对象。
class AircraftSensorLink {
public:
  void configure(Stream *output, Stream *input, bool isLeft);
  void receiveSensorData();
  void forwardReceivedChildSensors();
  bool hasReceivedData() const { return dataReceived; }
  bool isDataFresh() const;

private:
  static constexpr unsigned int CHILD_COUNT = 3;
  static constexpr size_t HEADER_SIZE = 2;
  static constexpr size_t CHECKSUM_SIZE = 1;
  static constexpr uint32_t SEND_INTERVAL_US = 5000; // 上行状态 200 Hz。
  static constexpr uint32_t FRESH_INTERVAL_MS = 1000;

  // 修改协议时同步更新 setSensorLink、getSensorLink 和 printReceivedData。
#pragma pack(push, 1)
  struct SensorPayload {
    static constexpr float ANGLE_SCALE = 1000.0f;
    bool isLeft = false; // 左链 true（B/D/F），右链 false（C/E/G）。
    int16_t relativeAngles[CHILD_COUNT] = {}; // 0.001 rad，相邻 IMU 滚转角差。
    int16_t rollRates[CHILD_COUNT] = {};      // 0.001 rad/s。
    int16_t rollAngles[CHILD_COUNT] = {};     // 0.001 rad。
    int16_t pitchAngles[CHILD_COUNT] = {};    // 0.001 rad。
    int16_t yawAngles[CHILD_COUNT] = {};      // 0.001 rad。
  };
#pragma pack(pop)
  static_assert(sizeof(bool) == 1 && sizeof(SensorPayload) == 31, "Sensor wire layout changed");
  static_assert(std::is_trivially_copyable<SensorPayload>::value,
                "Sensor payload must support byte copying");
  static constexpr size_t FRAME_SIZE = HEADER_SIZE + sizeof(SensorPayload) + CHECKSUM_SIZE;
  SensorPayload sensorPayload{};
  Stream *transmitPort = nullptr;
  Stream *receivePort = nullptr;
  uint8_t receiveBuffer[FRAME_SIZE] = {};
  size_t receivePosition = 0;
  bool dataReceived = false;
  uint32_t lastReceiveTimeMs = 0;
  uint32_t lastTransmitTimeUs = 0;
  uint32_t receivedFrameCount = 0;

  void getSensorLink();
  void setSensorLink();
  void printReceivedData(int enabled);
};

namespace SensorLinks {
extern AircraftSensorLink *left;
extern AircraftSensorLink *right;
}
int32_t &sensorLinkDebugSetting();
void initializeSensorLinks();
// 统一编排 A 的两条链或从机的一条链，按接收链路查询有效状态。
void receiveSensorData();
void forwardReceivedChildSensors();
bool isSensorDataFresh(bool left);

// 从相邻机体接收的三轴姿态，供控制器、显示和日志读取。
// 各机姿态角，单位 rad，顺序滚转/俯仰/偏航。
extern float att_B[3], att_C[3], att_D[3], att_E[3], att_F[3], att_G[3];
// 三轴顺序 X/Y/Z，单位 rad/s；当前协议只更新 [0]，Y/Z 未传输且保持 0。
extern float ang_vel_B[3], ang_vel_C[3], ang_vel_D[3], ang_vel_E[3], ang_vel_F[3], ang_vel_G[3];

// Local downstream joint, derived from the two IMU rolls; never MT6701.
bool localWingRelativeAngle(float &angle);

void beginParentLink();
void beginChildLinks();
