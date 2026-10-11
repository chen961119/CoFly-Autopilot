#pragma once

#include <Arduino.h>
#include <type_traits>

// 每个对象管理一条命令链；A 使用 left/right，从机只使用所在侧对象。
class AircraftCommandLink {
public:
  AircraftCommandLink();
  // 对象绑定一个下游输出并独立限频；链路编号：A=1，B/C=2，D/E=3，F/G=4。
  void configure(Stream *output, Stream *input, unsigned int ownAircraftId = 1);
  // 输入为控制偏移（μs）和角度（度）；在这里统一转换为协议类型。
  bool setCommandLink(unsigned int index, int aileron1, int aileron2, int throttle, int rudder,
                      float pitch, int elevatorManual, int elevatorFeedForward, bool light,
                      bool integralEnabled, bool manualMode);
  void sendPreparedChildCommands();
  void receiveCommandData();
  void forwardReceivedChildCommands();
  bool hasReceivedData() const { return dataReceived; }

private:
  static constexpr unsigned int CHILD_COUNT = 3;
  static constexpr size_t HEADER_SIZE = 2;
  static constexpr size_t CHECKSUM_SIZE = 1;
  static constexpr uint32_t SEND_INTERVAL_US = 10000; // 100 Hz，每条链独立限频。

  // 仅包含线上字段，按声明顺序紧密排列；各机使用相同定义和小端编码。
  // 增删或修改字段时，同步更新 setCommandLink、getCommandLink 和 printReceivedData。
#pragma pack(push, 1)
  struct CommandPayload {
    uint8_t intIsValid = 0;
    uint8_t forceManual = 0;
    int16_t servoCommands[CHILD_COUNT * 4] = {};
    int16_t pitchAngles[CHILD_COUNT] = {}; // 单位 0.1 度。
    int16_t elePwmCommands[CHILD_COUNT] = {};
    int16_t eleFfPwmCommands[CHILD_COUNT] = {};
    uint8_t lightSignals[CHILD_COUNT] = {0, 0, 1};
  };
#pragma pack(pop)
  static_assert(std::is_trivially_copyable<CommandPayload>::value,
                "Command payload must support byte copying");
  static constexpr size_t FRAME_SIZE = HEADER_SIZE + sizeof(CommandPayload) + CHECKSUM_SIZE;
  CommandPayload commandPayload{};

  bool dataReceived = false;
  Stream *transmitPort = nullptr;
  Stream *receivePort = nullptr;
  unsigned int localAircraftId = 1; // 链路编号，提取从机命令时减 2 转为数组下标。
  uint8_t receiveBuffer[FRAME_SIZE] = {};
  size_t receivePosition = 0;
  uint32_t lastTransmitTimeUs = 0;
  uint32_t receivedFrameCount = 0;

  void getCommandLink();
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
    bool isLeft = false; // 左链 true（B/D/F），右链 false（C/E/G）。
    int16_t relativeAngles[CHILD_COUNT] = {}; // 0.1 度，相邻 IMU 滚转角差。
    int16_t rollRates[CHILD_COUNT] = {};      // 0.1 度/秒。
    int16_t rollAngles[CHILD_COUNT] = {};     // 0.1 度。
    int16_t pitchAngles[CHILD_COUNT] = {};    // 0.1 度。
    int16_t elevatorPwm[CHILD_COUNT] = {};    // 实际升降舵 PWM，μs。
  };
#pragma pack(pop)
  static_assert(sizeof(bool) == 1, "Sensor side flag requires one byte");
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

// 从相邻机体接收的姿态、舵机及期望状态，供控制器和日志读取。
extern float Local_pitch_des;
extern int Local_ail1_control_us, Local_ail2_control_us, Local_thro_control_us,
    Local_rudd_control_us, Local_ele_control_us, Local_ele_ff_control_us;

extern float phiB_raw, phiC_raw, phiD_raw, phiE_raw, phiF_raw, phiG_raw;
extern float thetaB_raw, thetaC_raw, thetaD_raw, thetaE_raw, thetaF_raw, thetaG_raw;
extern int B_ele_PWM, C_ele_PWM, D_ele_PWM, E_ele_PWM, F_ele_PWM, G_ele_PWM;
extern float GYRO_X_B, GYRO_X_C, GYRO_X_D, GYRO_X_E, GYRO_X_F, GYRO_X_G;

// Local downstream joint, derived from the two IMU rolls; never MT6701.
bool localWingRelativeAngle(float &angle);

void beginParentLink();
void beginChildLinks();
