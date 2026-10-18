#include "math/units.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "config/aircraft_config.h"
#include "config/flight_config.h"
#include "config/serial_config.h"
#include "sensors/sensor_processing.h"
#include <cstring>
#include <cstddef>

// 各机姿态角，单位 rad，顺序滚转/俯仰/偏航。
float att_B[3], att_C[3], att_D[3], att_E[3], att_F[3], att_G[3];
float ang_vel_B[3], ang_vel_C[3], ang_vel_D[3], ang_vel_E[3], ang_vel_F[3], ang_vel_G[3];

namespace {
int32_t commandLinkDebug = 1;
int32_t sensorLinkDebug = 0;

} // namespace

int32_t &commandLinkDebugSetting() { return commandLinkDebug; }
int32_t &sensorLinkDebugSetting() { return sensorLinkDebug; }

void AircraftCommandLink::configure(Stream *output, Stream *input,
                                    unsigned int ownAircraftLevel) {
  *this = AircraftCommandLink{};
  transmitPort = output;
  receivePort = input;
  localAircraftLevel = ownAircraftLevel;
}

bool AircraftCommandLink::setCommandLink(unsigned int index, const SlaveCommand &command) {
  if (index >= CHILD_COUNT || command.mode < MANUAL_MODE || command.mode > STABILIZE_MODE)
    return false;
  const ActuatorCommand &output = command.masterContribution;
  // 分配层已统一舵面下偏为正；通信只编码数值，不引入安装方向或额外取反。
  const float values[] = {output.aileron1, output.aileron2, output.throttle,
                          output.rudder, output.elevator};
  // 在写缓存前检查所有浮点输入；位检查不受 release 的 -ffast-math 影响。
  const auto finite = [](float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
  };
  for (float value : values) if (!finite(value)) return false;
  // 与解码的 float32 毫弧度步长一致；避免 -ffast-math 的倒数乘法
  // 让有效的最大编码回读后因一个浮点舍入位而被拒绝。
  constexpr float angleStep = 1.0f / CommandPayload::ANGLE_SCALE;
  if (!finite(command.target.pitch) || command.target.pitch < -32768.0f * angleStep ||
      command.target.pitch > 32767.0f * angleStep) return false;
  const auto encode = [](float value) -> int16_t {
    // 先限幅再缩放和取最近整数，避免大输入溢出；负值也使用对称舍入。
    if (value >= 32767.0f / CONTROL_OUTPUT_SCALE) return 32767;
    if (value <= -32768.0f / CONTROL_OUTPUT_SCALE) return -32768;
    const float scaled = value * CONTROL_OUTPUT_SCALE;
    return static_cast<int16_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
  };
  const unsigned offset = index * 5;
  for (unsigned field = 0; field < 5; ++field)
    commandPayload.controlOutputs[offset + field] = encode(values[field]);
  const float scaledPitch = command.target.pitch * CommandPayload::ANGLE_SCALE;
  commandPayload.pitchAngles[index] = static_cast<int16_t>(
      scaledPitch + (scaledPitch >= 0.0f ? 0.5f : -0.5f));
  commandPayload.controlMode = static_cast<uint8_t>(command.mode);
  return true;
}

// 左右命令链直接复制协议结构体；帧长度自动随 CommandPayload 调整。
void AircraftCommandLink::sendPreparedChildCommands() {
  if (!transmitPort)
    return;
  if (receivePort && !dataReceived)
    // 配有上级输入的对象不能在首个有效命令到达前发送默认缓存。
    return;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - lastTransmitTimeUs) < SEND_INTERVAL_US)
    return;
  lastTransmitTimeUs = now;

  uint8_t buffer[FRAME_SIZE] = {};
  buffer[0] = 0x55;
  buffer[1] = 0x60;
  std::memcpy(buffer + HEADER_SIZE, &commandPayload, sizeof(commandPayload));
  uint8_t checksum = 0;
  for (size_t index = 0; index < FRAME_SIZE - CHECKSUM_SIZE; ++index)
    checksum += buffer[index];
  buffer[FRAME_SIZE - CHECKSUM_SIZE] = checksum;
  transmitPort->write(buffer, sizeof(buffer));
}

// 完整帧校验通过后才更新命令属性；半帧和坏帧保留上一份有效命令。
void AircraftCommandLink::receiveCommandData() {
  if (!receivePort)
    return;
  while (receivePort->available()) {
    const uint8_t byte = static_cast<uint8_t>(receivePort->read());
    if (receivePosition == 0 && byte != 0x55)
      continue;
    if (receivePosition == 1 && byte != 0x60) {
      // 连续两个 0x55 时，第二个仍可作为下一帧起点。
      receivePosition = byte == 0x55 ? 1 : 0;
      continue;
    }
    receiveBuffer[receivePosition++] = byte;
    if (receivePosition != FRAME_SIZE)
      continue;
    receivePosition = 0;
    uint8_t checksum = 0;
    for (unsigned int index = 0; index < FRAME_SIZE - 1; ++index)
      checksum += receiveBuffer[index];
    if (checksum != receiveBuffer[FRAME_SIZE - 1])
      continue;

    std::memcpy(&commandPayload, receiveBuffer + HEADER_SIZE, sizeof(commandPayload));
    dataReceived = true;
    getCommandLink();
    printReceivedData(commandLinkDebugSetting());
  }
}

const SlaveCommand &AircraftCommandLink::getCommandLink() {
  // A 机没有本机槽位；无效层级或模式保留最近一次有效解码结果。
  if (localAircraftLevel < 2 || localAircraftLevel > CHILD_COUNT + 1 ||
      commandPayload.controlMode > static_cast<uint8_t>(STABILIZE_MODE))
    return parsedCommand;
  const unsigned localSlot = localAircraftLevel - 2;
  const unsigned offset = localSlot * 5;
  SlaveCommand command;
  command.mode = static_cast<FlightMode>(commandPayload.controlMode);
  command.target.pitch = commandPayload.pitchAngles[localSlot] / CommandPayload::ANGLE_SCALE;
  command.masterContribution.aileron1 = commandPayload.controlOutputs[offset] / CONTROL_OUTPUT_SCALE;
  command.masterContribution.aileron2 = commandPayload.controlOutputs[offset + 1] / CONTROL_OUTPUT_SCALE;
  command.masterContribution.throttle = commandPayload.controlOutputs[offset + 2] / CONTROL_OUTPUT_SCALE;
  command.masterContribution.rudder = commandPayload.controlOutputs[offset + 3] / CONTROL_OUTPUT_SCALE;
  command.masterContribution.elevator = commandPayload.controlOutputs[offset + 4] / CONTROL_OUTPUT_SCALE;
  parsedCommand = command;
  return parsedCommand;
}

void AircraftCommandLink::forwardReceivedChildCommands() {
  if (!receivePort || !dataReceived)
    return;
  // 保留完整协议数据和固定槽位，不清零、不移位、不重复量化角度。
  sendPreparedChildCommands();
}

void AircraftCommandLink::printReceivedData(int enabled) {
  if (enabled != 1)
    return;
  USBSerial.printf("\n=== 帧#%lu ===\n", static_cast<unsigned long>(++receivedFrameCount));
  USBSerial.printf("controlMode: %u\n", static_cast<unsigned>(commandPayload.controlMode));
  USBSerial.print("controlOutputs (int16, scale=4096): ");
  for (int16_t value : commandPayload.controlOutputs) USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\npitchAngles (deg): ");
  for (int16_t value : commandPayload.pitchAngles)
    USBSerial.printf("%.3f ", Units::toDegrees(value / CommandPayload::ANGLE_SCALE));
  USBSerial.println("\n=============");
}


void AircraftSensorLink::configure(Stream *output, Stream *input, bool isLeft) {
  *this = AircraftSensorLink{};
  transmitPort = output;
  receivePort = input;
  sensorPayload.isLeft = isLeft;
}

bool AircraftSensorLink::isDataFresh() const {
  return dataReceived && static_cast<uint32_t>(millis() - lastReceiveTimeMs) <= FRESH_INTERVAL_MS;
}

// 每个对象只保存自己串口的半帧；校验通过后复制协议数据并统一发布。
void AircraftSensorLink::receiveSensorData() {
  if (!receivePort)
    return;
  while (receivePort->available()) {
    const uint8_t byte = static_cast<uint8_t>(receivePort->read());
    if (receivePosition == 0 && byte != 0x55)
      continue;
    if (receivePosition == 1 && byte != 0x71) {
      receivePosition = byte == 0x55 ? 1 : 0;
      continue;
    }
    receiveBuffer[receivePosition++] = byte;
    if (receivePosition != FRAME_SIZE)
      continue;
    receivePosition = 0;
    uint8_t checksum = 0;
    for (size_t index = 0; index < FRAME_SIZE - CHECKSUM_SIZE; ++index)
      checksum += receiveBuffer[index];
    if (checksum != receiveBuffer[FRAME_SIZE - CHECKSUM_SIZE])
      continue;
    // 在复制为 bool 前验证其线上表示，避免读取非法 bool 值。
    if (receiveBuffer[HEADER_SIZE + offsetof(SensorPayload, isLeft)] > 1)
      continue;
    std::memcpy(&sensorPayload, receiveBuffer + HEADER_SIZE, sizeof(sensorPayload));
    getSensorLink();
    dataReceived = true;
    lastReceiveTimeMs = millis();
    printReceivedData(sensorLinkDebugSetting());
  }
}

// 仅此处把协议值转换为控制/日志使用的单位，按包内左右标志统一处理。
void AircraftSensorLink::getSensorLink() {
  const bool left = sensorPayload.isLeft;
  float *attitudeTargets[CHILD_COUNT] = {
      left ? att_B : att_C, left ? att_D : att_E, left ? att_F : att_G};
  float *rateTargets[CHILD_COUNT] = {
      left ? &ang_vel_B[0] : &ang_vel_C[0], left ? &ang_vel_D[0] : &ang_vel_E[0],
      left ? &ang_vel_F[0] : &ang_vel_G[0]};
  for (unsigned index = 0; index < CHILD_COUNT; ++index) {
    attitudeTargets[index][0] = sensorPayload.rollAngles[index] / SensorPayload::ANGLE_SCALE;
    attitudeTargets[index][1] = sensorPayload.pitchAngles[index] / SensorPayload::ANGLE_SCALE;
    attitudeTargets[index][2] = sensorPayload.yawAngles[index] / SensorPayload::ANGLE_SCALE;
    *rateTargets[index] = sensorPayload.rollRates[index] / SensorPayload::ANGLE_SCALE;
  }
}

void AircraftSensorLink::printReceivedData(int enabled) {
  if (enabled != 1)
    return;
  USBSerial.printf("\n=== Sensor frame #%lu ===\n",
                   static_cast<unsigned long>(++receivedFrameCount));
  USBSerial.printf("isLeft: %u\n", sensorPayload.isLeft ? 1u : 0u);
  USBSerial.print("relativeAngles (deg): ");
  for (int16_t value : sensorPayload.relativeAngles)
    USBSerial.printf("%.3f ", Units::toDegrees(value / SensorPayload::ANGLE_SCALE));
  USBSerial.print("\nrollRates (deg/s): ");
  for (int16_t value : sensorPayload.rollRates)
    USBSerial.printf("%.3f ", Units::toDegrees(value / SensorPayload::ANGLE_SCALE));
  USBSerial.print("\nrollAngles (deg): ");
  for (int16_t value : sensorPayload.rollAngles)
    USBSerial.printf("%.3f ", Units::toDegrees(value / SensorPayload::ANGLE_SCALE));
  USBSerial.print("\npitchAngles (deg): ");
  for (int16_t value : sensorPayload.pitchAngles)
    USBSerial.printf("%.3f ", Units::toDegrees(value / SensorPayload::ANGLE_SCALE));
  USBSerial.print("\nyawAngles (deg): ");
  for (int16_t value : sensorPayload.yawAngles)
    USBSerial.printf("%.3f ", Units::toDegrees(value / SensorPayload::ANGLE_SCALE));
  USBSerial.println("\n=============");
}

// 汇总本机和下游状态；固定槽位不移位，转换为协议类型后再复制发送。
void AircraftSensorLink::setSensorLink() {
  const float localRollRate = ang_vel[0];
  const bool left = aircraftIsLeft();
  sensorPayload.isLeft = left;
  const unsigned int localSlot = static_cast<unsigned int>((aircraftId() - 2) / 2);
  const float *downstreamAttitudes[CHILD_COUNT] = {
      left ? att_B : att_C, left ? att_D : att_E, left ? att_F : att_G};
  const float downstreamRates[CHILD_COUNT] = {
      left ? ang_vel_B[0] : ang_vel_C[0], left ? ang_vel_D[0] : ang_vel_E[0],
      left ? ang_vel_F[0] : ang_vel_G[0]};
  const auto encode = [](float value) -> int16_t {
    // 状态不能拒绝整帧：非有限测量置零，超量程饱和，防止 int16 转换溢出。
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    if ((bits & 0x7f800000u) == 0x7f800000u) return 0;
    if (value >= 32767.0f / SensorPayload::ANGLE_SCALE) return 32767;
    if (value <= -32768.0f / SensorPayload::ANGLE_SCALE) return -32768;
    const float scaled = value * SensorPayload::ANGLE_SCALE;
    return static_cast<int16_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
  };
  for (unsigned int index = 0; index < CHILD_COUNT; ++index) {
    float relativeAngle = 0;
    float roll = 0;
    float pitch = 0;
    float rate = 0;
    float yaw = 0;
    if (index == localSlot) {
      roll = att[0];
      pitch = att[1];
      rate = localRollRate;
      yaw = att[2];
    } else if (index > localSlot) {
      roll = downstreamAttitudes[index][0];
      pitch = downstreamAttitudes[index][1];
      rate = downstreamRates[index];
      yaw = downstreamAttitudes[index][2];
      const float previousRoll = index - 1 == localSlot ? att[0] : downstreamAttitudes[index - 1][0];
      // 保留相邻 IMU 原始滚转角差，不用量化后的协议角度计算；MT6701 仅记录。
      relativeAngle = roll - previousRoll;
    }
    sensorPayload.relativeAngles[index] = encode(relativeAngle);
    sensorPayload.rollRates[index] = encode(rate);
    sensorPayload.rollAngles[index] = encode(roll);
    sensorPayload.pitchAngles[index] = encode(pitch);
    sensorPayload.yawAngles[index] = encode(yaw);
  }
}

void AircraftSensorLink::forwardReceivedChildSensors() {
  if (!receivePort || !transmitPort)
    return;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - lastTransmitTimeUs) < SEND_INTERVAL_US)
    return;
  lastTransmitTimeUs = now;
  setSensorLink();

  uint8_t buffer[FRAME_SIZE] = {};
  buffer[0] = 0x55;
  buffer[1] = 0x71;
  std::memcpy(buffer + HEADER_SIZE, &sensorPayload, sizeof(sensorPayload));
  uint8_t checksum = 0;
  for (size_t index = 0; index < FRAME_SIZE - CHECKSUM_SIZE; ++index)
    checksum += buffer[index];
  buffer[FRAME_SIZE - CHECKSUM_SIZE] = checksum;
  transmitPort->write(buffer, sizeof(buffer));
}
