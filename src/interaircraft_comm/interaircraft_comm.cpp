#include "interaircraft_comm/interaircraft_comm.h"
#include "config/aircraft_config.h"
#include "config/flight_config.h"
#include "config/serial_config.h"
#include "control/control_state.h"
#include "sensors/sensor_processing.h"
#include <cstring>
#include <cstddef>

float Local_pitch_des;
// 未收到上级命令前保持 1000 μs 的逻辑低油门。
int Local_thro_control_us = 1000 - PWM_CENTER_US;
int Local_ail1_control_us, Local_ail2_control_us, Local_rudd_control_us, Local_ele_control_us,
    Local_ele_ff_control_us;
float phiB_raw, phiC_raw, phiD_raw, phiE_raw, phiF_raw, phiG_raw;
float thetaB_raw, thetaC_raw, thetaD_raw, thetaE_raw, thetaF_raw, thetaG_raw;
int B_ele_PWM, C_ele_PWM, D_ele_PWM, E_ele_PWM, F_ele_PWM, G_ele_PWM;
float GYRO_X_B, GYRO_X_C, GYRO_X_D, GYRO_X_E, GYRO_X_F, GYRO_X_G;

namespace {
int32_t commandLinkDebug = 1;
int32_t sensorLinkDebug = 0;

} // namespace

int32_t &commandLinkDebugSetting() { return commandLinkDebug; }
int32_t &sensorLinkDebugSetting() { return sensorLinkDebug; }

AircraftCommandLink::AircraftCommandLink() {
  // 尚未准备命令时，油门偏移保持逻辑 PWM 1000 μs。
  for (unsigned int index = 0; index < CHILD_COUNT; ++index)
    commandPayload.servoCommands[index * 4 + 2] = 1000 - PWM_CENTER_US;
}

void AircraftCommandLink::configure(Stream *output, Stream *input,
                                    unsigned int ownAircraftId) {
  *this = AircraftCommandLink{};
  transmitPort = output;
  receivePort = input;
  localAircraftId = ownAircraftId;
}

bool AircraftCommandLink::setCommandLink(unsigned int index, int aileron1, int aileron2,
                                         int throttle, int rudder, float pitch,
                                         int elevatorManual, int elevatorFeedForward, bool light,
                                         bool integralEnabled, bool manualMode) {
  if (index >= CHILD_COUNT)
    return false;
  const unsigned int offset = index * 4;
  commandPayload.servoCommands[offset] = static_cast<int16_t>(aileron1);
  commandPayload.servoCommands[offset + 1] = static_cast<int16_t>(aileron2);
  commandPayload.servoCommands[offset + 2] = static_cast<int16_t>(throttle);
  commandPayload.servoCommands[offset + 3] = static_cast<int16_t>(rudder);
  commandPayload.pitchAngles[index] = static_cast<int16_t>(pitch * 10.0f);
  commandPayload.elePwmCommands[index] = static_cast<int16_t>(elevatorManual);
  commandPayload.eleFfPwmCommands[index] = static_cast<int16_t>(elevatorFeedForward);
  commandPayload.lightSignals[index] = light ? 1 : 0;
  commandPayload.intIsValid = integralEnabled ? 1 : 0;
  commandPayload.forceManual = manualMode ? 1 : 0;
  return true;
}

// 左右命令链直接复制协议结构体；帧长度自动随 CommandPayload 调整。
void AircraftCommandLink::sendPreparedChildCommands() {
  if (aircraftIsSingle() || !aircraftConfigurationValid() || !transmitPort)
    return;
  // 配有上级输入的对象不能在首个有效命令到达前发送默认缓存。
  if (receivePort && !dataReceived)
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
  if (aircraftIsSingle() || !receivePort || localAircraftId < 2 ||
      localAircraftId > CHILD_COUNT + 1)
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

void AircraftCommandLink::getCommandLink() {
  const unsigned int localSlot = localAircraftId - 2;
  const unsigned int offset = localSlot * 4;
  Local_ail1_control_us = commandPayload.servoCommands[offset];
  Local_ail2_control_us = commandPayload.servoCommands[offset + 1];
  Local_thro_control_us = commandPayload.servoCommands[offset + 2];
  Local_rudd_control_us = commandPayload.servoCommands[offset + 3];
  Local_pitch_des = commandPayload.pitchAngles[localSlot] / 10.0f;
  Local_ele_control_us = commandPayload.elePwmCommands[localSlot];
  Local_ele_ff_control_us = commandPayload.eleFfPwmCommands[localSlot];
  int_is_valid = commandPayload.intIsValid == 1;
  force_manual = commandPayload.forceManual == 1;
}

void AircraftCommandLink::forwardReceivedChildCommands() {
  if (!receivePort || !dataReceived || localAircraftId < 2 ||
      localAircraftId > CHILD_COUNT + 1)
    return;
  // 保留完整协议数据和固定槽位，不清零、不移位、不重复量化角度。
  sendPreparedChildCommands();
}

void AircraftCommandLink::printReceivedData(int enabled) {
  if (enabled != 1)
    return;
  USBSerial.printf("\n=== 帧#%lu ===\n", static_cast<unsigned long>(++receivedFrameCount));
  USBSerial.printf("intIsValid: %u forceManual: %u\n",
                   static_cast<unsigned int>(commandPayload.intIsValid),
                   static_cast<unsigned int>(commandPayload.forceManual));
  USBSerial.print("servoCommands: ");
  for (int16_t value : commandPayload.servoCommands)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\npitchAngles (0.1 deg): ");
  for (int16_t value : commandPayload.pitchAngles)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\nelePwmCommands: ");
  for (int16_t value : commandPayload.elePwmCommands)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\neleFfPwmCommands: ");
  for (int16_t value : commandPayload.eleFfPwmCommands)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\nlightSignals: ");
  for (uint8_t value : commandPayload.lightSignals)
    USBSerial.printf("%u ", static_cast<unsigned int>(value));
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
  if (aircraftIsSingle() || !aircraftConfigurationValid() || !receivePort)
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
  float *rollTargets[CHILD_COUNT] = {
      left ? &phiB_raw : &phiC_raw, left ? &phiD_raw : &phiE_raw, left ? &phiF_raw : &phiG_raw};
  float *pitchTargets[CHILD_COUNT] = {
      left ? &thetaB_raw : &thetaC_raw, left ? &thetaD_raw : &thetaE_raw,
      left ? &thetaF_raw : &thetaG_raw};
  float *rateTargets[CHILD_COUNT] = {
      left ? &GYRO_X_B : &GYRO_X_C, left ? &GYRO_X_D : &GYRO_X_E, left ? &GYRO_X_F : &GYRO_X_G};
  int *elevatorTargets[CHILD_COUNT] = {
      left ? &B_ele_PWM : &C_ele_PWM, left ? &D_ele_PWM : &E_ele_PWM, left ? &F_ele_PWM : &G_ele_PWM};
  for (unsigned int index = 0; index < CHILD_COUNT; ++index) {
    *rollTargets[index] = sensorPayload.rollAngles[index] / 10.0f;
    *pitchTargets[index] = sensorPayload.pitchAngles[index] / 10.0f;
    *rateTargets[index] = sensorPayload.rollRates[index] / 10.0f;
    *elevatorTargets[index] = sensorPayload.elevatorPwm[index];
  }
  if (left) {
    phibd = sensorPayload.relativeAngles[1] / 10.0f;
    phidf = sensorPayload.relativeAngles[2] / 10.0f;
  } else {
    phiac = sensorPayload.relativeAngles[0] / 10.0f;
    phice = sensorPayload.relativeAngles[1] / 10.0f;
    phieg = sensorPayload.relativeAngles[2] / 10.0f;
  }
}

void AircraftSensorLink::printReceivedData(int enabled) {
  if (enabled != 1)
    return;
  USBSerial.printf("\n=== Sensor frame #%lu ===\n",
                   static_cast<unsigned long>(++receivedFrameCount));
  USBSerial.printf("isLeft: %u\n", sensorPayload.isLeft ? 1u : 0u);
  USBSerial.print("relativeAngles (0.1 deg): ");
  for (int16_t value : sensorPayload.relativeAngles)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\nrollRates (0.1 deg/s): ");
  for (int16_t value : sensorPayload.rollRates)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\nrollAngles (0.1 deg): ");
  for (int16_t value : sensorPayload.rollAngles)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\npitchAngles (0.1 deg): ");
  for (int16_t value : sensorPayload.pitchAngles)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.print("\nelevatorPwm (us): ");
  for (int16_t value : sensorPayload.elevatorPwm)
    USBSerial.printf("%d ", static_cast<int>(value));
  USBSerial.println("\n=============");
}

// 汇总本机和下游状态；固定槽位不移位，转换为协议类型后再复制发送。
void AircraftSensorLink::setSensorLink() {
  float localRollRate = 0;
#if defined USE_MPU6050_I2C
  localRollRate = -GyroX_6050;
#endif
#if defined USE_MPU9250_SPI
  localRollRate = -GyroX_9250;
#endif
#if defined EXTIMU
  localRollRate = Gyro_X_EXT;
#endif
  const bool left = aircraftIsLeft();
  sensorPayload.isLeft = left;
  const unsigned int localSlot = static_cast<unsigned int>((aircraftId() - 2) / 2);
  const float downstreamRolls[CHILD_COUNT] = {
      left ? phiB_raw : phiC_raw, left ? phiD_raw : phiE_raw, left ? phiF_raw : phiG_raw};
  const float downstreamPitches[CHILD_COUNT] = {
      left ? thetaB_raw : thetaC_raw, left ? thetaD_raw : thetaE_raw, left ? thetaF_raw : thetaG_raw};
  const float downstreamRates[CHILD_COUNT] = {
      left ? GYRO_X_B : GYRO_X_C, left ? GYRO_X_D : GYRO_X_E, left ? GYRO_X_F : GYRO_X_G};
  const int downstreamElevators[CHILD_COUNT] = {
      left ? B_ele_PWM : C_ele_PWM, left ? D_ele_PWM : E_ele_PWM, left ? F_ele_PWM : G_ele_PWM};
  for (unsigned int index = 0; index < CHILD_COUNT; ++index) {
    float relativeAngle = 0;
    float roll = 0;
    float pitch = 0;
    float rate = 0;
    int elevator = 0;
    if (index == localSlot) {
      roll = roll_IMU;
      pitch = pitch_IMU;
      rate = localRollRate;
      elevator = static_cast<int>(ele_PWM);
    } else if (index > localSlot) {
      roll = downstreamRolls[index];
      pitch = downstreamPitches[index];
      rate = downstreamRates[index];
      elevator = downstreamElevators[index];
      const float previousRoll = index - 1 == localSlot ? roll_IMU : downstreamRolls[index - 1];
      // 保留相邻 IMU 原始滚转角差，不用量化后的协议角度计算；MT6701 仅记录。
      relativeAngle = roll - previousRoll;
    }
    sensorPayload.relativeAngles[index] = static_cast<int16_t>(relativeAngle * 10.0f);
    sensorPayload.rollRates[index] = static_cast<int16_t>(rate * 10.0f);
    sensorPayload.rollAngles[index] = static_cast<int16_t>(roll * 10.0f);
    sensorPayload.pitchAngles[index] = static_cast<int16_t>(pitch * 10.0f);
    sensorPayload.elevatorPwm[index] = static_cast<int16_t>(elevator);
  }
}

void AircraftSensorLink::forwardReceivedChildSensors() {
  if (aircraftIsSingle() || !aircraftConfigurationValid() || aircraftId() == 1 || !transmitPort)
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
