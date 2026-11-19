#include "config/aircraft_config.h"
#include "config/flight_config.h"
#include "config/serial_config.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "sensors/sensor_processing.h"
#include "radio_comm/radio_comm.h"
#include "control/command_generation.h"
#include "control/control_modes.h"
#include <FreeRTOS.h>
#include <task.h>
#include <cstring>

namespace {
struct LinkReception {
  AircraftPeerState peers;
  SlaveCommand parent;
  uint32_t receivedAtMs[2]{};
  bool received[2]{};
};
struct LinkTransmission {
  LocalLinkState local;
  SlaveCommand children[CONTROL_AIRCRAFT_SLOTS]{};
  int commandDebug = 0;
  int sensorDebug = 0;
  bool enabled = false;
  uint32_t generation = 0;
};
LinkReception publishedReception, controlReception;
LinkTransmission publishedTransmission;
namespace CommandLinks {
AircraftCommandLink *left = nullptr;
AircraftCommandLink *right = nullptr;
}
namespace SensorLinks {
AircraftSensorLink *left = nullptr;
AircraftSensorLink *right = nullptr;
}
} // namespace

// 身份参数加载后，仅构造并配置该机使用的命令链。
void initializeCommandLinks() {
  CommandLinks::left = nullptr;
  CommandLinks::right = nullptr;
  if (aircraftIsSingle() || !aircraftConfigurationValid())
    return;
  Stream *parentInput = aircraftId() == 1 ? nullptr :
      (aircraftIsLeft() ? static_cast<Stream *>(&RightLinkSerial) : &LeftLinkSerial);
  const unsigned int localAircraftLevel = aircraftId() == 1 ? 1 :
      static_cast<unsigned int>((aircraftId() - 2) / 2 + 2);
  if (aircraftId() == 1 || aircraftIsLeft()) {
    static AircraftCommandLink left;
    left.configure(&LeftLinkSerial, parentInput, localAircraftLevel);
    CommandLinks::left = &left;
  }
  if (aircraftId() == 1 || aircraftIsRight()) {
    static AircraftCommandLink right;
    right.configure(&RightLinkSerial, parentInput, localAircraftLevel);
    CommandLinks::right = &right;
  }
}

void initializeSensorLinks() {
  SensorLinks::left = nullptr;
  SensorLinks::right = nullptr;
  if (aircraftIsSingle() || !aircraftConfigurationValid())
    return;
  Stream *parentOutput = aircraftId() == 1 ? nullptr :
      (aircraftIsLeft() ? static_cast<Stream *>(&RightLinkSerial) : &LeftLinkSerial);
  if (aircraftId() == 1 || aircraftIsLeft()) {
    // A 和左侧从机使用 left；静态存储避免堆分配。
    static AircraftSensorLink left;
    left.configure(parentOutput, &LeftLinkSerial, true);
    SensorLinks::left = &left;
  }
  if (aircraftId() == 1 || aircraftIsRight()) {
    // A 和右侧从机使用 right，未走此分支不会构造该对象。
    static AircraftSensorLink right;
    right.configure(parentOutput, &RightLinkSerial, false);
    SensorLinks::right = &right;
  }
}

void latchInteraircraftInputs() {
  taskENTER_CRITICAL();
  controlReception = publishedReception;
  taskEXIT_CRITICAL();
  float *const attitudes[] = {att_B, att_C, att_D, att_E, att_F, att_G};
  float *const rates[] = {ang_vel_B, ang_vel_C, ang_vel_D, ang_vel_E, ang_vel_F, ang_vel_G};
  for (unsigned i = 0; i < CONTROL_AIRCRAFT_SLOTS; ++i) {
    std::memcpy(attitudes[i], controlReception.peers.attitude[i], sizeof(att_B));
    rates[i][0] = controlReception.peers.rollRate[i];
  }
  slaveCommand = controlReception.parent;
}

bool isSensorDataFresh(bool left) {
  const unsigned side = left ? 0 : 1;
  return controlReception.received[side] &&
         millis() - controlReception.receivedAtMs[side] <= AircraftSensorLink::FRESH_INTERVAL_MS;
}

void publishInteraircraftOutputs(bool enabled) {
  LinkTransmission next{};
  next.enabled = enabled;
  next.commandDebug = commandLinkDebugSetting();
  next.sensorDebug = sensorLinkDebugSetting();
  if (enabled && !aircraftIsSingle()) {
    if (aircraftId() == 1) std::memcpy(next.children, slaveCommands, sizeof(next.children));
    else {
      std::memcpy(next.local.attitude, att, sizeof(next.local.attitude));
      next.local.rollRate = ang_vel[0];
    }
  }
  taskENTER_CRITICAL();
  next.generation = publishedTransmission.generation + 1;
  publishedTransmission = next;
  taskEXIT_CRITICAL();
}

void pollFlightLinks() {
  pollRadioInput();
  if (aircraftIsSingle() || !aircraftConfigurationValid()) return;

  LinkTransmission tx;
  taskENTER_CRITICAL();
  tx = publishedTransmission;
  taskEXIT_CRITICAL();
  if (!tx.enabled) return;

  // Only this task touches the parser state and the two physical UARTs.
  static LinkReception rx;
  AircraftSensorLink *const sensors[] = {SensorLinks::left, SensorLinks::right};
  AircraftCommandLink *const commands[] = {CommandLinks::left, CommandLinks::right};
  for (int side = 1; side >= 0; --side) {
    if (!sensors[side]) continue;
    sensors[side]->receiveSensorData(rx.peers, tx.sensorDebug);
    rx.received[side] = sensors[side]->hasReceivedData();
    rx.receivedAtMs[side] = sensors[side]->lastReceivedAtMs();
  }
  const unsigned ownSide = aircraftIsLeft() ? 0 : 1;
  if (aircraftId() != 1 && commands[ownSide]) {
    commands[ownSide]->receiveCommandData(tx.commandDebug);
    rx.parent = commands[ownSide]->getCommandLink();
  }
  taskENTER_CRITICAL();
  publishedReception = rx;
  tx = publishedTransmission;
  taskEXIT_CRITICAL();

  // Retry on the 1 ms link tick until due and UART space is available. Each
  // UART sends at most once per completed control cycle, even if control stalls.
  if (!tx.enabled) return;
  if (aircraftId() == 1) {
    for (unsigned side = 0; side < 2; ++side) {
      if (!commands[side]) continue;
      for (unsigned slot = 0; slot < 3; ++slot)
        commands[side]->setCommandLink(slot, tx.children[slot * 2 + side]);
      commands[side]->sendPreparedChildCommands(tx.generation);
    }
  } else {
    if (sensors[ownSide]) sensors[ownSide]->forwardReceivedChildSensors(tx.local, rx.peers, tx.generation);
    if (commands[ownSide]) commands[ownSide]->forwardReceivedChildCommands(tx.generation);
  }
}

// 用于地面站和屏幕显示
bool localWingRelativeAngle(float &angle) {
  if (aircraftIsSingle()) {
    angle = 0;
    return false;
  }
  if ((aircraftId() == 1)) {
    angle = att_B[0] - att[0];
    return isSensorDataFresh(true);
  } else if ((aircraftId() == 2)) {
    angle = att_D[0] - att[0];
    return isSensorDataFresh(true);
  } else if ((aircraftId() == 4)) {
    angle = att_F[0] - att[0];
    return isSensorDataFresh(true);
  } else if ((aircraftId() == 3)) {
    angle = att_E[0] - att[0];
    return isSensorDataFresh(false);
  } else if ((aircraftId() == 5)) {
    angle = att_G[0] - att[0];
    return isSensorDataFresh(false);
  } else {
    angle = 0;
    return false;
  }
}
// 固定初始化两个物理端口；翼尖也保留下级端口绑定，沿用原收发判断。
void beginInteraircraftLinks() {
  // Teensyduino's default TX ring has 39 usable bytes; a command frame needs 40.
  static uint8_t leftTx[64], rightTx[64], leftRx[128], rightRx[128];
  LeftLinkSerial.addMemoryForWrite(leftTx, sizeof(leftTx));
  RightLinkSerial.addMemoryForWrite(rightTx, sizeof(rightTx));
  LeftLinkSerial.addMemoryForRead(leftRx, sizeof(leftRx));
  RightLinkSerial.addMemoryForRead(rightRx, sizeof(rightRx));
  RightLinkSerial.begin(921600);
  LeftLinkSerial.begin(921600);
}
