#include "config/aircraft_config.h"
#include "config/flight_config.h"
#include "config/serial_config.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "sensors/sensor_processing.h"

namespace CommandLinks {
AircraftCommandLink *left = nullptr;
AircraftCommandLink *right = nullptr;
}

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

void receiveCommandData() {
  if (aircraftIsSingle() || !aircraftConfigurationValid() || aircraftId() == 1)
    return;
  AircraftCommandLink *link = aircraftIsLeft() ? CommandLinks::left : CommandLinks::right;
  if (link)
    link->receiveCommandData();
}

void forwardReceivedChildCommands() {
  if (aircraftIsSingle() || !aircraftConfigurationValid() || aircraftId() == 1)
    return;
  AircraftCommandLink *link = aircraftIsLeft() ? CommandLinks::left : CommandLinks::right;
  if (link)
    link->forwardReceivedChildCommands();
}

bool hasReceivedCommandData() {
  if (aircraftIsSingle() || !aircraftConfigurationValid() || aircraftId() == 1)
    return false;
  const AircraftCommandLink *link = aircraftIsLeft() ? CommandLinks::left : CommandLinks::right;
  return link && link->hasReceivedData();
}

namespace SensorLinks {
AircraftSensorLink *left = nullptr;
AircraftSensorLink *right = nullptr;
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

void receiveSensorData() {
  // 保留原 A 节点先右后左的接收顺序。
  if (SensorLinks::right)
    SensorLinks::right->receiveSensorData();
  if (SensorLinks::left)
    SensorLinks::left->receiveSensorData();
}

void forwardReceivedChildSensors() {
  if (aircraftIsSingle() || !aircraftConfigurationValid() || aircraftId() == 1)
    return;
  AircraftSensorLink *link = aircraftIsLeft() ? SensorLinks::left : SensorLinks::right;
  if (link)
    link->forwardReceivedChildSensors();
}

bool isSensorDataFresh(bool left) {
  if (aircraftIsSingle() || !aircraftConfigurationValid())
    return false;
  const AircraftSensorLink *link = left ? SensorLinks::left : SensorLinks::right;
  return link && link->isDataFresh();
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
  RightLinkSerial.begin(921600);
  LeftLinkSerial.begin(921600);
}
