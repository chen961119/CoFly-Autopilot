#include "config/aircraft_config.h"
#include "config/flight_config.h"
#include "config/serial_config.h"
#include "control/control_state.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "sensors/sensor_processing.h"

// 用于地面站和屏幕显示
bool localWingRelativeAngle(float &angle) {
  if (aircraftIsSingle()) {
    angle = 0;
    return false;
  }
  if ((aircraftId() == 1)) {
    angle = phiB_raw - roll_IMU;
    return wingStateFresh(true);
  } else if ((aircraftId() == 2)) {
    angle = phiD_raw - roll_IMU;
    return wingStateFresh(true);
  } else if ((aircraftId() == 4)) {
    angle = phiF_raw - roll_IMU;
    return wingStateFresh(true);
  } else if ((aircraftId() == 3)) {
    angle = phiE_raw - roll_IMU;
    return wingStateFresh(false);
  } else if ((aircraftId() == 5)) {
    angle = phiG_raw - roll_IMU;
    return wingStateFresh(false);
  } else {
    angle = 0;
    return false;
  }
}
// 以 921600 波特率启动面向上一级机体的 ParentSerial。
void beginParentLink() {
  ParentSerial.begin(921600);
}
// 以 921600 波特率启动左右子机所用的 LeftChildSerial 与 RightChildSerial。
void beginChildLinks() {
  RightChildSerial.begin(921600);
  LeftChildSerial.begin(921600);
}
