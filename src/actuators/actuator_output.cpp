#include "parameters/control_parameters.h"
#include "config/aircraft_config.h"
#include "config/serial_config.h"
#include "actuators/actuator_output.h"
#include "control/control_modes.h"
#include "control/command_generation.h"
#include "diagnostics/debug_print.h"
#include "config/flight_config.h"
#include "system/flight_lock.h"
#include "actuators/actuator_pwm_limits.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include <Servo.h>

// 最终物理 PWM（μs），顺序为副翼1、副翼2、升降舵、油门、方向舵。
float ActuatorOutput[5] = {};

// 预留的电调输出引脚；实际舵面由下方 Servo 对象输出 PWM。
// const int m1Pin = 37;
// const int m2Pin = 37;
// const int m3Pin = 38;
// const int m4Pin = 39;
// const int m5Pin = 40;
// const int m6Pin = 41;

const int servo1Pin = 2; // 左副翼
const int servo2Pin = 3; // 右副翼
const int servo3Pin = 4; // 升降
const int servo4Pin = 5; // 油门
const int servo5Pin = 6; // 方向
// const int servo6Pin = 9;
// const int servo7Pin = 9;

// 舵机或电调 PWM 输出对象。
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
// Servo servo6;
// Servo servo7;

// 在指定硬件版本上电阶段将 5 号引脚写为低，并延时等待执行器供电稳定。
// void prepareActuatorPower()
// {
//   // 指定硬件版本在绑定舵机前，先将 5 号引脚写为低并等待 100 ms。
// #if defined expensive
//   digitalWrite(5, LOW);
//   delay(100);
// #endif
// }

// 绑定各舵机和电调引脚，配置独立于各通道软件限幅的驱动脉宽范围。
void attachActuators()
{
  // 指定各路执行器的有效脉宽范围，单位 μs。
  servo1.attach(servo1Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US); // 左副翼
  servo2.attach(servo2Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US); // 右副翼
  servo3.attach(servo3Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US); // 升降
  servo4.attach(servo4Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US); // 油门
  servo5.attach(servo5Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US); // 方向
  // servo6.attach(servo6Pin, 1000, 2000);
  // servo7.attach(servo7Pin, 1000, 2000);

  commandSafeActuatorPositions();
}

// 上电和锁定时的固定物理位置，不应用 rev 或控制量。
void commandSafeActuatorPositions()
{
  ActuatorOutput[0] = static_cast<int>(PWM_CENTER_US + pwm_channel1_trim);
  servo1.writeMicroseconds(ActuatorOutput[0]);
  ActuatorOutput[1] = static_cast<int>(PWM_CENTER_US + pwm_channel2_trim);
  servo2.writeMicroseconds(ActuatorOutput[1]);
  ActuatorOutput[2] = static_cast<int>(PWM_CENTER_US + pwm_channel3_trim);
  servo3.writeMicroseconds(ActuatorOutput[2]);
  ActuatorOutput[3] = static_cast<int>(PWM_CENTER_US + pwm_channel4_trim);
  servo4.writeMicroseconds(ActuatorOutput[3]);
  ActuatorOutput[4] = static_cast<int>(PWM_CENTER_US + pwm_channel5_trim);
  servo5.writeMicroseconds(ActuatorOutput[4]);
}

// 只将融合后的归一化指令转换到本机物理 PWM；分配与交联已完成。
void convertControlCommandsToPWM(const ControlCycleOutput &cycle)
{
  const ActuatorCommand &local = cycle.local;
  ActuatorOutput[0] = PWM_CENTER_US + pwm_channel1_trim + pwm_channel1_rev * (1000 * local.aileron1);
  ActuatorOutput[1] = PWM_CENTER_US + pwm_channel2_trim + pwm_channel2_rev * (1000 * local.aileron2);
  ActuatorOutput[2] = PWM_CENTER_US + pwm_channel3_trim + pwm_channel3_rev * (1000 * local.elevator);
  ActuatorOutput[3] = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * (1000 * local.throttle);
  ActuatorOutput[4] = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * (1000 * local.rudder);
}

// 直接将本周期从机逻辑输出打包，不保留额外中间缓存。
void prepareActuatorCommands() {
  if (aircraftId() != 1) return;
  for (unsigned side = 0; side < 2; ++side) {
    AircraftCommandLink *link = side == 0 ? CommandLinks::left : CommandLinks::right;
    if (!link) continue;
    for (unsigned slot = 0; slot < 3; ++slot) {
      const unsigned aircraft = slot * 2 + side;
      link->setCommandLink(slot, slaveCommands[aircraft]);
    }
  }
}

// 直接应用已准备好的物理 PWM，再发送或转发机间数据。
void applyAndTransmitActuatorCommands()
{
  // 锁定时直接回到 trim 位置；仅解锁后的控制输出应用软件限幅。
  if (isFlightLocked())
  {
    ActuatorOutput[0] = static_cast<int>(PWM_CENTER_US + pwm_channel1_trim);
    ActuatorOutput[1] = static_cast<int>(PWM_CENTER_US + pwm_channel2_trim);
    ActuatorOutput[2] = static_cast<int>(PWM_CENTER_US + pwm_channel3_trim);
    ActuatorOutput[3] = static_cast<int>(PWM_CENTER_US + pwm_channel4_trim);
    ActuatorOutput[4] = static_cast<int>(PWM_CENTER_US + pwm_channel5_trim);
  }
  else
  {
    ActuatorOutput[0] = limitActuatorPwm(ActuatorOutput[0], PWM_SERVO1_MIN_US, PWM_SERVO1_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel1_trim));
    ActuatorOutput[1] = limitActuatorPwm(ActuatorOutput[1], PWM_SERVO2_MIN_US, PWM_SERVO2_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel2_trim));
    ActuatorOutput[2] = limitActuatorPwm(ActuatorOutput[2], PWM_SERVO3_MIN_US, PWM_SERVO3_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel3_trim));
    ActuatorOutput[3] = limitActuatorPwm(ActuatorOutput[3], PWM_SERVO4_MIN_US, PWM_SERVO4_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel4_trim));
    ActuatorOutput[4] = limitActuatorPwm(ActuatorOutput[4], PWM_SERVO5_MIN_US, PWM_SERVO5_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel5_trim));
  }
  servo1.writeMicroseconds(ActuatorOutput[0]);
  servo2.writeMicroseconds(ActuatorOutput[1]);
  servo3.writeMicroseconds(ActuatorOutput[2]);
  servo4.writeMicroseconds(ActuatorOutput[3]);
  servo5.writeMicroseconds(ActuatorOutput[4]);

  // if (aircraftIsSingle())
  //   return;
  if (aircraftId() == 1)
  {
    if (CommandLinks::left)
    {
      CommandLinks::left->sendPreparedChildCommands();
    }
    if (CommandLinks::right)
    {
      CommandLinks::right->sendPreparedChildCommands();
    }
  }
  else
  {
    forwardReceivedChildSensors();
    forwardReceivedChildCommands();
  }
}
