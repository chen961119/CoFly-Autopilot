#include "parameters/control_parameters.h"
#include "config/aircraft_config.h"
#include "actuators/actuator_output.h"
#include "control/control_modes.h"
#include "control/command_generation.h"
#include "config/flight_config.h"
#include "system/flight_lock.h"
#include "actuators/actuator_pwm_limits.h"
#include "actuators/actuator_pwm_mapping.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include <Servo.h>
#include <control/control_pipeline.h>

// 最终物理 PWM（μs），顺序为副翼1、副翼2、升降舵、油门、方向舵。
float ActuatorOutput[5] = {};

// 本机 SD 保存的软件输出范围；驱动 attach 范围仍由 flight_config.h 定义。
int32_t pwm_servo1_min_us = 1000;
int32_t pwm_servo1_max_us = 2000;
int32_t pwm_servo2_min_us = 1000;
int32_t pwm_servo2_max_us = 2000;
int32_t pwm_servo3_min_us = 1000;
int32_t pwm_servo3_max_us = 2000;
int32_t pwm_servo4_min_us = 1000;
int32_t pwm_servo4_max_us = 2000;
int32_t pwm_servo5_min_us = 1000;
int32_t pwm_servo5_max_us = 2000;

int32_t pwm_channel1_rev = -1;
int32_t pwm_channel2_rev = 1;
int32_t pwm_channel3_rev = 1;
int32_t pwm_channel4_rev = 1;
int32_t pwm_channel5_rev = 1;

// 每路独立选择映射，默认使用 min/trim/max 三点插值。
int32_t pwm_channel1_mode = 0;
int32_t pwm_channel2_mode = 0;
int32_t pwm_channel3_mode = 0;
int32_t pwm_channel4_mode = 0;
int32_t pwm_channel5_mode = 0;

// 本机绝对配平/锁定 PWM（μs），默认 1500；开机从本机 SD 卡恢复。
// mode=0 时 trim 同时为映射中位；mode=1 时仅用于上电、锁定及异常回退。
// trim 不随 rev 反向，各机在各自 USB 参数表中校准并保存。
float pwm_channel1_trim = 1500;
float pwm_channel2_trim = 1500;
float pwm_channel3_trim = 1500;
float pwm_channel4_trim = 1500;
float pwm_channel5_trim = 1500;

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
  ActuatorOutput[0] = static_cast<int>(pwm_channel1_trim);
  servo1.writeMicroseconds(ActuatorOutput[0]);
  ActuatorOutput[1] = static_cast<int>(pwm_channel2_trim);
  servo2.writeMicroseconds(ActuatorOutput[1]);
  ActuatorOutput[2] = static_cast<int>(pwm_channel3_trim);
  servo3.writeMicroseconds(ActuatorOutput[2]);
  ActuatorOutput[3] = static_cast<int>(pwm_channel4_trim);
  servo4.writeMicroseconds(ActuatorOutput[3]);
  ActuatorOutput[4] = static_cast<int>(pwm_channel5_trim);
  servo5.writeMicroseconds(ActuatorOutput[4]);
}

// 只将融合后的归一化指令转换到本机物理 PWM；舵面均下偏为正，rev 仅适配安装方向。
void convertControlCommandsToPWM()
{
  // 每周期使用当前参数计算行程，SD 加载和在线修改后无需刷新缓存。
  ActuatorOutput[0] = actuatorCommandToPwm(commands.local.aileron1, pwm_channel1_rev,
      pwm_servo1_min_us, pwm_channel1_trim, pwm_servo1_max_us, pwm_channel1_mode);
  ActuatorOutput[1] = actuatorCommandToPwm(commands.local.aileron2, pwm_channel2_rev,
      pwm_servo2_min_us, pwm_channel2_trim, pwm_servo2_max_us, pwm_channel2_mode);
  ActuatorOutput[2] = actuatorCommandToPwm(commands.local.elevator, pwm_channel3_rev,
      pwm_servo3_min_us, pwm_channel3_trim, pwm_servo3_max_us, pwm_channel3_mode);
  ActuatorOutput[3] = actuatorCommandToPwm(commands.local.throttle, pwm_channel4_rev,
      pwm_servo4_min_us, pwm_channel4_trim, pwm_servo4_max_us, pwm_channel4_mode);
  ActuatorOutput[4] = actuatorCommandToPwm(commands.local.rudder, pwm_channel5_rev,
      pwm_servo5_min_us, pwm_channel5_trim, pwm_servo5_max_us, pwm_channel5_mode);
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
    ActuatorOutput[0] = static_cast<int>(pwm_channel1_trim);
    ActuatorOutput[1] = static_cast<int>(pwm_channel2_trim);
    ActuatorOutput[2] = static_cast<int>(pwm_channel3_trim);
    ActuatorOutput[3] = static_cast<int>(pwm_channel4_trim);
    ActuatorOutput[4] = static_cast<int>(pwm_channel5_trim);
  }
  else
  {
    ActuatorOutput[0] = limitActuatorPwm(ActuatorOutput[0], pwm_servo1_min_us, pwm_servo1_max_us, static_cast<int>(pwm_channel1_trim));
    ActuatorOutput[1] = limitActuatorPwm(ActuatorOutput[1], pwm_servo2_min_us, pwm_servo2_max_us, static_cast<int>(pwm_channel2_trim));
    ActuatorOutput[2] = limitActuatorPwm(ActuatorOutput[2], pwm_servo3_min_us, pwm_servo3_max_us, static_cast<int>(pwm_channel3_trim));
    ActuatorOutput[3] = limitActuatorPwm(ActuatorOutput[3], pwm_servo4_min_us, pwm_servo4_max_us, static_cast<int>(pwm_channel4_trim));
    ActuatorOutput[4] = limitActuatorPwm(ActuatorOutput[4], pwm_servo5_min_us, pwm_servo5_max_us, static_cast<int>(pwm_channel5_trim));
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
