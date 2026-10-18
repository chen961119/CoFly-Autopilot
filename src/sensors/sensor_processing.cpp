#include "math/units.h"
#include "parameters/parameter_service.h"
#include "parameters/parameter_keys.h"
#include "parameters/imu_calibration_integrity.h"
#include "config/aircraft_config.h"
#include "config/serial_config.h"
#include "sensors/sensor_processing.h"
#include "sensors/imu_internal.h"
#include "config/flight_config.h"
#include "system/flight_clock.h"
#include "math/math_utils.h"
#include "MS4525.h"
#include "MT6701.h"
#include <cmath>
#include <cstring>
#include <Wire.h>
#if defined USE_BAROMETER
#include <Adafruit_BMP280.h>
#define BMP280_I2C Wire2
Adafruit_BMP280 bmp(&BMP280_I2C);
// 滤波参数
#define FILTER_ALPHA 0.2
float filteredPressure = 101325.0f; // 初始海平面气压(Pa)
float filteredAltitude = 0;
float temp;
float pressure;
#endif

#if defined USE_MPU6050_I2C
#include <MPU6050.h>
MPU6050 mpu6050;
#endif

//========================================================================================================================//

// 根据飞控配置选择陀螺仪、加速度计的量程与原始值换算系数。

#if defined USE_MPU6050_I2C
#define GYRO_FS_SEL_250_6050 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500_6050 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000_6050 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000_6050 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2_6050 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4_6050 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8_6050 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16_6050 MPU6050_ACCEL_FS_16
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_250_6050
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_500_6050
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_1000_6050
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_2000_6050
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_2_6050
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_4_6050
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_8_6050
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_16_6050
#define ACCEL_SCALE_FACTOR 2048.0
#endif


// 以 115200 波特率启动外置 IMU 所用的 ExternalImuSerial。
void beginExternalImuLink() { ExternalImuSerial.begin(115200); }
// 以 115200 波特率启动应变传感器所用的 StrainSensorSerial。
void beginStrainSensorLink() { StrainSensorSerial.begin(115200); }

// 在启动阶段反复读取 BMI088 并迭代姿态滤波器，使姿态估计预热。
// 每轮以 2000 Hz 为目标节拍，循环次数由函数内常量决定。
void calibrateAttitude() {
  // 舵机输出前预热 IMU 和姿态滤波器；每次迭代按 2000 Hz 节拍运行。
  for (int i = 0; i <= 10000; i++) {
    updateFlightClock();
    updateInertialSensors();
    loopRate(2000);
  }
}

// 传感器测量值由本模块更新，控制器与日志模块读取。
SPISettings bmiSettings(10000000, MSBFIRST, SPI_MODE3);
FC_Binary_Packet airdata;
CalibrationAccGyroData calAccGyroData;
namespace { CalibrationAccGyroData appliedImuCalibration; }
int32_t imuCalibrationValidParameter = 0, imuCalibrationModelParameter = 1;
int32_t imuCalibrationChecksumParameter = 0;
int32_t strain_value[5] = {};
float AccX_6050, AccY_6050, AccZ_6050;
float GyroX_6050, GyroY_6050, GyroZ_6050;
float Gyro_X_EXT, Gyro_Y_EXT, Gyro_Z_EXT;
float Acc_X_EXT, Acc_Y_EXT, Acc_Z_EXT;
float airspeed;

namespace {
unsigned long last_read_Strain_Time = 0;
unsigned int read_Strain_freq = 10; // 读取应变数据频率
constexpr uint8_t SPI_READ = 0x80;
constexpr uint8_t ICM_ADDR = 0x69;
constexpr uint8_t REG_ICM_BANK_SEL = 0x76;
constexpr uint8_t REG_ICM_PWR_MGMT0 = 0x4E;
constexpr uint8_t REG_ICM_ACC_DATA = 0x1F;
constexpr uint8_t REG_ICM_GYRO_DATA = 0x25;
constexpr uint8_t BMI_ACC_ADDR = 0x19;
constexpr uint8_t BMI_GYR_ADDR = 0x69;
constexpr uint8_t REG_BMI_ACC_DATA = 0x12;
constexpr uint8_t REG_BMI_GYR_DATA = 0x02;
constexpr uint8_t REG_BMI_ACC_PWR_CTRL = 0x7D;
constexpr uint8_t REG_BMI_ACC_PWR_CONF = 0x7C;
const float inv_acc_lsb = 1.0f / 5460.0f;
const float inv_gyr_lsb = 1.0f / 16.384f;
}

// 解析 ExternalImuSerial 的 11 字节外置 IMU 帧；校验通过后更新姿态角、角速度或加速度。
// 全局输出为 rad、rad/s、m/s²；设备量程与原始帧保持不变。
void getIMUdata_EXT() {
#if defined EXTIMU
  // 串口帧长 11 字节：0x55 帧头、数据类型、8 字节数据及累加校验字节。
  static uint8_t buf[11], pos = 0;

  while (ExternalImuSerial.available()) {
    uint8_t b = ExternalImuSerial.read();

    // 检查数据头
    if (pos == 0 && b != 0x55)
      continue;
    if (pos == 1 && b != 0x52 && b != 0x59 && b != 0x51 && b != 0x53) {
      pos = 0;
      continue;
    }

    buf[pos++] = b;

    // 完整数据包处理
    if (pos == 11) {
      pos = 0;

      // 计算校验和
      uint8_t sum = buf[0] + buf[1];
      for (int i = 2; i < 10; i++)
        sum += buf[i];
      if (sum == buf[10]) {

        if (buf[1] == 0x59) { // 四元数数据
          quat[0] = ((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f;
          quat[1] = ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f;
          quat[2] = ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f;
          quat[3] = ((int16_t)(buf[9] << 8) | buf[8]) / 32768.0f;
          // USBSerial.print("Q:"); USBSerial.print(quat[0],3); USBSerial.print(",");
          // USBSerial.print(quat[1],3); USBSerial.print(","); USBSerial.print(quat[2],3);
          // USBSerial.print(","); USBSerial.println(quat[3],3);
          att[0] = -atan2(quat[0] * quat[1] + quat[2] * quat[3], 0.5f - quat[1] * quat[1] - quat[2] * quat[2]) ; // rad。
          att[1] = asin(constrain(-2.0f * (quat[1] * quat[3] - quat[0] * quat[2]), -0.999999,
                                         0.999999)) ; // rad。
          att[2] = - atan2(quat[1] * quat[2] + quat[0] * quat[3], 0.5f - quat[2] * quat[2] - quat[3] * quat[3]) ; // rad。
          // USBSerial.print("R:");
          // USBSerial.println(att[0]);
        } else if (buf[1] == 0x52) { // 角速度数据
          Gyro_X_EXT =
              Units::toRadians(-1 * (((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 2000.0f));
          Gyro_Y_EXT = Units::toRadians(((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 2000.0f);
          Gyro_Z_EXT = Units::toRadians(((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 2000.0f);
          // USBSerial.print("X:");
          // USBSerial.println(Gyro_X_EXT);
          // USBSerial.print(wy,1); USBSerial.print(","); USBSerial.println(wz,1);
        }

        else if (buf[1] == 0x51) { // 加速度数据
          Acc_X_EXT =
              ((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 16.0f * 9.8f;
          Acc_Y_EXT =
              ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 16.0f * 9.8f;
          Acc_Z_EXT =
              ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 16.0f * 9.8f;
          // USBSerial.print("A:");
          // USBSerial.println(Acc_X_EXT);
          // USBSerial.print("A:"); USBSerial.print(ax,1); USBSerial.print(",");
          // USBSerial.print(ay,1); USBSerial.print(","); USBSerial.println(az,1);
        }
      }
    }
  }

#endif
}

// 应变传感器的五通道数据按设备协议逐通道转换。
// 从应变响应帧中按设备字节顺序取出指定通道的 32 位有符号测量值。
// index 范围为 0～4；调用方需先保证响应帧长度足够。
int32_t parseChannel(uint8_t *buf, int index) {
  // index 为通道编号，范围 0～4。

  int offset = 3 + index * 4;

  union {
    int32_t val;
    uint8_t b[4];
  } u;

  // 按手册：低位在前，高位在后
  u.b[0] = buf[offset + 1];
  u.b[1] = buf[offset + 0];
  u.b[2] = buf[offset + 3];
  u.b[3] = buf[offset + 2];

  return u.val;
}

// Modbus CRC16，发送时低字节在前。
// 计算 Modbus RTU 的 CRC16；返回值发送时先写低字节。
uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];

    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// 发送功能码 0x03 的寄存器读取请求。
// 构造应变传感器的 8 字节 Modbus 读取请求并写入 StrainSensorSerial。
// addr 为设备地址，fcode 为功能码，reg 为起始寄存器，num 为寄存器数量。
void sendStrainCommand(uint8_t addr, uint16_t fcode, uint16_t reg,
                       uint16_t num) {

  uint8_t frame[8];

  frame[0] = addr;
  frame[1] = fcode;
  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;
  frame[4] = num >> 8;
  frame[5] = num & 0xFF;

  uint16_t crc = modbusCRC(frame, 6);

  frame[6] = crc & 0xFF; // CRC低位
  frame[7] = crc >> 8;   // CRC高位

  // USBSerial.write(frame, 8);
  StrainSensorSerial.write(frame, 8);
}

// 按设定频率请求 10 个寄存器，并在收到足够长的响应时更新五路应变值。
void Strain_read_all() {
  // 按 read_Strain_freq 限制请求频率；只在收到完整响应时更新五通道值。
  float invFreq = 1.0 / read_Strain_freq * 1000000.0;
  unsigned long checker = micros();

  if (checker - last_read_Strain_Time < invFreq)
    return;
  last_read_Strain_Time = checker;
  // 从地址 0x0000 起读取 10 个寄存器。
  sendStrainCommand(1, 0x03, 0x0000, 10);
  uint8_t buf[32];
  int len = 0;

  while (StrainSensorSerial.available()) {
    // USBSerial.println("available ");
    buf[len++] = StrainSensorSerial.read();
  }

  // 解析
  if (len >= 25 && buf[1] == 0x03) {

    strain_value[0] = parseChannel(buf, 0);
    strain_value[1] = parseChannel(buf, 1);
    strain_value[2] = parseChannel(buf, 2);
    strain_value[3] = parseChannel(buf, 3);
    strain_value[4] = parseChannel(buf, 4);
  }
}

// 经 I2C 连续读取 ICM42688 三轴加速度和角速度，并换算到共享 IMU 变量。
// 当前量程换算后的全局单位分别为 m/s² 和 rad/s；通信失败时保持旧值。
void getICM42688data() {
  // 切换到寄存器 Bank 0 后连续读取三轴加速度和三轴角速度。
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_BANK_SEL);
  Wire.write(0x00);
  Wire.endTransmission();

  // 连续读取 12 字节：前 6 字节为加速度，后 6 字节为陀螺仪。
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_ACC_DATA);
  if (Wire.endTransmission(false) != 0)
    return; // 通讯失败直接退出

  if (Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)12) == 12) {
    // ICM-42688 数据按高字节在前拼接为有符号 16 位整数。
    int16_t raw_ax = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_ay = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_az = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gx = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gy = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gz = (int16_t)(Wire.read() << 8 | Wire.read());

    // 保持控制器沿用的 6050 变量名；按当前 ±16 g、±2000 deg/s 量程换算。
    AccX_6050 = Units::acceleration(raw_ax / 2048.0);
    AccY_6050 = Units::acceleration(raw_ay / 2048.0);
    AccZ_6050 = Units::acceleration(raw_az / 2048.0);

    GyroX_6050 = Units::toRadians(raw_gx / 16.4);
    GyroY_6050 = Units::toRadians(raw_gy / 16.4);
    GyroZ_6050 = Units::toRadians(raw_gz / 16.4);

    // 4. 应用低通滤波 (保留你原有的 B_accel_6050 逻辑)
    /*
    AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 *
    AccX_6050; AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050
    * AccY_6050; AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 +
    B_accel_6050 * AccZ_6050; AccX_prev_6050 = AccX_6050; AccY_prev_6050 =
    AccY_6050; AccZ_prev_6050 = AccZ_6050;

    GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 *
    GyroX_6050; GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050
    * GyroY_6050; GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 +
    B_gyro_6050 * GyroZ_6050; GyroX_prev_6050 = GyroX_6050; GyroY_prev_6050 =
    GyroY_6050; GyroZ_prev_6050 = GyroZ_6050;
    */
  }
}

// 经 SPI 读取 BMI088，加速度与角速度按安装方向旋转 90° 并扣除 EEPROM 零偏。
// 结果写入沿用的 6050 变量，单位分别为 m/s² 和 rad/s。
void getBMI088data() {
#if defined INTIMU
  // 加速度计 SPI 读取需要一个哑字节；陀螺仪读取不需要。

  uint8_t buf[6];
  const float inv_acc = 1.0f / 5460.0f; // ±6g
  const float inv_gyr = 1.0f / 16.384f; // ±2000dps
  static unsigned long lastBMI088AccDebug = 0;
  static unsigned long lastBMI088GyrDebug = 0;

  // 读取加速度计原始值。
  SPI.beginTransaction(bmiSettings);
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(0x12 | SPI_READ);
  SPI.transfer(0x00); // 丢弃协议要求的哑字节。
  for (int i = 0; i < 6; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_ACC, HIGH);
  SPI.endTransaction();

  int16_t rx = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t ry = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t rz = (int16_t)(buf[4] | (buf[5] << 8));

  // 读取陀螺仪原始值。
  SPI.beginTransaction(bmiSettings);
  digitalWrite(CS_GYR, LOW);
  SPI.transfer(0x02 | SPI_READ);
  for (int i = 0; i < 6; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_GYR, HIGH);
  SPI.endTransaction();

  int16_t rgx = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t rgy = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t rgz = (int16_t)(buf[4] | (buf[5] << 8));

  // 按安装方向绕 Z 轴旋转 90°：新 X=旧 Y，新 Y=-旧 X，Z 不变。
  AccX_6050 = Units::acceleration((ry * inv_acc) - appliedImuCalibration.AccErrorX_6050);
  AccY_6050 = Units::acceleration((-rx * inv_acc) - appliedImuCalibration.AccErrorY_6050);
  AccZ_6050 = Units::acceleration((rz * inv_acc) - appliedImuCalibration.AccErrorZ_6050);

  GyroX_6050 = Units::toRadians((rgy * inv_gyr) - appliedImuCalibration.GyroErrorX_6050);
  GyroY_6050 = Units::toRadians((-rgx * inv_gyr) - appliedImuCalibration.GyroErrorY_6050);
  GyroZ_6050 = Units::toRadians((rgz * inv_gyr) - appliedImuCalibration.GyroErrorZ_6050);

  // --- C. 应用低通滤波 (保留你原有的 B_accel_6050 参数逻辑) ---
  /*
  AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 * AccX_6050;
  AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050 * AccY_6050;
  AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 + B_accel_6050 * AccZ_6050;
  AccX_prev_6050 = AccX_6050; AccY_prev_6050 = AccY_6050; AccZ_prev_6050 =
  AccZ_6050;

  GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 * GyroX_6050;
  GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050 * GyroY_6050;
  GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 + B_gyro_6050 * GyroZ_6050;
  GyroX_prev_6050 = GyroX_6050; GyroY_prev_6050 = GyroY_6050; GyroZ_prev_6050 =
  GyroZ_6050;
  */

#endif
}

// 以十六进制打印原始字节和三轴有符号值；用于核对 BMI088 SPI 读数。
void printBMI088Raw(const char *tag, const uint8_t *buf, int len, int16_t x,
                    int16_t y, int16_t z) {
  USBSerial.print(tag);
  USBSerial.print(F(" raw:"));
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10)
      USBSerial.print('0');
    USBSerial.print(buf[i], HEX);
    if (i < len - 1)
      USBSerial.print(' ');
  }
  USBSerial.print(F(" val:"));
  USBSerial.print(x);
  USBSerial.print(F(","));
  USBSerial.print(y);
  USBSerial.print(F(","));
  USBSerial.println(z);
}

// 辅助函数：SPI 写寄存器
// 向指定片选引脚对应的 SPI 设备写入一个寄存器值。
void writeRegSPI(int cs, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(bmiSettings);
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// 通用 I2C 寄存器写入辅助函数
// 通过指定 I2C 总线向设备地址和寄存器写入一个字节。
void writeReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  bus.endTransmission();
}


float relativeAngle_raw, relativeAngle_ready, relativeAngle_offset;


namespace {
MT6701 angleSensor(Wire1);
bool angleReadValid = false;
bool angleReadAttempted = false;
unsigned long angleReadAttemptTime = 0;
constexpr unsigned long ANGLE_RETRY_INTERVAL_US = 20000;
} // namespace


// CUAV MEAS/MS4525 空速计：底板6号口，SCL2=24（TX6）、SDA2=25（RX6）。
MS4525read_Task airspeedSensor(&Wire2);
namespace {
bool airspeedReadValid = false;
uint32_t lastValidAirspeedTime = 0;
}

bool airspeedValid() {
  return airspeedReadValid &&
      static_cast<uint32_t>(micros() - lastValidAirspeedTime) <= 100000UL;
}

AirspeedDiagnostics airspeedDiagnostics() {
  const auto &sample = airspeedSensor.MS4525_param;
  return {airspeedSensor.lastReadBytes, airspeedSensor.lastStatus,
          airspeedSensor.lastPressureCounts, airspeedSensor.lastReadOk,
          airspeedSensor.successfulReads, airspeedSensor.failedReads,
          sample.pressure * PSI_to_Pa, sample.pressure_drift * PSI_to_Pa,
          (sample.pressure - sample.pressure_drift) * PSI_to_Pa, sample.temperature};
}


float B_accel_6050 =
    0.6; // MPU6050 加速度一阶低通系数；越小，平滑越强、延迟越大。
float B_gyro_6050 = 0.47;  // MPU6050 陀螺仪低通系数。


// 内置 IMU 的静态校准常量；在线校准结果另存于 calAccGyroData。
// MPU6050

float AccErrorX_6050 = Units::acceleration(-0.02f);
float AccErrorY_6050 = Units::acceleration(-0.01f);
float AccErrorZ_6050 = Units::acceleration(0.03f);
float GyroErrorX_6050 = Units::toRadians(-13.83f);
float GyroErrorY_6050 = Units::toRadians(-9.41f);
float GyroErrorZ_6050 = Units::toRadians(1.48f);


// 内置 IMU 当前值与一阶低通的上一采样值。
float AccX_prev_6050, AccY_prev_6050, AccZ_prev_6050;
float GyroX_prev_6050, GyroY_prev_6050, GyroZ_prev_6050;


// 转角零偏由统一参数服务恢复。
void loadRotateSensorOffset() {
  // Already restored and range checked by the unified parameter service.

}
// 上电校零前显式初始化 Wire2；Serial6 已从机间通信中释放。
void calibrateAirspeedSensor() {
  Wire2.setSCL(24);
  Wire2.setSDA(25);
  Wire2.begin();
  Wire2.setClock(100000);
  airspeedSensor.calib();
}

// 按编译配置初始化内置 IMU 的总线、电源状态、量程及采样参数。
void IMUinit() {
#if defined INTIMU
// 根据编译配置初始化内置 IMU；BMI088 通过 SPI。
#if defined USE_MPU6050_I2C
  Wire.begin();
  // Wire.setClock(1000000);  // 历史配置：1 MHz 超过器件手册常用的 400 kHz。
  // max...

  // mpu6050.initialize();
  //  --- 插入 ICM-42688 初始化 --- [cite: 1833]
  // Wire.setClock(1000000); // 42688p建议 400kHz 比较稳
  // Wire.beginTransmission(ICM_ADDR);
  // Wire.write(REG_ICM_PWR_MGMT0);
  // Wire.write(0x0F); // 开启加速度计和陀螺仪的低噪声模式
  // Wire.endTransmission();
  delay(10);

  // 1. 初始化引脚
  pinMode(CS_ACC, OUTPUT);
  pinMode(CS_GYR, OUTPUT);
  digitalWrite(CS_ACC, HIGH);
  digitalWrite(CS_GYR, HIGH);

  // 2. 启动 SPI
  SPI.begin();
  delay(100);

  // 3. 唤醒加速度计 (必需序列)
  writeRegSPI(CS_ACC, 0x7D, 0x04); // ACC_PWR_CTRL: Active
  delay(10);
  writeRegSPI(CS_ACC, 0x7C, 0x00); // ACC_PWR_CONF: Active
  delay(10);

  // 4. 配置陀螺仪
  writeRegSPI(CS_GYR, 0x0F, 0x00); // ±2000 dps
  writeRegSPI(CS_GYR, 0x10, 0x02); // ODR 1000Hz, BW 116Hz

  USBSerial.println("BMI088 SPI Initialized.");

  /*
    if (mpu6050.testConnection() == false) {
      USBSerial.println("MPU6050 initialization unsuccessful");
      USBSerial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }
    */

  // 配置量程；BMI088 的电源和陀螺仪寄存器已在上方写入。
  mpu6050.setFullScaleGyroRange(GYRO_SCALE_6050);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE_6050);
#endif


#endif
}


// 读取 MPU6050 原始惯性数据，做单位换算、零偏校正和一阶低通。
// 这是旧内置 IMU 路径，是否调用由当前硬件配置决定。
void getIMUdata() {
  // 读取内置 IMU 原始值，换算单位、扣除零偏，再做一阶低通。
  // 全局加速度为 m/s²、角速度为 rad/s；标定参数仍为 g、°/s。
  int16_t AcX_6050, AcY_6050, AcZ_6050, GyX_6050, GyY_6050, GyZ_6050;

#if defined USE_MPU6050_I2C
  mpu6050.getMotion6(&AcX_6050, &AcY_6050, &AcZ_6050, &GyX_6050, &GyY_6050,
                     &GyZ_6050);
#endif

  // 加速度：寄存器量程 g 转为 m/s²，标定零偏在读入时同样换算。
#if defined USE_MPU6050_I2C
  AccX_6050 = Units::acceleration(AcX_6050 / ACCEL_SCALE_FACTOR);
  AccY_6050 = Units::acceleration(AcY_6050 / ACCEL_SCALE_FACTOR);
  AccZ_6050 = Units::acceleration(AcZ_6050 / ACCEL_SCALE_FACTOR);
  // MPU6050 加速度零偏校正。
  AccX_6050 = AccX_6050 - Units::acceleration(appliedImuCalibration.AccErrorX_6050);
  AccY_6050 = AccY_6050 - Units::acceleration(appliedImuCalibration.AccErrorY_6050);
  AccZ_6050 = AccZ_6050 - Units::acceleration(appliedImuCalibration.AccErrorZ_6050);
  // MPU6050 加速度一阶低通。
  AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 * AccX_6050;
  AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050 * AccY_6050;
  AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 + B_accel_6050 * AccZ_6050;
  AccX_prev_6050 = AccX_6050;
  AccY_prev_6050 = AccY_6050;
  AccZ_prev_6050 = AccZ_6050;

#endif
  // 角速度：寄存器量程 °/s 转为 rad/s，标定零偏在读入时同样换算。
#if defined USE_MPU6050_I2C
  GyroX_6050 = Units::toRadians(GyX_6050 / GYRO_SCALE_FACTOR);
  GyroY_6050 = Units::toRadians(GyY_6050 / GYRO_SCALE_FACTOR);
  GyroZ_6050 = Units::toRadians(GyZ_6050 / GYRO_SCALE_FACTOR);
  // MPU6050 陀螺仪零偏校正。
  GyroX_6050 = GyroX_6050 - Units::toRadians(appliedImuCalibration.GyroErrorX_6050);
  GyroY_6050 = GyroY_6050 - Units::toRadians(appliedImuCalibration.GyroErrorY_6050);
  GyroZ_6050 = GyroZ_6050 - Units::toRadians(appliedImuCalibration.GyroErrorZ_6050);
  // MPU6050 角速度一阶低通。
  GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 * GyroX_6050;
  GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050 * GyroY_6050;
  GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 + B_gyro_6050 * GyroZ_6050;
  GyroX_prev_6050 = GyroX_6050;
  GyroY_prev_6050 = GyroY_6050;
  GyroZ_prev_6050 = GyroZ_6050;

#endif
}


namespace {
bool imuCalibrationReady = false;
bool validImuOffsets(const CalibrationAccGyroData &data) {
  const float values[] = {data.AccErrorX_6050, data.AccErrorY_6050,
                         data.AccErrorZ_6050, data.GyroErrorX_6050,
                         data.GyroErrorY_6050, data.GyroErrorZ_6050};
  for (unsigned i = 0; i < 6; ++i) {
    uint32_t bits;
    memcpy(&bits, &values[i], sizeof(bits));
    // 位检查在 -ffast-math 下仍能识别 NaN/Inf。
    if ((bits & 0x7f800000u) == 0x7f800000u) return false;
    const float limit = i < 3 ? 6.0f : 2000.0f;
    if (values[i] < -limit || values[i] > limit) return false;
  }
  return true;
}
} // namespace

bool imuCalibrationValid() { return imuCalibrationReady; }

// 静置采集 BMI088 多组数据，计算旋转后机体系的加速度和角速度零偏。
// 完成后在 RAM 生效，按普通参数逻辑延迟保存；标定期间应保持水平且静止。
void calculate_IMU_error() {
  // 采集 12000 组 BMI088 数据，计算机体系静态零偏并交给统一参数服务。
  double sAX = 0, sAY = 0, sAZ = 0, sGX = 0, sGY = 0, sGZ = 0;
  int c = 0;
  uint8_t buf[6];

  USBSerial.println(
      "BMI088 SPI Calibrating (Rotated 90)... Keep vehicle flat and still.");

  while (c < 12000) {
    // 1. SPI 读取原始加速度
    SPI.beginTransaction(bmiSettings);
    digitalWrite(CS_ACC, LOW);
    SPI.transfer(0x12 | 0x80); // 读取加速度起始地址
    SPI.transfer(0x00); // 重要：BMI088 加速度计 SPI 读取必需的 Dummy Byte
    for (int i = 0; i < 6; i++)
      buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_ACC, HIGH);
    SPI.endTransaction();

    int16_t rx = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t ry = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t rz = (int16_t)(buf[4] | (buf[5] << 8));

    // 2. SPI 读取原始陀螺仪
    SPI.beginTransaction(bmiSettings);
    digitalWrite(CS_GYR, LOW);
    SPI.transfer(0x02 | 0x80); // 读取陀螺仪起始地址
    for (int i = 0; i < 6; i++)
      buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_GYR, HIGH);
    SPI.endTransaction();

    int16_t rgx = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t rgy = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t rgz = (int16_t)(buf[4] | (buf[5] << 8));

    // 3. 映射与累加 (保持 90 度旋转逻辑: 新X=旧Y, 新Y=-旧X)
    sAX += ry / 5460.0;
    sAY += -rx / 5460.0;
    sAZ += rz / 5460.0;

    sGX += rgy / 16.384;
    sGY += -rgx / 16.384;
    sGZ += rgz / 16.384;

    if (c % 1000 == 0)
      USBSerial.print(".");
    c++;
    delayMicroseconds(100);
  }

  const CalibrationAccGyroData old = calAccGyroData;
  // 4. 计算均值并补偿 Z 轴重力 (假设平放时 Z 应为 1g)
  calAccGyroData.AccErrorX_6050 = (float)(sAX / c);
  calAccGyroData.AccErrorY_6050 = (float)(sAY / c);
  calAccGyroData.AccErrorZ_6050 = (float)((sAZ / c) - 1.0f);

  calAccGyroData.GyroErrorX_6050 = (float)(sGX / c);
  calAccGyroData.GyroErrorY_6050 = (float)(sGY / c);
  calAccGyroData.GyroErrorZ_6050 = (float)(sGZ / c);

  const CalibrationAccGyroData measured = calAccGyroData;
  calAccGyroData = old;
  float offsets[ParameterKey::ImuOffsetCount];
  static_assert(sizeof(measured) == sizeof(offsets), "Calibration field layout changed");
  memcpy(offsets, &measured, sizeof(offsets));
  if (!validImuOffsets(measured) || !saveImuCalibrationParameters(offsets)) {
    USBSerial.println("\nIMU calibration invalid or aircraft unlocked; calibration not accepted.");
    loadImuCalibration();
    return;
  }
  loadImuCalibration();
  USBSerial.println("\nBMI088 calibration verified and applied; changes save after 2 seconds.");
}


// 函数结尾停机，不会返回飞行主循环。


// 每隔至少 20 ms 从空速驱动刷新一次 airspeed。
void getairspeed() {
  // 以 50 Hz 更新空速；其他控制周期沿用上次有效测量。
  static unsigned long lastAirspeedRead = 0;
  if (micros() - lastAirspeedRead > 20000) {
    lastAirspeedRead = micros();
    airspeed = airspeedSensor.getAirspeed();
    uint32_t bits;
    memcpy(&bits, &airspeed, sizeof(bits));
    airspeedReadValid = airspeedSensor.lastReadOk && airspeedSensor.lastStatus == 0 &&
        (bits & 0x7f800000u) != 0x7f800000u && airspeed >= 0 && airspeed <= 100;
    if (airspeedReadValid) lastValidAirspeedTime = micros();
  }
}


// 启用气压计时读取温度、气压和高度，并打印测量值。
void getbarodata() {
  // 读取气压计温度、气压和按标准海平面气压换算的高度。
#if defined USE_BAROMETER
  // 快速读取原始数据
  temp = bmp.readTemperature() + 273.15f;  // K
  pressure = bmp.readPressure(); // Pa
  // 计算海拔高度(国际标准大气模型)
  filteredAltitude = bmp.readAltitude(1013.25);
  // 输出调试信息
  USBSerial.printf("Temp: %.1fK | Pressure: %.2fPa | Alt: %.2fm\n", temp, pressure,
                filteredAltitude);
#endif
}


// 启用气压计时初始化 BMP280，配置 I2C 时钟、采样倍率和内部滤波。
void initBAROMETER() {
#if defined USE_BAROMETER

  BMP280_I2C.begin();
  BMP280_I2C.setClock(400000); // 提升至400kHz
  if (!bmp.begin(0x76)) {      // 尝试默认地址0x76
    USBSerial.println("BMP280未找到，尝试0x77...");
    if (!bmp.begin(0x77)) {
      USBSerial.println("BMP280初始化失败!");
      // while(1);
    }
  }

  // 配置高性能模式
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   // 连续采样
                  Adafruit_BMP280::SAMPLING_X2,   // 温度采样
                  Adafruit_BMP280::SAMPLING_X16,  // 气压采样
                  Adafruit_BMP280::FILTER_X16,    // IIR滤波
                  Adafruit_BMP280::STANDBY_MS_500 // 待机时间
  );
#endif
}


// MT6701 接入屏幕所在 Wire1 总线；初始化一次，不在采样时重设总线。
void initializeRotateSensor() {
  Wire1.begin();
  Wire1.setClock(100000);
  angleReadValid = false;
  angleReadAttempted = false;
  getRotateSensor1();
}

bool rotateSensorValid() { return angleReadValid; }

// 每周期读取绝对角，减去本机零点并处理跨越 0/360 度的夹角。
void getRotateSensor1() {
  const unsigned long now = micros();
  if (!angleReadValid && angleReadAttempted &&
      now - angleReadAttemptTime < ANGLE_RETRY_INTERVAL_US) return;
  angleReadAttempted = true;
  angleReadAttemptTime = now;
  float angle;
  angleReadValid = angleSensor.readAngle(angle);
  if (!angleReadValid) return; // 保持最后有效角度，屏幕通过状态显示 ERR。
  relativeAngle_raw = Units::toRadians(angle);
  const bool reverse = !aircraftIsRight();
  relativeAngle_ready = Units::toRadians(MT6701::relativeAngle(angle, relativeAngle_offset, reverse));
}

// 将当前原始转角设为新零点，经统一参数服务延迟合并保存。
void ResetRotateSensor() {
  if (!angleReadValid) return; // 未连接或读取失败时不覆盖已保存零点。
  if (!saveParameterValue(ParameterKey::Mt6701Zero, Units::toDegrees(relativeAngle_raw))) {
    USBSerial.println("MT6701 zero change rejected");
    return;
  }
  relativeAngle_ready = 0;
  USBSerial.println("Zero Set! Offset: " + String(relativeAngle_offset));
}


// 验证已恢复的惯性传感器校准参数。
void loadImuCalibration() {
  float offsets[ParameterKey::ImuOffsetCount]; memcpy(offsets,&calAccGyroData,sizeof(offsets));
  imuCalibrationReady = imuCalibrationValidParameter == 1 &&
      imuCalibrationModelParameter == 1 && validImuOffsets(calAccGyroData) &&
      imuCalibrationChecksumParameter == imuCalibrationFingerprint(offsets,imuCalibrationModelParameter);
  if (!imuCalibrationReady) {
    imuCalibrationValidParameter = 0;
  }
  appliedImuCalibration = imuCalibrationReady ? calAccGyroData : CalibrationAccGyroData{};
}

// 内置 IMU 启用时读取首帧 BMI088，并据此初始化姿态四元数。
void initializeInitialAttitude() {
  // 上电时用首帧加速度估计初始姿态四元数。
#if defined INTIMU && !defined EXTIMU
  getBMI088data();
  float phi, theta, psi;
#if defined USE_MPU6050_I2C
  phi = atan2(AccY_6050, AccZ_6050); // 滚转角，绕 x 轴。
  theta = atan2(-AccX_6050, sqrt(AccY_6050 * AccY_6050 +
                                 AccZ_6050 * AccZ_6050)); // 俯仰角，绕 y 轴。
  psi = 0;                                                // 无磁力计时偏航角初始化为 0。
#endif


  const Eigen::Quaternionf initialAttitude = eulerToQuaternion(phi, theta, psi);
  quat[0] = initialAttitude.w();
  quat[1] = initialAttitude.x();
  quat[2] = initialAttitude.y();
  quat[3] = initialAttitude.z();
#endif
}

namespace {
// 角加速度低通滤波器（三轴）
static PX4LowPassFilter2p gyroDerivFiltX, gyroDerivFiltY,
    gyroDerivFiltZ; // getAngularACC() 求 ang_acc[0]/ang_acc[1]/ang_acc[2] 前的角速度滤波

static PX4LowPassFilter2p angularAccFiltX, angularAccFiltY, angularAccFiltZ;

// 前次滤波后陀螺仪值（用于差分）
float gyroX_filt_prev = 0.0f;
float gyroY_filt_prev = 0.0f;
float gyroZ_filt_prev = 0.0f;
// 前次差分时间（微秒）
unsigned long prev_time_gyro_deriv = 0;


} // namespace

float ang_vel[3] = {};
float ang_vel_filtered[3] = {};
float ang_acc[3] = {};
float acc[3] = {};

// 为角速度差分前后两级滤波器设置 500 Hz 采样参数。
void initializeAngularAccelerationFilters() {
  angularAccFiltX.set_cutoff_frequency(500, 100);
  angularAccFiltY.set_cutoff_frequency(500, 20);
  angularAccFiltZ.set_cutoff_frequency(500, 100);
  gyroDerivFiltX.set_cutoff_frequency(500, 100);
  gyroDerivFiltY.set_cutoff_frequency(500, 40);
  gyroDerivFiltZ.set_cutoff_frequency(500, 100);
}

// 由选定 IMU 的 ang_vel 估计 ang_acc，单位为 rad/s²；异常采样间隔时清零输出。
void getAngularACC() {
  // 先平滑角速度再差分，最后滤波并限幅，避免差分放大传感器噪声。
  unsigned long now = micros();

  float dt_deriv = (now - prev_time_gyro_deriv) * 1.0e-6f;
  prev_time_gyro_deriv = now;

  if (dt_deriv <= 0.0f || dt_deriv > 0.01f) {
    // 首次调用或采样中断后重建历史值，跳过本次角加速度计算。
    gyroX_filt_prev = Units::toRadians(gyroDerivFiltX.apply(Units::toDegrees(ang_vel[0])));
    gyroY_filt_prev = Units::toRadians(gyroDerivFiltY.apply(Units::toDegrees(ang_vel[1])));
    gyroZ_filt_prev = Units::toRadians(gyroDerivFiltZ.apply(Units::toDegrees(ang_vel[2])));
    ang_acc[0] = 0.0f;
    ang_acc[1] = 0.0f;
    ang_acc[2] = 0.0f;
    return;
  }

  float gyroX_now = gyroDerivFiltX.apply(Units::toDegrees(ang_vel[0]));
  float gyroY_now = gyroDerivFiltY.apply(Units::toDegrees(ang_vel[1]));
  float gyroZ_now = gyroDerivFiltZ.apply(Units::toDegrees(ang_vel[2]));

  float dp_raw = (gyroX_now - Units::toDegrees(gyroX_filt_prev)) / dt_deriv;
  float dq_raw = (gyroY_now - Units::toDegrees(gyroY_filt_prev)) / dt_deriv;
  float dr_raw = (gyroZ_now - Units::toDegrees(gyroZ_filt_prev)) / dt_deriv;

  ang_acc[0] = Units::toRadians(angularAccFiltX.apply(dp_raw));
  ang_acc[1] = Units::toRadians(angularAccFiltY.apply(dq_raw));
  ang_acc[2] = Units::toRadians(angularAccFiltZ.apply(dr_raw));

  gyroX_filt_prev = Units::toRadians(gyroX_now);
  gyroY_filt_prev = Units::toRadians(gyroY_now);
  gyroZ_filt_prev = Units::toRadians(gyroZ_now);

  const float deriv_limit = 3000.0f;
  ang_acc[0] = Units::toRadians(constrain(Units::toDegrees(ang_acc[0]), -deriv_limit, deriv_limit));
  ang_acc[1] = Units::toRadians(constrain(Units::toDegrees(ang_acc[1]), -deriv_limit, deriv_limit));
  ang_acc[2] = Units::toRadians(constrain(Units::toDegrees(ang_acc[2]), -deriv_limit, deriv_limit));
}


// 数据源优先级与原控制一致：外置 IMU 优先，否则使用选定内置 IMU。
void updateSensorMeasurements() {
#if defined EXTIMU
  ang_vel[0] = Gyro_X_EXT; 
  ang_vel[1] = Gyro_Y_EXT; 
  ang_vel[2] = Gyro_Z_EXT;
  acc[0] = Acc_X_EXT; 
  acc[1] = Acc_Y_EXT; 
  acc[2] = Acc_Z_EXT;
#elif defined INTIMU
  ang_vel[0] = - GyroX_6050; 
  ang_vel[1] = GyroY_6050; 
  ang_vel[2] = - GyroZ_6050;
  acc[0] = - AccX_6050; 
  acc[1] = AccY_6050; 
  acc[2] = AccZ_6050;
#endif
  updateAngularVelocityFilters();
}

void updateInertialSensors() {
#if defined EXTIMU
  getIMUdata_EXT();
#elif defined INTIMU
  getBMI088data();
#endif
  Madgwick();
  updateSensorMeasurements();
  getAngularACC();
}
