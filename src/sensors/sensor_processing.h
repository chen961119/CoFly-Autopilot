#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "sensors/filters.h"

// 外置空速/迎角传感器的打包数据，字段顺序与串口协议保持一致。
#pragma pack(push, 1)
struct FC_Binary_Packet {
  uint8_t head1;
  uint8_t head2;
  float aoa; // 线上 deg；全局 airdata 发布 rad。
  float aos; // 线上 deg；全局 airdata 发布 rad。
  float tas; // m/s。
  uint8_t statA;
  uint8_t statS;
  uint8_t checksum;
};
#pragma pack(pop)

typedef struct {
  // 内置 IMU 的加速度与陀螺仪零偏，由统一参数服务按名称存储。
  float AccErrorX_6050;
  float AccErrorY_6050;
  float AccErrorZ_6050;

  float GyroErrorX_6050;
  float GyroErrorY_6050;
  float GyroErrorZ_6050;
} CalibrationAccGyroData;

extern FC_Binary_Packet airdata;
extern CalibrationAccGyroData calAccGyroData;
extern int32_t imuCalibrationValidParameter, imuCalibrationModelParameter;
extern int32_t imuCalibrationChecksumParameter;
// 持久化标定参数保持度制；不是运行时物理状态。
extern float relativeAngle_offset;
extern SPISettings bmiSettings;
// 五路应变原始测量值，顺序与传感器报文通道 1～5 一致。
extern int32_t strain_value[5];

constexpr int CS_ACC = 10;
constexpr int CS_GYR = 37;

void sendStrainCommand(uint8_t addr, uint16_t fcode, uint16_t reg,
                       uint16_t num);
// 传感器读取函数更新下方共享的测量值，调用方不持有硬件接口。
void Strain_read_all();
void getICM42688data();
void getBMI088data();
void printBMI088Raw(const char *tag, const uint8_t *buf, int len, int16_t x,
                    int16_t y, int16_t z);
void getairdata();
void getIMUdata_EXT();
void writeRegSPI(int cs, uint8_t reg, uint8_t val);

// MT6701 原始绝对角和归零后的相对角，均为 rad。
extern float relativeAngle_raw, relativeAngle_ready;
extern float airspeed_A;
// 统一测量接口：三轴顺序 X/Y/Z；角速度 rad/s、角加速度 rad/s²、加速度 m/s²。
// X 轴方向与原控制/日志使用方向一致；型号选择、安装轴映射在传感器层完成。
extern float ang_vel[3];
// 传感器层每次发布测量时统一滤波，三轴顺序 X/Y/Z，单位 rad/s。
extern float ang_vel_filtered[3];
extern float ang_acc[3];
extern float acc[3];
// 每周期在 IMU 采样/姿态更新后发布原始和滤波角速度，随后计算 ang_acc。
void updateSensorMeasurements();
// 完成本周期的 IMU 采样、姿态估计、测量发布和角加速度估计。
void updateInertialSensors();

void beginExternalImuLink();
void beginStrainSensorLink();
void loadImuCalibration();
bool imuCalibrationValid();
void initializeInitialAttitude();
void IMUinit();
void getIMUdata();
void calculate_IMU_error();
void getairspeed();
// 角加速度估计器的专用滤波初始化；由 initializeControlFilters() 调用。
void initializeAngularAccelerationFilters();
void getAngularACC();
void getbarodata();
void initBAROMETER();
void initializeRotateSensor();
bool rotateSensorValid();
void getRotateSensor1();
void ResetRotateSensor();
void loadRotateSensorOffset();
void calibrateAirspeedSensor();
void calibrateAttitude();
