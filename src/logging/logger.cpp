#include "radio_comm/radio_comm.h"
#include "control/control_output.h"
#include "config/aircraft_config.h"
#include "logging/logger.h"
#include "actuators/actuator_output.h"
#include "config/serial_config.h"
#include "control/command_generation.h"
#include "control/control_allocation.h"
#include "system/flight_clock.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "sensors/sensor_processing.h"
#include <SD.h>

namespace
{

  // SD 卡日志状态。
  char filename_sd[20];
  File dataFile;
  bool sdReady = false;
  unsigned int fileNumber = 0;
  unsigned int logfreq = 50; // 记录频率Hz 注意要是2000的因数。
  unsigned long lastLogTime = 0;
  unsigned long fileCycle = 0;
  String dataString = "";
  bool headerWritten = false;

  // 扫描 SD 卡中 datalogNNN.txt 文件，选择下一个 1～999 的日志编号。
  void findMaxFileNumber()
  {
    File root = SD.open("/");
    while (true)
    {
      File entry = root.openNextFile();
      if (!entry)
        break;

      String name = entry.name();
      if (name.startsWith("datalog") && name.endsWith(".txt"))
      {
        // 提取编号部分
        String numStr = name.substring(7, 10); // 从第8个字符开始取3位

        // 验证是否为纯数字
        bool valid = true;
        for (uint8_t i = 0; i < 3; i++)
        {
          if (!isDigit(numStr.charAt(i)))
          {
            valid = false;
            break;
          }
        }

        if (valid)
        {
          uint16_t num = numStr.toInt();
          if (num > fileNumber)
          {
            fileNumber = num;
          }
        }
      }
      entry.close();
    }
    root.close();

    fileNumber++;                               // 新文件编号递增
    fileNumber = constrain(fileNumber, 1, 999); // 限制在1-999之间
  }

} // namespace

// 初始化 SD 卡，并创建带表头的新日志文件。
void initializeLogger()
{
  headerWritten = false;
  // Some cards need time to return to their startup state after a warm reset.
  // Retry only during setup; never stall the flight loop trying to remount SD.
  for (unsigned attempt = 0; attempt < 3; ++attempt)
  {
    if (attempt)
    {
      SD.sdfs.end();
      delay(100);
    }
    sdReady = SD.begin(BUILTIN_SDCARD);
    if (sdReady)
      break;
  }
  if (!sdReady)
  {
    USBSerial.println("Card failed, or not present");
    return;
  }
  else
  {
    USBSerial.println("card initialized.");
  }
  findMaxFileNumber();
  sprintf(filename_sd, "datalog%03d.txt", fileNumber);
  dataFile = SD.open(filename_sd, FILE_WRITE);
  if (!dataFile)
  {
    USBSerial.println("Error opening datalog.txt");
  }
  delay(10);
  USBSerial.print("新建日志文件：");
  USBSerial.println(filename_sd);
}

bool loggerSdReady() { return sdReady; }
void flushLogger()
{
  if (dataFile)
    dataFile.flush();
}

unsigned int loggerFileNumber() { return fileNumber; }

namespace
{
  void writeLoggerHeader()
  {
    if (headerWritten || !dataFile)
      return;
    if (aircraftIsSingle())
    {
      dataFile.println("TimeStamp(us),quat[0],quat[1],quat[2],quat[3],att[0](deg),att[1](deg),att[2](deg),ROLL_des(deg),PITCH_des_local(deg),YAW_des(deg),RC_CH1,RC_CH2,RC_CH3,RC_CH4,RC_CH5,RC_CH6,RC_CH7,RC_CH8,ActuatorOutput[0],ActuatorOutput[1],ActuatorOutput[2],ActuatorOutput[3],ActuatorOutput[4],ang_vel[0],ang_vel[1],ang_vel[2],acc[0],acc[1],acc[2],airspeed_A,ang_acc[0],ang_acc[1],ang_acc[2],indi_pitch_q_des_log,indi_pitch_q_filt_log,indi_pitch_dq_des_log,indi_pitch_dq_used_log,indi_pitch_delta_e_cmd_deg_log,indi_pitch_delta_e_est_deg_log,MT6701_raw(deg),MT6701_relative(deg),MT6701_valid");
      dataFile.flush();
    }
    else
    {
      dataFile.println(String(
          "TimeStamp(us),att[0](deg),ROLL_Eq(deg),att[1](deg),att[2](deg),"
          "ROLL_des(deg),PITCH_des_local(deg),YAW_des(deg),RC_CH1,RC_CH2,RC_CH3,"
          "RC_CH4,RC_CH5,RC_CH6,RC_CH7,RC_CH8,ActuatorOutput[0],ActuatorOutput[1],ActuatorOutput[2],"
          "ActuatorOutput[3],ActuatorOutput[4],"
          ""
          ""
          "Pab(deg),Pac(deg),Pbd("
          "deg),Pce(deg),Pdf(deg),Peg(deg),Phiab_Mea(deg),Phiab_des(deg),"
          "phiac(deg),phibd(deg),phice(deg),phidf(deg),phieg(deg),Apitchsp,"
          "Bpitchsp,Cpitchsp,Dpitchsp,Epitchsp,Fpitchsp,Gpitchsp,"
          "att_B[1],att_C[1],att_D[1],att_E[1],att_F[1],att_G[1],acc[0],acc[1],acc[2],"
          "ang_vel[0],ang_vel[1],ang_vel[2],phiab_PID,phiac_PID,"
          "phibd_PID,phice_PID,phidf_PID,phieg_PID,roll_PID,pitch_PID,airspeed_A,"
          "strain_value[0],strain_value[1],strain_value[2],strain_value[3],strain_value[4],"
          "AOA,AOS,TAS,ang_acc[0],ang_acc[1],ang_acc[2],INDI_q_des,INDI_q_filt,INDI_dq_des,INDI_dq_used,"
          "INDI_delta_e_cmd_deg,INDI_delta_e_est_deg,"
          "MT6701_raw(deg),MT6701_relative(deg),MT6701_valid"));
      dataFile.flush();
    }
    headerWritten = true;
  }
} // namespace

// 单机日志使用本机 IMU、执行器与 INDI 数据，不读取相邻机状态。
void loggerSINGLE()
{
  if (!aircraftIsSingle() || !sdReady || !dataFile)
    return;
  const unsigned long now = micros();
  if (now - lastLogTime < 1000000UL / logfreq)
    return;
  lastLogTime = now;
  writeLoggerHeader();
  dataString =
      String(current_time) + "," +
      String(quat[0]) + "," +
      String(quat[1]) + "," +
      String(quat[2]) + "," +
      String(quat[3]) + "," +
      String(att[0]) + "," +
      String(att[1]) + "," +
      String(att[2]) + "," +
      String(targetRollDegrees()) + "," +
      String(targetPitchDegrees()) + "," +
      String(targetYawDegrees()) + "," +
      String(radio_channel1_normalized, 6) + "," +
      String(radio_channel2_normalized, 6) + "," +
      String(radio_channel3_normalized, 6) + "," +
      String(radio_channel4_normalized, 6) + "," +
      String(radio_channel5_normalized, 6) + "," +
      String(radio_channel6_normalized, 6) + "," +
      String(radio_channel7_normalized, 6) + "," +
      String(radio_channel8_normalized, 6) + "," +
      String(ActuatorOutput[0]) + "," +
      String(ActuatorOutput[1]) + "," +
      String(ActuatorOutput[2]) + "," +
      String(ActuatorOutput[3]) + "," +
      String(ActuatorOutput[4]) + "," +
      String(ang_vel[0]) + "," +
      String(ang_vel[1]) + "," +
      String(ang_vel[2]) + "," +
      String(acc[0]) + "," +
      String(acc[1]) + "," +
      String(acc[2]) + "," +
      String(airspeed_A) + "," +
      String(ang_acc[0]) + "," +
      String(ang_acc[1]) + "," +
      String(ang_acc[2]) + "," +
      String(pitchIndiControlOutput.diagnostics.desiredRate) + "," +
      String(pitchIndiControlOutput.diagnostics.filteredRate) + "," +
      String(pitchIndiControlOutput.diagnostics.desiredAcceleration) + "," +
      String(pitchIndiControlOutput.diagnostics.usedAcceleration) + "," +
      String(pitchIndiControlOutput.diagnostics.commandedDeflection) + "," +
      String(pitchIndiControlOutput.diagnostics.estimatedDeflection) + "," +
      String(relativeAngle_raw) + "," +
      String(relativeAngle_ready) + "," +
      String(rotateSensorValid() ? 1 : 0);
  dataFile.println(dataString);
  if (millis() - fileCycle > 2000)
  {
    dataFile.flush();
    fileCycle = millis();
  }
}

// 按日志频率将编队传感器、控制和通信状态写入 SD 文件。
void loggerTEAM()
{
  if (aircraftIsSingle() || !sdReady || !dataFile)
    return;
  writeLoggerHeader();
  float invFreq = 1.0 / logfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastLogTime < invFreq)
    return;
  lastLogTime = checker;

  const float pitchTargetLog = targetPitchDegrees();
  // dataFile = SD.open(filename_sd, FILE_WRITE);
  // 光打开不close 拔电就没了，但是每次都打开又close很浪费时间。
  // 将本次采样的传感器与控制状态拼接为日志记录。
  dataString =
      String(current_time) + "," + String(att[0]) + "," + String(rollControlOutput.equivalentRoll) +
      "," + String(att[1]) + "," + String(att[2]) + "," + String(targetRollDegrees()) +
      "," + String(pitchTargetLog) + "," + String(targetYawDegrees()) + "," +
      String(radio_channel1_normalized, 6) + "," + String(radio_channel2_normalized, 6) + "," +
      String(radio_channel3_normalized, 6) + "," + String(radio_channel4_normalized, 6) + "," +
      String(radio_channel5_normalized, 6) + "," + String(radio_channel6_normalized, 6) + "," +
      String(radio_channel7_normalized, 6) + "," + String(radio_channel8_normalized, 6) + "," +
      String(ActuatorOutput[0]) + "," +
      String(ActuatorOutput[1]) + "," +
      String(ActuatorOutput[2]) + "," + String(ActuatorOutput[3]) + "," +
      String(ActuatorOutput[4]) + "," + String(configurationControlOutput.rates[0]) +
      "," + String(configurationControlOutput.rates[1]) + "," + String(configurationControlOutput.rates[2]) + "," + String(configurationControlOutput.rates[3]) + "," +
      String(configurationControlOutput.rates[4]) + "," + String(configurationControlOutput.rates[5]) + "," + String(configurationControlOutput.angles[0]) + "," +
      String(targets.configuration[0]) + "," + String(configurationControlOutput.angles[1]) + "," +
      String(configurationControlOutput.angles[2]) + "," + String(configurationControlOutput.angles[3]) + "," +
      String(configurationControlOutput.angles[4]) + "," + String(configurationControlOutput.angles[5]) + "," +
      String(pitchTargetLog) + "," + String(slaveTargets[0].pitch) + "," + String(slaveTargets[1].pitch) +
      "," + String(slaveTargets[2].pitch) + "," + String(slaveTargets[3].pitch) + "," +
      String(slaveTargets[4].pitch) + "," + String(slaveTargets[5].pitch) + "," + String(att_B[1]) +
      "," + String(att_C[1]) + "," + String(att_D[1]) + "," +
      String(att_E[1]) + "," + String(att_F[1]) + "," + String(att_G[1]) +
      "," +
      String(acc[0]) + "," + String(acc[1]) +
      "," + String(acc[2]) + "," + String(ang_vel[0]) + "," +
      String(ang_vel[1]) + "," + String(ang_vel[2]) + "," +
      String(configurationControlOutput.commands[0]) + "," + String(configurationControlOutput.commands[1]) + "," + String(configurationControlOutput.commands[2]) +
      "," + String(configurationControlOutput.commands[3]) + "," + String(configurationControlOutput.commands[4]) + "," +
      String(configurationControlOutput.commands[5]) + "," + String(rollControlOutput.command) + "," + String(pitchPidControlOutput.command) +
      "," + String(airspeed_A) + "," + String(strain_value[0]) + "," +
      String(strain_value[1]) + "," + String(strain_value[2]) + "," +
      String(strain_value[3]) + "," + String(strain_value[4]) + "," +
      String(airdata.aoa) + "," + String(airdata.aos) + "," +
      String(airdata.tas) + "," + String(ang_acc[0]) + "," + String(ang_acc[1]) + "," +
      String(ang_acc[2]) + "," + String(pitchIndiControlOutput.diagnostics.desiredRate) + "," +
      String(pitchIndiControlOutput.diagnostics.filteredRate) + "," + String(pitchIndiControlOutput.diagnostics.desiredAcceleration) +
      "," + String(pitchIndiControlOutput.diagnostics.usedAcceleration) + "," +
      String(pitchIndiControlOutput.diagnostics.commandedDeflection) + "," +
      String(pitchIndiControlOutput.diagnostics.estimatedDeflection) + "," +
      String(relativeAngle_raw) + "," + String(relativeAngle_ready) + "," +
      String(rotateSensorValid() ? 1 : 0);
  dataFile.println(dataString);

  // 文件可用时写入本次记录。

  // 定期刷新 SD 写入缓存。

  if (millis() - fileCycle > 2000)
  { // 每2秒刷新缓存
    dataFile.flush();

    fileCycle = millis();
  }
}
