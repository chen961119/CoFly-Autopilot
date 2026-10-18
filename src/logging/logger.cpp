#include "radio_comm/radio_comm.h"
#include "control/control_output.h"
#include "control/control_modes.h"
#include "parameters/control_parameters.h"
#include "logging/logger.h"
#include "actuators/actuator_output.h"
#include "config/serial_config.h"
#include "control/command_generation.h"
#include "system/flight_clock.h"
#include "sensors/sensor_processing.h"
#include <SD.h>

using ControlParameters::integralControlEnabled;

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
  // 表头和数据共用同一份字段清单，避免列名与数值错位。
  class CsvLogWriter {
  public:
    explicit CsvLogWriter(bool header) : header(header) { dataString = ""; }

    template <typename T>
    void field(const char *name, T value) {
      if (!first) dataString += ',';
      first = false;
      if (header) dataString += name;
      else appendValue(value);
    }

    template <typename T, size_t N>
    void array(const char *name, const T (&values)[N]) {
      for (size_t i = 0; i < N; ++i) {
        if (header) {
          const String column = String(name) + '[' + String(i) + ']';
          field(column.c_str(), values[i]);
        } else {
          field(nullptr, values[i]);
        }
      }
    }

  private:
    template <typename T>
    void appendValue(T value) { dataString += String(value); }
    void appendValue(float value) { dataString += String(value, 6); }
    void appendValue(PitchController value) { dataString += String(static_cast<int>(value)); }

    bool header;
    bool first = true;
  };

  void buildLoggerRecord(bool header) {
    CsvLogWriter writer(header);
#define LOG_FIELD(value) writer.field(#value, value)
#define LOG_ARRAY(value) writer.array(#value, value)

    // 1. 本周期时间戳（us）与循环间隔（s）。
    LOG_FIELD(current_time);
    LOG_FIELD(dt);
    // Identify SI physical columns in logs that share names with older firmware.
    // Time and PWM units retain their original definitions.
    writer.field("si_units_version", 1);

    // 2. 本机传感器发布值和滤波值；不读取其他飞机的传感器状态。
    LOG_ARRAY(quat);
    LOG_ARRAY(att);
    LOG_ARRAY(ang_vel);
    LOG_ARRAY(ang_vel_filtered);
    LOG_ARRAY(ang_acc);
    LOG_ARRAY(acc);
    LOG_FIELD(airspeed);
    LOG_ARRAY(strain_value);
    LOG_FIELD(airdata.aoa);
    LOG_FIELD(airdata.aos);
    LOG_FIELD(airdata.tas);
    LOG_FIELD(airdata.statA);
    LOG_FIELD(airdata.statS);
    LOG_FIELD(relativeAngle_raw);
    LOG_FIELD(relativeAngle_ready);
    LOG_FIELD(rotateSensorValid());

    // 3. 控制实际使用的八路归一化遥控指令，范围 [-1, 1]。
    LOG_FIELD(radio_channel1_normalized);
    LOG_FIELD(radio_channel2_normalized);
    LOG_FIELD(radio_channel3_normalized);
    LOG_FIELD(radio_channel4_normalized);
    LOG_FIELD(radio_channel5_normalized);
    LOG_FIELD(radio_channel6_normalized);
    LOG_FIELD(radio_channel7_normalized);
    LOG_FIELD(radio_channel8_normalized);

    // 4. 本机最终物理 PWM，单位 us。
    LOG_ARRAY(ActuatorOutput);

    // 5. initializeInitialControlMode() 初始化的全部模式变量。
    LOG_FIELD(currentFlightMode);
    LOG_FIELD(previousFlightMode);
    LOG_FIELD(activePitchController);
    LOG_FIELD(previousPitchController);
    LOG_FIELD(mixerStrategyCommand);
    LOG_FIELD(configurationControlScale);
    LOG_FIELD(configurationAngle);
    LOG_FIELD(brakeEnabled);
    LOG_FIELD(controlModeChanged);


    // 6. 直接记录 targets 原值，手动/增稳模式的尺度约定见 ControlTargets。
    LOG_FIELD(targets.roll);
    LOG_FIELD(targets.pitch);
    LOG_FIELD(targets.yawRate);
    LOG_FIELD(targets.throttle);
    LOG_ARRAY(targets.configuration);

    // 7. 各控制器输出，顺序与 control_output.h 的声明一致。
    LOG_FIELD(manualControlOutput.rollCommand);
    LOG_FIELD(manualControlOutput.pitchCommand);
    LOG_FIELD(manualControlOutput.yawCommand);
    LOG_FIELD(manualControlOutput.throttleCommand);
    LOG_FIELD(rollControlOutput.pidOutput);
    LOG_FIELD(rollControlOutput.filteredPidOutput);
    LOG_FIELD(rollControlOutput.filteredPidOutputDerivative);
    LOG_FIELD(rollControlOutput.equivalentRollAngle);
    LOG_FIELD(pitchPidControlOutput.pidOutput);
    LOG_FIELD(pitchIndiControlOutput.q_dot_ref);
    LOG_FIELD(pitchIndiControlOutput.q_ref);
    LOG_FIELD(pitchIndiControlOutput.q_f);
    LOG_FIELD(pitchIndiControlOutput.isValid);
    LOG_FIELD(pitchIndiControlOutput.indiOutput);
    LOG_FIELD(pitchIndiControlOutput.q_dot_meas);
    LOG_FIELD(pitchIndiControlOutput.delta_e_cmd);
    LOG_FIELD(pitchIndiControlOutput.delta_e_hat);
    LOG_FIELD(rollDampingControlOutput.dampingCommand);
    LOG_FIELD(yawRateControlOutput.pidOutput);
    LOG_FIELD(throttleControlOutput.throttleCommand);
    LOG_ARRAY(configurationControlOutput.pidOutputs);
    LOG_ARRAY(configurationControlOutput.angleErrors);
    LOG_ARRAY(configurationControlOutput.measuredAngles);
    LOG_ARRAY(configurationControlOutput.measuredRates);

#undef LOG_ARRAY
#undef LOG_FIELD
  }

  void writeLoggerHeader() {
    if (headerWritten || !dataFile) return;
    buildLoggerRecord(true);
    dataFile.println(dataString);
    dataFile.flush();
    headerWritten = true;
  }
} // namespace

// 单机、编队和主从角色使用同一日志格式。
void logFlightData() {
  if (!sdReady || !dataFile) return;
  const unsigned long now = micros();
  if (now - lastLogTime < 1000000UL / logfreq) return;
  lastLogTime = now;
  writeLoggerHeader();
  buildLoggerRecord(false);
  dataFile.println(dataString);
  if (millis() - fileCycle > 2000) {
    dataFile.flush();
    fileCycle = millis();
  }
}
