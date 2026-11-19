#include "radio_comm/radio_comm.h"
#include "control/control_output.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"
#include "parameters/control_parameters.h"
#include "logging/logger.h"
#include "ground_station_service/sd_file_service.h"
#include "actuators/actuator_output.h"
#include "config/serial_config.h"
#include "control/command_generation.h"
#include "system/flight_clock.h"
#include "sensors/sensor_processing.h"
#include "logging/flight_service_sample.h"
#include "logging/flight_storage.h"
#include "system/flight_lock.h"
#include "system/device_reboot.h"
#include "system/rtos_sync.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <atomic>
#include <SD.h>

using ControlParameters::integralControlEnabled;
using ControlParameters::configurationControlScale;

namespace
{

  // SD 卡日志状态。
  char filename_sd[20];
  FsFile dataFile;
  bool sdReady = false;
  unsigned int fileNumber = 0;
  constexpr unsigned logfreq = 50;
  unsigned long lastLogTime = 0;
  unsigned long fileCycle = 0;
  String dataString = "";
  bool headerWritten = false;
  std::atomic<bool> logFileOpen{false};
  std::atomic<uint32_t> writeFailures{0};
  bool storageStarted = false;
  std::atomic<TaskHandle_t> storageTask{nullptr};
  FlightServiceSample latestSample{};
  struct StorageRequest {
    StorageOperation operation;
    void *context;
    bool result;
  };
  constexpr unsigned logDepth = 8;
  StaticQueue_t logQueueState, requestQueueState;
  alignas(FlightServiceSample) uint8_t logBuffer[logDepth * sizeof(FlightServiceSample)];
  alignas(StorageRequest *) uint8_t requestBuffer[sizeof(StorageRequest *)];
  QueueHandle_t logQueue, requestQueue;
  StaticSemaphore_t wakeState, completionState;
  SemaphoreHandle_t wakeStorage, requestCompleted;
  std::atomic<uint32_t> drops{0};
  void writeSample(const FlightServiceSample &sample);
  void stopFailedLog() {
    ++writeFailures;
    logFileOpen.store(false);
    dataFile.close();
  }

  void drainLogQueue() {
    FlightServiceSample sample;
    for (unsigned i = 0; i < logDepth && xQueueReceive(logQueue, &sample, 0) == pdTRUE; ++i)
      writeSample(sample);
  }

  // 扫描 SD 卡中 datalogNNN.csv 文件，选择下一个 1～999 的日志编号。
  void findMaxFileNumber()
  {
    File root = SD.open("/");
    while (true)
    {
      File entry = root.openNextFile();
      if (!entry)
        break;

      String name = entry.name();
      if (name.startsWith("datalog") && name.endsWith(".csv"))
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
  logQueue = xQueueCreateStatic(logDepth, sizeof(FlightServiceSample), logBuffer, &logQueueState);
  requestQueue = xQueueCreateStatic(1, sizeof(StorageRequest *), requestBuffer, &requestQueueState);
  wakeStorage = xSemaphoreCreateBinaryStatic(&wakeState);
  requestCompleted = xSemaphoreCreateBinaryStatic(&completionState);
  configASSERT(logQueue && requestQueue && wakeStorage && requestCompleted);
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
  sprintf(filename_sd, "datalog%03d.csv", fileNumber);
  dataFile = SD.sdfs.open(filename_sd, O_RDWR | O_CREAT | O_AT_END);
  logFileOpen = static_cast<bool>(dataFile);
  if (!logFileOpen)
  {
    ++writeFailures;
    USBSerial.println("Error opening datalog.csv");
  }
  // delay(10);
  USBSerial.print("新建日志文件：");
  USBSerial.println(filename_sd);
}

bool loggerSdReady() { return sdReady; }
void flushLogger()
{
  runStorageOperation([](void *) {
    drainLogQueue();
    if (logFileOpen && !dataFile.sync()) stopFailedLog();
    return true;
  }, nullptr);
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

  void buildLoggerRecord(const FlightServiceSample &sample, bool header) {
    CsvLogWriter writer(header);
#define LOG_FIELD(value) writer.field(#value, sample.value)
#define LOG_ARRAY(value) writer.array(#value, sample.value)

    // 1. 本周期时间戳（us）与循环间隔（s）。
    LOG_FIELD(current_time);
    LOG_FIELD(dt);

    // 2. 本机传感器发布值和滤波值；不读取其他飞机的传感器状态。
    LOG_ARRAY(quat);
    LOG_ARRAY(att);
    LOG_ARRAY(ang_vel);
    LOG_ARRAY(ang_vel_filtered);
    LOG_ARRAY(ang_acc);
    LOG_ARRAY(acc);
    LOG_FIELD(airspeed);
    LOG_ARRAY(strain_value);
    LOG_FIELD(relativeAngle_raw);
    LOG_FIELD(relativeAngle_ready);
    writer.field("rotateSensorValid()", sample.rotateValid);

    // 3. 控制实际使用的八路归一化遥控指令，范围 [-1, 1]。
    writer.field("radio_channel1_normalized", sample.radioNormalized[0]);
    writer.field("radio_channel2_normalized", sample.radioNormalized[1]);
    writer.field("radio_channel3_normalized", sample.radioNormalized[2]);
    writer.field("radio_channel4_normalized", sample.radioNormalized[3]);
    writer.field("radio_channel5_normalized", sample.radioNormalized[4]);
    writer.field("radio_channel6_normalized", sample.radioNormalized[5]);
    writer.field("radio_channel7_normalized", sample.radioNormalized[6]);
    writer.field("radio_channel8_normalized", sample.radioNormalized[7]);

    // 4. 本机最终物理 PWM，单位 us。
    LOG_ARRAY(ActuatorOutput);

    // 5. 模式变量和构型控制输出系数参数。
    LOG_FIELD(currentFlightMode);
    LOG_FIELD(previousFlightMode);
    LOG_FIELD(activePitchController);
    LOG_FIELD(previousPitchController);
    LOG_FIELD(mixerStrategyCommand);
    LOG_FIELD(configurationControlScale);
    LOG_FIELD(configurationAngle);
    LOG_FIELD(brakeEnabled);
    LOG_FIELD(controlModeChanged);


    // 6. 主机遥控直通输入与物理目标；targets 的单位不随模式变化。
    LOG_ARRAY(manual_input);
    LOG_FIELD(targets.roll);
    LOG_FIELD(targets.pitch);
    LOG_FIELD(targets.yawRate);
    LOG_FIELD(targets.throttle);
    LOG_ARRAY(targets.configuration);

    // 7. 各控制器输出，顺序与 control_output.h 的声明一致。
    LOG_FIELD(rollControlOutput.pidOutput);
    LOG_FIELD(rollControlOutput.filteredPidOutput);
    LOG_FIELD(rollControlOutput.filteredPidOutputDerivative);
    LOG_FIELD(rollControlOutput.equivalentRollAngle);
    LOG_FIELD(rollControlOutput.rateTarget);
    LOG_FIELD(rollControlOutput.equivalentRollRate);
    LOG_FIELD(pitchPidControlOutput.pidOutput);
    LOG_FIELD(pitchPidControlOutput.rateTarget);
    LOG_FIELD(pitchIndiControlOutput.q_dot_ref);
    LOG_FIELD(pitchIndiControlOutput.q_ref);
    LOG_FIELD(pitchIndiControlOutput.q_f);
    LOG_FIELD(pitchIndiControlOutput.isValid);
    LOG_FIELD(pitchIndiControlOutput.indiOutput);
    LOG_FIELD(pitchIndiControlOutput.q_dot_meas);
    LOG_FIELD(pitchIndiControlOutput.delta_e_cmd);
    LOG_FIELD(pitchIndiControlOutput.delta_e_hat);
    LOG_FIELD(rollDampingControlOutput.dampingCommand);
    LOG_FIELD(rollDampingControlOutput.rateTarget);
    LOG_FIELD(yawRateControlOutput.pidOutput);
    LOG_FIELD(yawRateControlOutput.proportionalTerm);
    LOG_FIELD(yawRateControlOutput.integralTerm);
    LOG_FIELD(yawRateControlOutput.derivativeTerm);
    LOG_FIELD(yawRateControlOutput.feedforwardTerm);
    LOG_FIELD(yawRateControlOutput.rateTarget);
    LOG_FIELD(throttleControlOutput.throttleCommand);
    LOG_FIELD(throttleControlOutput.airspeedTarget);
    LOG_FIELD(throttleControlOutput.filteredAirspeed);
    LOG_FIELD(throttleControlOutput.airspeedError);
    LOG_FIELD(throttleControlOutput.integralTerm);
    LOG_FIELD(throttleControlOutput.speedControlActive);
    LOG_ARRAY(configurationControlOutput.pidOutputs);
    LOG_ARRAY(configurationControlOutput.angleErrors);
    LOG_ARRAY(configurationControlOutput.measuredAngles);
    LOG_ARRAY(configurationControlOutput.measuredRates);
    LOG_ARRAY(configurationControlOutput.rateTargets);

    // 8. 本机混控/融合后的逻辑指令，保留 PWM 映射前的实际值。
    writer.field("commands.local.aileron1", sample.local.aileron1);
    writer.field("commands.local.aileron2", sample.local.aileron2);
    writer.field("commands.local.elevator", sample.local.elevator);
    writer.field("commands.local.throttle", sample.local.throttle);
    writer.field("commands.local.rudder", sample.local.rudder);

#undef LOG_ARRAY
#undef LOG_FIELD
  }

  bool appendCsvLine() {
    const uint64_t start = dataFile.curPosition();
    if (dataFile.println(dataString.c_str()) == dataString.length() + 2 && !dataFile.getWriteError())
      return true;
    // A short write must not leave half a row followed by a valid-looking record.
    dataFile.truncate(start);
    stopFailedLog();
    return false;
  }

  bool writeLoggerHeader(const FlightServiceSample &sample) {
    if (headerWritten) return true;
    buildLoggerRecord(sample, true);
    if (!appendCsvLine()) return false;
    if (!dataFile.sync()) {
      stopFailedLog();
      return false;
    }
    headerWritten = true;
    return true;
  }
} // namespace

void copyFlightServiceSample(FlightServiceSample &destination) {
  TaskCriticalSection guard;
  destination = latestSample;
}

void captureFlightServices(bool logEnabled) {
  FlightServiceSample &s = latestSample;
  {
    // Only fixed-size copies run under the guard. No driver access or formatting.
    TaskCriticalSection guard;
    s.current_time = current_time;
    s.sampledMs = millis();
    s.dt = dt;
    memcpy(s.quat, quat, sizeof(s.quat));
    memcpy(s.att, att, sizeof(s.att));
    memcpy(s.ang_vel, ang_vel, sizeof(s.ang_vel));
    memcpy(s.ang_vel_filtered, ang_vel_filtered, sizeof(s.ang_vel_filtered));
    memcpy(s.ang_acc, ang_acc, sizeof(s.ang_acc));
    memcpy(s.acc, acc, sizeof(s.acc));
    memcpy(s.strain_value, strain_value, sizeof(s.strain_value));
    memcpy(s.ActuatorOutput, ActuatorOutput, sizeof(s.ActuatorOutput));
    memcpy(s.manual_input, manual_input, sizeof(s.manual_input));
    s.airspeed = airspeed;
    s.relativeAngle_raw = relativeAngle_raw;
    s.relativeAngle_ready = relativeAngle_ready;
    s.currentFlightMode = currentFlightMode;
    s.previousFlightMode = previousFlightMode;
    s.activePitchController = activePitchController;
    s.previousPitchController = previousPitchController;
    s.mixerStrategyCommand = mixerStrategyCommand;
    s.configurationControlScale = configurationControlScale;
    s.configurationAngle = configurationAngle;
    s.brakeEnabled = brakeEnabled;
    s.controlModeChanged = controlModeChanged;
    s.integralControlEnabled = integralControlEnabled;
    s.targets = targets;
    s.rollControlOutput = rollControlOutput;
    s.pitchPidControlOutput = pitchPidControlOutput;
    s.pitchIndiControlOutput = pitchIndiControlOutput;
    s.rollDampingControlOutput = rollDampingControlOutput;
    s.yawRateControlOutput = yawRateControlOutput;
    s.throttleControlOutput = throttleControlOutput;
    s.configurationControlOutput = configurationControlOutput;
    s.local = commands.local;
    s.rotateValid = rotateSensorValid();
    s.locked = isFlightLocked();
    s.radioValid = radioInputValid();
    s.leftFresh = isSensorDataFresh(true);
    s.rightFresh = isSensorDataFresh(false);
    s.airspeedStatus = airspeedDiagnostics();
    const int raw[] = {radio_channel1_raw, radio_channel2_raw, radio_channel3_raw, radio_channel4_raw,
                       radio_channel5_raw, radio_channel6_raw, radio_channel7_raw, radio_channel8_raw};
    const float normalized[] = {radio_channel1_normalized, radio_channel2_normalized, radio_channel3_normalized, radio_channel4_normalized,
                                radio_channel5_normalized, radio_channel6_normalized, radio_channel7_normalized, radio_channel8_normalized};
    memcpy(s.radioRaw, raw, sizeof(raw));
    memcpy(s.radioNormalized, normalized, sizeof(normalized));
    const float *const attitudes[] = {att_B, att_C, att_D, att_E, att_F, att_G};
    const float rates[] = {ang_vel_B[0], ang_vel_C[0], ang_vel_D[0], ang_vel_E[0], ang_vel_F[0], ang_vel_G[0]};
    for (unsigned i = 0; i < 6; ++i) memcpy(s.wingAttitude[i], attitudes[i], sizeof(s.wingAttitude[i]));
    memcpy(s.wingRollRate, rates, sizeof(rates));
  }
  if (!logEnabled || !logFileOpen || sdFileSessionActive() || deviceRebootPending()) return;
  if (s.current_time - lastLogTime < 1000000UL / logfreq) return;
  lastLogTime = s.current_time;
  if (xQueueSend(logQueue, &s, 0) == pdTRUE)
    xSemaphoreGive(wakeStorage);
  else
    ++drops;
}

uint32_t droppedFlightLogSamples() { return drops.load(); }
uint32_t flightLogWriteFailures() { return writeFailures.load(); }

void startFlightStorage() { storageStarted = true; }

bool runStorageOperation(StorageOperation operation, void *context) {
  if (!storageStarted) return operation(context);
  // Nested operations (e.g. OPEN -> flushLogger) execute on the owner.
  if (xTaskGetCurrentTaskHandle() == storageTask.load()) return operation(context);
  StorageRequest request{operation, context, false};
  StorageRequest *pointer = &request;
  // The parameter task is the sole synchronous client.
  xQueueSend(requestQueue, &pointer, portMAX_DELAY);
  xSemaphoreGive(wakeStorage);
  xSemaphoreTake(requestCompleted, portMAX_DELAY);
  return request.result;
}


namespace {
void writeSample(const FlightServiceSample &sample) {
  if (!logFileOpen || !writeLoggerHeader(sample)) return;
  buildLoggerRecord(sample, false);
  if (!appendCsvLine()) return;
  if (millis() - fileCycle > 2000) {
    if (!dataFile.sync()) stopFailedLog();
    fileCycle = millis();
  }
}
}

void serviceFlightLogger() {
  storageTask.store(xTaskGetCurrentTaskHandle());
  xSemaphoreTake(wakeStorage, portMAX_DELAY);
  // Serve maintenance first. OPEN and REBOOT explicitly drain accepted rows.
  StorageRequest *request;
  if (xQueueReceive(requestQueue, &request, 0) == pdTRUE) {
    request->result = request->operation(request->context);
    xSemaphoreGive(requestCompleted);
  }
  FlightServiceSample sample;
  if (xQueueReceive(logQueue, &sample, 0) == pdTRUE) writeSample(sample);
  if (uxQueueMessagesWaiting(logQueue)) xSemaphoreGive(wakeStorage);
}
