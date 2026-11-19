#include "config/aircraft_config.h"
#include "system/flight_lock.h"
// 飞控入口：仅编排初始化与每周期任务；各模块的状态和实现留在对应模块中。

#include "actuators/actuator_output.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"
#include "sensors/filters.h"
#include "diagnostics/firmware_debug.h"
#include "diagnostics/debug_print.h"
#include "system/flight_clock.h"
#include "interface/human_interface.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "logging/logger.h"
#include "parameters/parameter_service.h"
#include "radio_comm/radio_comm.h"
#include "sensors/sensor_processing.h"
#include "sensors/sensor_calibration.h"
#include "telemetry/telemetry.h"
#include <Arduino.h>

#include "system/rtos_sync.h"
#include "system/device_reboot.h"
#include "system/flight_tasks.h"

namespace {
constexpr TickType_t controlPeriod = pdMS_TO_TICKS(2);
static_assert(configTICK_RATE_HZ == 1000 && controlPeriod == 2, "CoFly requires a 1 kHz tick");
static_assert(configMAX_PRIORITIES > 5, "CoFly requires five application priorities");

// The five persistent tasks use static OCRAM stacks. Depths are in StackType_t words.
DMAMEM StackType_t controlStack[2048], linksStack[1024], auxiliaryStack[1536];
DMAMEM StackType_t parameterStack[3072], storageStack[3072];
StaticTask_t controlTcb, linksTcb, auxiliaryTcb, parameterTcb, storageTcb;
TaskHandle_t controlHandle, linksHandle, auxiliaryHandle, parameterHandle, storageHandle;

// Advance to the latest elapsed release. xTaskDelayUntil then waits for
// the next future release, including when the tick counter wraps.
TickType_t skipElapsedReleases(TickType_t &release, TickType_t period) {
  const TickType_t missed = (xTaskGetTickCount() - release) / period;
  release += missed * period;
  return missed;
}

void controlTask(void *) {
  TickType_t release = xTaskGetTickCount();
  for (;;) {
    const uint32_t cycleStart = ARM_DWT_CYCCNT;
    updateFlightClock();
    latchAuxiliarySensors();
    const uint32_t imuStart = ARM_DWT_CYCCNT;
    updateInertialSensors();
    latchInteraircraftInputs();
    const bool enabled = aircraftConfigurationValid() && !parameterRebootRequired() && !deviceRebootPending();
    if (enabled) {
      runControlCycle();
      convertControlCommandsToPWM();
      applyActuatorCommands();
    } else {
      setFlightLocked(true);
      commandSafeActuatorPositions();
    }
    const uint32_t cyclesPerUs = F_CPU_ACTUAL / 1000000u;
    const uint32_t imuToPwm = (ARM_DWT_CYCCNT - imuStart) / cyclesPerUs;
    acknowledgeCalibrationLock();
    publishInteraircraftOutputs(enabled);
    captureFlightServices(enabled);
    if (enabled) latchRadioInput();
    pollSensorCalibration(true);

    // Consume missed releases without running extra iterations on old measurements.
    const TickType_t missed = skipElapsedReleases(release, controlPeriod);
    finishFlightCycle(imuToPwm, (ARM_DWT_CYCCNT - cycleStart) / cyclesPerUs, missed);
    xTaskDelayUntil(&release, controlPeriod);
  }
}

void linkTask(void *) {
  TickType_t release = xTaskGetTickCount();
  for (;;) {
    pollFlightLinks();
    skipElapsedReleases(release, 1);
    xTaskDelayUntil(&release, 1);
  }
}

void auxiliaryTask(void *) {
  TickType_t release = xTaskGetTickCount();
  for (;;) {
    getRotateSensor1();
    getairspeed();
    getbarodata();
    Strain_read_all();
    pollSensorCalibration(false);
    publishAuxiliarySensors();
    ProcessButtonState();
    serviceDisplayTransfer();
    displayAttitude();
    skipElapsedReleases(release, controlPeriod);
    xTaskDelayUntil(&release, controlPeriod);
  }
}

void storageTask(void *) {
  for (;;) serviceFlightLogger();
}

void parameterTask(void *) {
  for (;;) {
    serviceCalibrationParameters();
    pollParameterService();
    telemetry();
    runDebugPrints();
    vTaskDelay(1);
  }
}

void startupTask(void *) {
  // 建立串口链路，再初始化传感器、执行器和控制器。
  beginHumanInterfaceLinks();
  initializeFirmwareDebug();
  beginTelemetryLink();
  beginInteraircraftLinks();
  beginStrainSensorLink();
  beginExternalImuLink();

  delay(20);

  initializeHumanInterface();
  initializeLogger();
  displayfilenum();
  initializeParameterService(loggerSdReady());
  initializeCommandLinks();
  initializeSensorLinks();
  initializeAirDataBus();
  loadRotateSensorOffset();
  initializeRotateSensor();
  loadImuCalibration();
  displayAircraftIdentity();
  attachActuators();
  delay(5);
  radioSetup();
  IMUinit();

  delay(5);
  commandSafeActuatorPositions();

  delay(5);
  calibrateAirspeedSensor();
  initializeInitialAttitude();
  initializeInitialControlMode();
  initializeControlFilters();
  publishAuxiliarySensors();
  latchAuxiliarySensors();
  startFlightStorage();

  // No worker can run until all handles and the safe initial state are installed.
  vTaskSuspendAll();
  storageHandle = xTaskCreateStatic(storageTask, "Storage", 3072, nullptr, 1, storageStack, &storageTcb);
  parameterHandle = xTaskCreateStatic(parameterTask, "Parameters", 3072, nullptr, 2, parameterStack, &parameterTcb);
  auxiliaryHandle = xTaskCreateStatic(auxiliaryTask, "Aux", 1536, nullptr, 3, auxiliaryStack, &auxiliaryTcb);
  linksHandle = xTaskCreateStatic(linkTask, "Links", 1024, nullptr, 4, linksStack, &linksTcb);
  controlHandle = xTaskCreateStatic(controlTask, "Control", 2048, nullptr, 5, controlStack, &controlTcb);
  configASSERT(storageHandle && parameterHandle && auxiliaryHandle && linksHandle && controlHandle);
  xTaskResumeAll();
  vTaskDelete(nullptr); // The idle task reclaims the one-shot startup stack.
}
} // namespace

FlightStackMargins flightStackMargins() {
  return {uxTaskGetStackHighWaterMark(controlHandle), uxTaskGetStackHighWaterMark(linksHandle),
          uxTaskGetStackHighWaterMark(auxiliaryHandle), uxTaskGetStackHighWaterMark(parameterHandle),
          uxTaskGetStackHighWaterMark(storageHandle)};
}

void setup() {
  const BaseType_t created = xTaskCreate(startupTask, "Startup", 3072, nullptr, 2, nullptr);
  configASSERT(created == pdPASS);
  if (created == pdPASS) vTaskStartScheduler();
  for (;;) {} // Stay here if startup task creation or scheduler startup fails.
}

void loop() {}
