#include "control/control_modes.h"
#include "config/aircraft_config.h"
#include "ground_station_service/ground_station_service.h"
#include "sensors/filters.h"
#include "system/flight_lock.h"
#include "system/device_reboot.h"
#include "sensors/sensor_processing.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "radio_comm/radio_comm.h"
#include "parameters/parameter_service.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

GroundStationResult requestGroundCalibration(const char *) { return GroundStationResult::NotImplemented; }
GroundStationResult requestGroundSensorSetting(const char *, const char *) { return GroundStationResult::NotImplemented; }

namespace {
bool validId(const char *text, uint32_t &id) {
  if (!*text) return false;
  uint64_t value = 0;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9') return false;
    value = value * 10 + (*text - '0');
    if (value > UINT32_MAX) return false;
  }
  id = static_cast<uint32_t>(value);
  return true;
}
// USB display only: use the original communications module's cached values.
// The original protocol reports freshness per branch, not per downstream node.
bool readGroundAircraftAttitude(int id, float &roll, float &pitch) {
  if (!aircraftConfigurationValid() || !validAircraftConfiguration(id) || id == 0) return false;
  if (id == aircraftId()) { roll = att[0]; pitch = att[1]; return true; }
  if (id == 1 || id < aircraftId() || !isSensorDataFresh(id % 2 == 0)) return false;
  const float rolls[] = {att_B[0], att_C[0], att_D[0], att_E[0], att_F[0], att_G[0]};
  const float pitches[] = {att_B[1], att_C[1], att_D[1], att_E[1], att_F[1], att_G[1]};
  roll = rolls[id - 2]; pitch = pitches[id - 2];
  return true;
}

bool activeSensor(const char *name) {
  if (!strcmp(name, "AIRSPEED") || !strcmp(name, "ROTATE")) return true;
#if defined INTIMU
  if (!strcmp(name, "IMU") || !strcmp(name, "GYRO") || !strcmp(name, "ACCEL")) return true;
#endif
#if defined EXTIMU
  if (!strcmp(name, "EXTIMU")) return true;
#endif
  return false;
}
}

void handleGroundStationCommand(char *line, char *response, size_t capacity) {
  char *tokens[20], *save;
  size_t count = 0;
  uint32_t id = 0;
  for (char *p = strtok_r(line, " \t", &save); p; p = strtok_r(nullptr, " \t", &save)) {
    if (count == 20) { count = 0; break; }
    tokens[count++] = p;
  }
  if (count < 3 || !validId(tokens[2], id)) {
    snprintf(response, capacity, "@COFLY\t0\tERROR\tSYNTAX\tUse GCS CAPS/DATA id\n");
    return;
  }
  const char *code = "SYNTAX", *message = "Invalid GCS arguments";
  if (!strcmp(tokens[1], "RC_SAVE") && count == 19) {
    if (!isFlightLocked()) { code = "LOCK_REQUIRED"; message = "Lock aircraft before radio calibration"; }
    else if (!radioInputValid()) { code = "RADIO_INVALID"; message = "Receiver signal missing or invalid"; }
    else if (!saveRadioCalibrationParameters(const_cast<const char *const *>(tokens + 3))) {
      code = "SAVE_FAILED"; message = "Invalid calibration or SD write failed; live parameters unchanged";
    } else {
      snprintf(response, capacity, "@COFLY\t%lu\tRC_SAVE\tstatus=SAVED\n", static_cast<unsigned long>(id));
      return;
    }
    snprintf(response, capacity, "@COFLY\t%lu\tERROR\t%s\t%s\n", static_cast<unsigned long>(id), code, message);
    return;
  }
  if (!strcmp(tokens[1], "REBOOT") && count == 3) {
    if (requestDeviceReboot())
      snprintf(response, capacity, "@COFLY\t%lu\tREBOOT\tstatus=REBOOTING\n", static_cast<unsigned long>(id));
    else
      snprintf(response, capacity, "@COFLY\t%lu\tERROR\t%s\t%s\n", static_cast<unsigned long>(id),
               isFlightLocked() ? "SAVE_FAILED" : "LOCK_REQUIRED",
               isFlightLocked() ? "Pending parameter writes failed; reboot cancelled" : "Lock aircraft before reboot");
    return;
  }
  if (!strcmp(tokens[1], "CAPS") && count == 3) {
    // main.loop samples BMI088 into the legacy *_6050 variables. Do not
    // claim MPU6050 or BMP280 is sampled based on old config names.
#if defined INTIMU && defined USE_MPU6050_I2C
    const char *imu = "BMI088_SPI";
#elif defined INTIMU
    const char *imu = "BMI088_SAMPLING_CONFIG_MISMATCH";
#else
    const char *imu = "DISABLED";
#endif
#if defined EXTIMU
    const char *external = "ENABLED";
#else
    const char *external = "DISABLED";
#endif
#if defined USE_BAROMETER
    const char *barometer = "INITIALIZED_NOT_SAMPLED";
#else
    const char *barometer = "DISABLED";
#endif
#if defined USE_SBUS_RX
    const char *radio = "SBUS";
#elif defined USE_PPM_RX
    const char *radio = "PPM";
#elif defined USE_PWM_RX
    const char *radio = "PWM";
#elif defined USE_DSM_RX
    const char *radio = "DSM";
#else
    const char *radio = "DISABLED";
#endif
    snprintf(response, capacity,
      "@COFLY\t%lu\tCAPS\tversion=1\timu=%s\texternal=%s\tairspeed=MS4525\trotate=MT6701\tbarometer=%s\tradio=%s\tcalibration=STUB\tsensor_setting=STUB\tflight_modes=REMOVED\treboot=SUPPORTED\tradio_calibration=SUPPORTED\n",
      static_cast<unsigned long>(id), imu, external, barometer, radio);
    return;
  }
  if (!strcmp(tokens[1], "DATA") && count == 3) {
    const char *mode = currentMode == MANUAL_MODE ? "MANUAL" :
      currentMode == STABLIZE_MODE_NO_I ? "STABILIZE_NO_I" : "STABILIZE";
    // Snapshot existing measurements only. No invented ground speed/altitude.
    float relativeRoll = 0;
    const bool relativeValid = localWingRelativeAngle(relativeRoll);
    const int written = snprintf(response, capacity,
      "@COFLY\t%lu\tDATA\tms=%lu\troll=%.3f\tpitch=%.3f\tyaw=%.3f\tgx=%.3f\tgy=%.3f\tgz=%.3f\tax=%.3f\tay=%.3f\taz=%.3f\tairspeed=%.3f\tangle=%.3f\tangle_valid=%d\trc1=%d\trc2=%d\trc3=%d\trc4=%d\trc5=%d\trc6=%d\trc7=%d\trc8=%d\tn1=%.3f\tn2=%.3f\tn3=%.3f\tn4=%.3f\tn5=%.3f\tn6=%.3f\tn7=%.3f\tn8=%.3f\tmode=%s\tlocked=%d\tradio_valid=%d\n",
      static_cast<unsigned long>(id), millis(), att[0], att[1], att[2],
      ang_vel[0], ang_vel[1], ang_vel[2], acc[0], acc[1], acc[2],
      airspeed_A, relativeRoll, relativeValid, radio_channel1_raw,
      radio_channel2_raw, radio_channel3_raw, radio_channel4_raw, radio_channel5_raw, radio_channel6_raw, radio_channel7_raw, radio_channel8_raw,
      radio_channel1_normalized, radio_channel2_normalized, radio_channel3_normalized, radio_channel4_normalized,
      radio_channel5_normalized, radio_channel6_normalized, radio_channel7_normalized, radio_channel8_normalized,
      mode, isFlightLocked(), radioInputValid());
    if (written < 0 || static_cast<size_t>(written) >= capacity)
      snprintf(response, capacity, "@COFLY\t%lu\tERROR\tDATA_TOO_LONG\tMeasurement exceeds frame capacity\n",
               static_cast<unsigned long>(id));
    return;
  }
  if (!strcmp(tokens[1], "CONFIG") && count == 3) {
    const unsigned master = aircraftConfigurationValid() && aircraftId() == 1;
    int written = snprintf(response, capacity,
      "@COFLY\t%lu\tCONFIG\tcount=%u\tmaster=%u\troll_a=%.3f\tab=%.3f\tac=%.3f\tbd=%.3f\tce=%.3f\tdf=%.3f\teg=%.3f\tleft_valid=%d\tright_valid=%d\taircraft_id=%d\tconfig_valid=%d\n",
      static_cast<unsigned long>(id), static_cast<unsigned>(aircraftCount()), master, att[0],
      att_B[0] - att[0], att_C[0] - att[0], att_D[0] - att_B[0],
      att_E[0] - att_C[0], att_F[0] - att_D[0], att_G[0] - att_E[0],
      isSensorDataFresh(true), isSensorDataFresh(false),
      aircraftId(), aircraftConfigurationValid());
    // Per-aircraft attitude comes from local IMU or valid downstream slots.
    if (written > 0 && static_cast<size_t>(written) < capacity) {
      --written; // Replace the preceding LF with additional fields.
      for (int node = 1; node <= aircraftCount(); ++node) {
        float roll = 0, pitch = 0;
        const bool valid = readGroundAircraftAttitude(node, roll, pitch);
        const int appended = snprintf(response + written, capacity - written,
          "\tr_%c=%.2f\tp_%c=%.2f\tv_%c=%d", 'a' + node - 1, roll, 'a' + node - 1, pitch, 'a' + node - 1, valid);
        if (appended < 0 || static_cast<size_t>(appended) >= capacity - written) { written = capacity; break; }
        written += appended;
      }
      if (static_cast<size_t>(written + 1) < capacity) {
        response[written++] = '\n'; response[written] = 0;
      } else written = capacity;
    }
    if (written < 0 || static_cast<size_t>(written) >= capacity)
      snprintf(response, capacity, "@COFLY\t%lu\tERROR\tDATA_TOO_LONG\tConfiguration exceeds frame capacity\n",
               static_cast<unsigned long>(id));
    return;
  }
  if (!strcmp(tokens[1], "CALIBRATE") && count == 4 && activeSensor(tokens[3])) {
    requestGroundCalibration(tokens[3]); code = "NOT_IMPLEMENTED"; message = "Calibration USB job reserved for V2";
  } else if (!strcmp(tokens[1], "SENSOR_SET") && count == 5 && activeSensor(tokens[3])) {
    requestGroundSensorSetting(tokens[3], tokens[4]); code = "NOT_IMPLEMENTED"; message = "Sensor setting reserved for V2";
  }
  snprintf(response, capacity, "@COFLY\t%lu\tERROR\t%s\t%s\n", static_cast<unsigned long>(id), code, message);
}
