#pragma once
#include <stdint.h>

// Internal lookup keys, never written to SD or EEPROM.
namespace ParameterKey {
constexpr uint16_t IntegralControlEnabled = 60;
constexpr uint16_t AircraftId = 1001;
constexpr uint16_t ImuAccErrorX6050 = 1101;
constexpr uint16_t ImuAccErrorY6050 = 1102;
constexpr uint16_t ImuAccErrorZ6050 = 1103;
constexpr uint16_t ImuGyroErrorX6050 = 1104;
constexpr uint16_t ImuGyroErrorY6050 = 1105;
constexpr uint16_t ImuGyroErrorZ6050 = 1106;
constexpr uint16_t ImuCalibrationValid = 1113;
constexpr uint16_t ImuCalibrationModel = 1114;
constexpr uint16_t ImuCalibrationChecksum = 1115;
constexpr uint16_t Mt6701Zero = 1201;
constexpr uint16_t UsbCommandLinkDebug = 1301;
constexpr uint16_t UsbSensorLinkDebug = 1302;
constexpr uint16_t UsbActuatorOutputDebug = 1303;
constexpr uint16_t UsbControlModeDebug = 1304;
constexpr uint16_t UsbAirspeedDebug = 1305;
// Five consecutive min/max pairs, channel 1 through channel 5.
constexpr uint16_t ActuatorPwmFirst = 1501;
constexpr uint16_t ActuatorPwmLast = 1510;
constexpr uint16_t ImuOffsetFirst = ImuAccErrorX6050;
constexpr uint16_t ImuOffsetLast = ImuGyroErrorZ6050;
constexpr unsigned ImuOffsetCount = 6;
} // namespace ParameterKey
