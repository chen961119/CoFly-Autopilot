#include "host.h"
#include "config/aircraft_config.h"
#include "parameters/control_parameters.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "radio_comm/radio_comm.h"
#include "control/control_modes.h"
#include "control/control_pipeline.h"
#include "actuators/actuator_output.h"
#include "sensors/sensor_processing.h"

// Inputs to the controller from the estimator, plant and operating mode.
float dt = 0.002f, ang_vel[3]{}, ang_vel_filtered[3]{}, ang_acc[3]{}, acc[3]{};
float airspeed = 12.0f;
float AccX_6050, AccY_6050, AccZ_6050, GyroX_6050, GyroY_6050, GyroZ_6050;
bool locked = true;
bool airspeedValid() { return true; }
bool isFlightLocked() { return locked; }
void displayFlightModeIndicators(FlightMode) {}
void initializeAngularAccelerationFilters() {}
bool queueDiagnosticOutput(const char *, size_t) { return true; }

static void enqueue(Stream &port, const std::vector<uint8_t> &bytes) {
  port.rx.insert(port.rx.end(), bytes.begin(), bytes.end());
}

static std::vector<uint8_t> sbusFrame(unsigned step, uint8_t flags = 0) {
  std::vector<uint8_t> bytes(25);
  bytes[0] = 0x0f;
  for (unsigned ch = 0; ch < 8; ++ch) {
    unsigned value = 350 + (80 * ch + step * 11) % 1300;
    if (ch == 4) value = step < 28 ? 1630 : 320;
    for (unsigned b = 0; b < 11; ++b) {
      unsigned bit = 11 * ch + b;
      bytes[1 + bit / 8] |= ((value >> b) & 1) << (bit % 8);
    }
  }
  bytes[23] = flags;
  return bytes;
}

static std::vector<uint8_t> peerFrame(bool command, bool left, unsigned step) {
  std::vector<uint8_t> bytes(command ? 40 : 34);
  bytes[0] = 0x55;
  bytes[1] = command ? 0x60 : 0x71;
  bytes[2] = command ? (step < 28 ? MANUAL_MODE : STABILIZE_MODE) : left;
  for (unsigned i = 3; i < bytes.size() - 1; i += 2) {
    const int16_t n = (static_cast<int>(31 * step + 17 * i) % 1300) - 650;
    bytes[i] = static_cast<uint8_t>(n);
    bytes[i + 1] = static_cast<uint16_t>(n) >> 8;
  }
  for (size_t i = 0; i < bytes.size() - 1; ++i) bytes.back() += bytes[i];
  return bytes;
}

static void reportWire(Stream &port, unsigned portNumber) {
  if (port.tx.empty()) return;
  std::printf("uart%u ", portNumber);
  for (uint8_t b : port.tx) std::printf("%02x", static_cast<unsigned>(b));
  std::puts("");
  port.tx.clear();
}

static void driveControl(unsigned step, int id) {
  att[0] = (static_cast<int>(step % 9) - 4) * .01f;
  att[1] = .08f - step * .0009f;
  att[2] = step * .0003f;
  for (unsigned a = 0; a < 3; ++a) {
    ang_vel[a] = (step + a) * .0012f;
    ang_vel_filtered[a] = .9f * ang_vel[a];
    ang_acc[a] = .2f * a;
  }
  // Inject the same bytes before both versions consume this control cycle.
  if (step % 4 == 0) enqueue(Serial2, sbusFrame(step));
  if (step % 3 == 0) {
    enqueue(Serial3, peerFrame(false, true, step));
    enqueue(Serial5, id == 1 ? peerFrame(false, false, step) : peerFrame(true, false, step));
  }
#if REPLAY_BASELINE
  receiveSensorData();
#else
  // Collect RX before control, suppress the next TX until the new output is published.
  Serial3.freeBytes = Serial5.freeBytes = 0;
  pollFlightLinks();
  Serial3.freeBytes = Serial5.freeBytes = 256;
  latchInteraircraftInputs();
#endif
  runControlCycle();
  convertControlCommandsToPWM();
#if REPLAY_BASELINE
  prepareActuatorCommands();
  applyAndTransmitActuatorCommands();
  getCommands();
  updateRadioNormalized();
#else
  applyActuatorCommands();
  publishInteraircraftOutputs(true);
  pollFlightLinks();
  latchRadioInput();
#endif
  std::printf("%u pwm %d %d %d %d %d rc %d %d %d %d valid %d mode %d\n",
              step, pwm[2], pwm[3], pwm[4], pwm[5], pwm[6], radio_channel1_raw,
              radio_channel2_raw, radio_channel3_raw, radio_channel4_raw,
              radioInputValid(), currentFlightMode);
  reportWire(Serial3, 3);
  reportWire(Serial5, 5);
}

#if !REPLAY_BASELINE
static void checkBoundaries() {
  // Serial bursts cannot publish half a frame or monopolize the 1 ms task.
  std::vector<uint8_t> full = peerFrame(false, true, 4);
  float oldRoll = att_B[0];
  Serial3.rx.insert(Serial3.rx.end(), 256, 0xcc);
  Serial3.rx.insert(Serial3.rx.end(), full.begin(), full.begin() + 10);
  pollFlightLinks();
  assert(Serial3.available() == 138);
  pollFlightLinks();
  pollFlightLinks();
  latchInteraircraftInputs();
  assert(att_B[0] == oldRoll);
  Serial3.rx.insert(Serial3.rx.end(), full.begin() + 10, full.end());
  pollFlightLinks();
  assert(att_B[0] == oldRoll);
  latchInteraircraftInputs();
  assert(att_B[0] != oldRoll && isSensorDataFresh(true));
  oldRoll = att_B[0];
  auto broken = peerFrame(false, true, 5);
  broken.back() ^= 1;
  enqueue(Serial3, broken);
  pollFlightLinks();
  latchInteraircraftInputs();
  assert(att_B[0] == oldRoll);

  enqueue(Serial2, sbusFrame(41, 4));
  pollFlightLinks();
  assert(radioInputValid());
  latchRadioInput();
  assert(!radioInputValid());
  enqueue(Serial2, sbusFrame(42));
  pollFlightLinks();
  latchRadioInput();
  assert(radioInputValid());
  replayUs += 1001000;
  assert(!radioInputValid() && !isSensorDataFresh(true));

  publishInteraircraftOutputs(true);
  Serial3.tx.clear();
  Serial3.freeBytes = 0;
  pollFlightLinks();
  assert(Serial3.tx.empty());
  Serial3.freeBytes = 256;
  pollFlightLinks();
  assert(Serial3.tx.size() == 40);
  replayUs += 20000;
  pollFlightLinks();
  assert(Serial3.tx.size() == 40); // no new control output
  publishInteraircraftOutputs(false);
  pollFlightLinks();
  assert(Serial3.tx.size() == 40);
}
#endif

int main(int argc, char **argv) {
  assert(argc == 2);
  int id = std::atoi(argv[1]);
  aircraftIdParameter = id;
  activateAircraftConfiguration();
  beginInteraircraftLinks();
  initializeCommandLinks();
  initializeSensorLinks();
  initializeInitialControlMode();
  initializeControlFilters();
  attachActuators();
  radioSetup();
  commandLinkDebugSetting() = sensorLinkDebugSetting() = 0;
#if !REPLAY_BASELINE
  publishInteraircraftOutputs(true);
#endif
  for (unsigned step = 0; step < 80; ++step) {
    replayUs = 100000 + 2000 * step;
    locked = step < 4 || step >= 76;
    ControlParameters::pitch_controller = step < 48 ? 0 : 1;
    driveControl(step, id);
  }
#if !REPLAY_BASELINE
  if (id == 1) checkBoundaries();
#endif
}
