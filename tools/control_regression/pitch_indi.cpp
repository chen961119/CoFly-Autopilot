// From the repository root: python tools/control_regression/run_pitch_indi.py
#include "actuators/actuator_pwm_limits.h"
#include "actuators/actuator_pwm_mapping.h"
#include "allocation/command_fusion.h"
#include "allocation/master_mixer.h"
#include "allocation/slave_mixer.h"
#include "config/aircraft_config.h"
#include "control/control_modes.h"
#include "control/control_output.h"
#include "control/control_pipeline.h"
#include "control/pitch_indi_controller.h"
#include "math/units.h"
#include "parameters/control_parameters.h"
#include "sensors/sensor_processing.h"
#include "system/flight_clock.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

// Hardware inputs and shared outputs used by the linked firmware translation units.
float dt = 0.01f;
float att[3]{}, ang_vel_filtered[3]{}, ang_acc[3]{};
int32_t pwm_servo3_min_us, pwm_servo3_max_us, pwm_channel3_rev, pwm_channel3_mode;
float pwm_channel3_trim;
FlightMode currentFlightMode = STABILIZE_MODE;
PitchController activePitchController = PitchController::INDI;
SlaveCommand slaveCommand{};
ControlCycleOutput commands{};
float manual_input[4]{};
float mixerStrategyCommand = 1;
bool brakeEnabled = false;
// Strategy 1's throttle allocation never calls this unrelated math helper.
float keeppositive(float) { std::abort(); }

using namespace ControlParameters;

struct Profile {
  int aircraft, minimum, trim, maximum;
};
// 2026-11-12 白鹿原刚性连接柔性机翼 A/B/C, physical servo channel 3.
constexpr Profile profiles[] = {{1, 1350, 1580, 1900}, {2, 1213, 1439, 1750},
                                {3, 1033, 1436, 1800}};

static void near(const char *what, float actual, float expected, float tolerance = 0.0002f) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    std::fprintf(stderr, "%s: expected %.6f, got %.6f (trim %.0f, rev %d, mode %d)\n",
                 what, expected, actual, pwm_channel3_trim, int(pwm_channel3_rev),
                 int(pwm_channel3_mode));
    std::exit(EXIT_FAILURE);
  }
}

static void configure(const Profile &profile, int reverse) {
  aircraftIdParameter = profile.aircraft;
  activateAircraftConfiguration();
  pwm_servo3_min_us = profile.minimum;
  pwm_channel3_trim = profile.trim;
  pwm_servo3_max_us = profile.maximum;
  pwm_channel3_rev = reverse;
  pwm_channel3_mode = 0;
  indi_pitch_up_rad = Units::toRadians(15.0f);
  indi_pitch_down_rad = Units::toRadians(20.0f);
  // With zero target and feedback rate, this makes an acceleration input of one
  // rad/s² request one radian of elevator from the initially neutral model.
  indi_pitch_effectiveness = -0.7f;
  indi_pitch_rate_limit_rad_s = Units::toRadians(5000.0f);
  indi_pitch_cmd_lpf_fc_hz = 0;
  indi_pitch_servo_delay_s = 0;
  indi_pitch_servo_tau_s = 0;
  ang_acc[1] = 0;
  slaveCommand = {};
}

static void update(PitchIndiController &controller, float accelerationDegPerSec2) {
  ang_acc[1] = Units::toRadians(accelerationDegPerSec2);
  controller.update(0, true);
  assert(pitchIndiControlOutput.isValid);
}

static int mixedPwm(float feedforward, float expectedPwm) {
  if (aircraftId() == 1) {
    assert(feedforward == 0);
    mixMasterCommands();
  } else {
    slaveCommand.masterContribution.elevator = feedforward;
    mixSlaveCommands();
    fuseSlaveCommands();
  }
  const float pwm = actuatorCommandToPwm(commands.local.elevator, pwm_channel3_rev,
      pwm_servo3_min_us, pwm_channel3_trim, pwm_servo3_max_us, pwm_channel3_mode);
  near("PWM before final clamp", pwm, expectedPwm, 0.001f);
  return limitActuatorPwm(pwm, pwm_servo3_min_us, pwm_servo3_max_us,
                          static_cast<int>(pwm_channel3_trim));
}

// Use independent measured PWM landmarks to check the published value, mixer,
// actuator mapping and delayed-PWM-to-angle conversion in the real controller.
static void checkPoint(float input, float degrees, float mixerCommand, float expectedPwm) {
  PitchIndiController controller;
  update(controller, input);
  near("command angle (deg)", Units::toDegrees(pitchIndiControlOutput.delta_e_cmd), degrees);
  near("estimated angle (deg)", Units::toDegrees(pitchIndiControlOutput.delta_e_hat), degrees);
  near("pitch intention", pitchIndiControlOutput.indiOutput, -mixerCommand);
  near("final PWM", mixedPwm(0, expectedPwm), static_cast<int>(expectedPwm), 1.0f);
}

static void checkProfile(const Profile &profile, int reverse) {
  configure(profile, reverse);
  const int up = reverse == 1 ? profile.minimum : profile.maximum;
  const int down = reverse == 1 ? profile.maximum : profile.minimum;
  checkPoint(0, 0, 0, profile.trim);
  checkPoint(-7.5f, -7.5f, -0.5f, (up + profile.trim) * 0.5f);
  checkPoint(10, 10, 0.5f, (down + profile.trim) * 0.5f);
  checkPoint(-100, -15, -1, up);
  if (profile.aircraft != 1)
    assert(mixedPwm(-0.5f, up + 0.5f * (up - profile.trim)) == up);
  checkPoint(100, 20, 1, down);
  if (profile.aircraft != 1)
    assert(mixedPwm(0.5f, down + 0.5f * (down - profile.trim)) == down);
}

static void checkDynamics() {
  configure(profiles[1], -1);
  PitchIndiController controller;
  // The real pitch loop must request nose-up / elevator-up even on a reversed installation.
  indi_pitch_effectiveness = -113.65f;
  controller.reset(Units::toRadians(5));
  controller.update(Units::toRadians(5), true);
  assert(pitchIndiControlOutput.q_ref > 0 && pitchIndiControlOutput.q_dot_ref > 0);
  assert(pitchIndiControlOutput.delta_e_cmd < 0 && pitchIndiControlOutput.indiOutput > 0);
  controller.reset(0);
  indi_pitch_effectiveness = -0.7f;
  indi_pitch_rate_limit_rad_s = Units::toRadians(100.0f);
  indi_pitch_servo_delay_s = 2 * dt;
  indi_pitch_servo_tau_s = dt;
  // At 10 ms per update the limited commands are 1°, 2°, 1°. Only the
  // third sample sees the first PWM, producing 0.5° in the first-order model.
  update(controller, 100);
  near("rate-limited first sample (deg)", Units::toDegrees(pitchIndiControlOutput.delta_e_cmd), 1);
  near("initial delay", pitchIndiControlOutput.delta_e_hat, 0);
  update(controller, 100);
  near("rate-limited second sample (deg)", Units::toDegrees(pitchIndiControlOutput.delta_e_cmd), 2);
  near("delay before first arrival", pitchIndiControlOutput.delta_e_hat, 0);
  update(controller, -100);
  near("rate-limited reversal (deg)", Units::toDegrees(pitchIndiControlOutput.delta_e_cmd), 1);
  near("first delayed response (deg)", Units::toDegrees(pitchIndiControlOutput.delta_e_hat), 0.5f);
  controller.reset(0);
  update(controller, 0);
  near("neutral after reset", pitchIndiControlOutput.delta_e_hat, 0);
  near("neutral output after reset", pitchIndiControlOutput.indiOutput, 0);
}

static void checkCompatibility() {
  configure(profiles[0], 1);
  indi_pitch_pwm_to_rad_k = Units::toRadians(0.1f);
  indi_pitch_up_rad = 0;  // This side alone falls back to the old 0.1°/µs slope.
  checkPoint(-100, -23, -1, 1350);
  checkPoint(100, 20, 1, 1900);
  indi_pitch_down_rad = 0;
  checkPoint(100, 32, 1, 1900);
  // In mode 1 the neutral is the arithmetic midpoint, and the logical output
  // is an absolute [0,1] position. The saved trim is still the safe position.
  configure(profiles[0], -1);
  pwm_channel3_mode = 1;
  checkPoint(0, 0, 0.5f, 1625);
  checkPoint(-100, -15, 0, 1900);
  checkPoint(100, 20, 1, 1350);
  configure(profiles[0], 1);
  pwm_servo3_min_us = static_cast<int>(pwm_channel3_trim);
  checkPoint(-100, 0, 0, pwm_channel3_trim);
  checkPoint(100, 20, 1, pwm_servo3_max_us);
}

int main() {
  for (const auto &profile : profiles) {
    checkProfile(profile, 1);
    checkProfile(profile, -1);
    std::printf("%s control-to-PWM checks passed\n", aircraftName());
  }
  checkDynamics();
  checkCompatibility();
  std::puts("Pitch INDI delay, rate limit and mapping compatibility passed");
}
