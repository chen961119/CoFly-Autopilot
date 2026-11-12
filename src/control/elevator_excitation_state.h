#pragma once

#include "control/control_types.h"
#include <stdint.h>

// Pure timing/state logic, separate from receiver, serial and servo hardware.
namespace ElevatorExcitation {
constexpr uint32_t SOURCE_TIMEOUT_MS = 100;

class Pulse {
public:
  void cancel() {
    active_ = armed_ = false;
    direction_ = 0;
  }

  // Neutral arms one edge; holding either end never repeats a completed pulse.
  // A direct +end -> -end transition cancels but cannot retrigger without neutral.
  bool update(uint32_t now, bool enabled, float position,
              int16_t x, int16_t y, uint16_t duration) {
    if (!enabled) {
      cancel();
      return false;
    }
    if (position >= -0.1f && position <= 0.1f) {
      active_ = false;
      armed_ = true;
      direction_ = 0;
      return false;
    }
    const int direction = position > 0.2f ? 1 : position < -0.2f ? -1 : 0;
    if (active_ && ((direction && direction != direction_) || remaining(now) == 0))
      active_ = false;
    if (armed_ && direction) {
      armed_ = false;
      active_ = true;
      direction_ = direction;
      start_ = now;
      duration_ = duration;
      delta_ = direction > 0 ? x : y;
      return true;
    }
    return false;
  }

  uint16_t remaining(uint32_t now) const {
    const uint32_t elapsed = now - start_;
    return active_ && elapsed < duration_ ? duration_ - elapsed : 0;
  }
  int16_t delta(uint32_t now) const { return remaining(now) ? delta_ : 0; }

private:
  bool active_ = false, armed_ = false;
  int direction_ = 0;
  int16_t delta_ = 0;
  uint16_t duration_ = 0;
  uint32_t start_ = 0;
};

class Master {
public:
  void cancel() {
    center_.cancel();
    all_.cancel();
    localDelta_ = 0;
    command_.pwmDeltaUs = 0;
    command_.remainingMs = 0;
  }

  void update(uint32_t now, bool enabled, bool centerValid, float centerPosition,
              bool allValid, float allPosition, int16_t x, int16_t y, uint16_t duration) {
    // Only A advances this counter. Relays forward it unchanged, including idle.
    if (++command_.sourceSequence == 0) ++command_.sourceSequence;
    if (all_.update(now, enabled && allValid, allPosition, x, y, duration)) {
      if (++command_.eventId == 0) ++command_.eventId;
    }
    // All-aircraft switch owns A while off-center, even after its pulse expires.
    // Suppressed center requests are cancelled, never queued or resumed.
    const bool allSelected = allValid && (allPosition < -0.1f || allPosition > 0.1f);
    center_.update(now, enabled && centerValid && !allSelected, centerPosition, x, y, duration);
    command_.remainingMs = all_.remaining(now);
    command_.pwmDeltaUs = all_.delta(now);
    localDelta_ = allSelected ? command_.pwmDeltaUs : center_.delta(now);
  }

  int16_t localDelta() const { return localDelta_; }
  const ElevatorExcitationCommand &command() const { return command_; }

private:
  Pulse center_, all_;
  ElevatorExcitationCommand command_{};
  int16_t localDelta_ = 0;
};

class Follower {
public:
  void cancel() {
    active_ = baseline_ = false;
  }

  int16_t update(uint32_t now, bool enabled, const ElevatorExcitationCommand &command) {
    if (!command.sourceSequence || command.pwmDeltaUs < -500 || command.pwmDeltaUs > 500 ||
        command.remainingMs > 60000 || (command.remainingMs && !command.eventId)) {
      cancel();
      return 0;
    }
    const uint32_t advance = command.sourceSequence - lastSequence_;
    const bool stale = haveSequence_ && now - sourceAdvancedAt_ >= SOURCE_TIMEOUT_MS;
    if (stale) cancel();
    bool progressed = !haveSequence_ || (advance && advance < 0x80000000u);
    // A reboot starts a lower sequence. Rejoin only after timeout and two
    // advancing idle samples; one old idle frame must not rewind the watermark.
    if (!progressed && stale && !command.remainingMs && advance) {
      const uint32_t restartAdvance = command.sourceSequence - restartSequence_;
      if (restartPending_ && restartAdvance && restartAdvance < 0x80000000u) {
        progressed = true;
      } else if (!restartPending_ || restartAdvance) {
        restartSequence_ = command.sourceSequence;
        restartPending_ = true;
      }
    }
    if (progressed) {
      haveSequence_ = true;
      lastSequence_ = command.sourceSequence;
      sourceAdvancedAt_ = now;
      restartPending_ = false;
    }
    if (!enabled || !haveSequence_ || now - sourceAdvancedAt_ >= SOURCE_TIMEOUT_MS) {
      cancel();
      blockedEvent_ = command.eventId;
      return 0;
    }
    if (progressed) {
      if (!command.remainingMs) {
        active_ = false;
        baseline_ = true;
        blockedEvent_ = command.eventId;
      } else if (active_ && event_ == command.eventId) {
        // Network jitter and repeated frames may shorten but never extend t.
        const uint32_t elapsed = now - start_;
        const uint16_t left = elapsed < duration_ ? duration_ - elapsed : 0;
        start_ = now;
        duration_ = command.remainingMs < left ? command.remainingMs : left;
      } else if (baseline_ && command.eventId != blockedEvent_) {
        event_ = blockedEvent_ = command.eventId;
        start_ = now;
        duration_ = command.remainingMs;
        delta_ = command.pwmDeltaUs;
        active_ = true;
      }
    }
    if (active_ && now - start_ >= duration_) active_ = false;
    return active_ ? delta_ : 0;
  }

private:
  bool active_ = false, baseline_ = false, haveSequence_ = false, restartPending_ = false;
  uint32_t lastSequence_ = 0, sourceAdvancedAt_ = 0;
  uint32_t restartSequence_ = 0;
  uint32_t event_ = 0, blockedEvent_ = 0, start_ = 0;
  uint16_t duration_ = 0;
  int16_t delta_ = 0;
};
} // namespace ElevatorExcitation
