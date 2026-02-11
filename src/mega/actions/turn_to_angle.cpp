#include "actions/turn_to_angle.h"

#include <math.h>
#include "motor/motor.h"

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

TurnToAngle::TurnToAngle()
: cfg_{},
  state_(State::Idle),
  targetHeadingDegContinuous_(0.0f),
  remainingDeg_(0.0f),
  requestedSpeed_(0.0f),
  startMs_(0) {}

void TurnToAngle::setConfig(const Config& cfg) { cfg_ = cfg; }
const TurnToAngle::Config& TurnToAngle::config() const { return cfg_; }

void TurnToAngle::begin(float currentHeadingDegContinuous, float deltaDeg, float requestedSpeed) {
  stopMotors();

  requestedSpeed_ = clamp01(requestedSpeed);
  targetHeadingDegContinuous_ = currentHeadingDegContinuous + deltaDeg;
  remainingDeg_ = deltaDeg;
  startMs_ = millis();

  // If already basically at target, finish immediately
  if (fabsf(deltaDeg) <= cfg_.toleranceDeg) {
    state_ = State::Succeeded;
    return;
  }

  state_ = State::Running;
}

bool TurnToAngle::update(float currentHeadingDegContinuous) {
  // “Finished” means caller doesn't need to keep calling.
  if (state_ != State::Running) return true;

  const uint32_t now = millis();
  if (now - startMs_ >= cfg_.timeoutMs) {
    stopMotors();
    state_ = State::TimedOut;
    return true;
  }

  const float remaining = targetHeadingDegContinuous_ - currentHeadingDegContinuous;
  const float remainingAbs = fabsf(remaining);
  remainingDeg_ = remaining;

  if (remainingAbs <= cfg_.toleranceDeg) {
    stopMotors();
    state_ = State::Succeeded;
    return true;
  }

  const float speedCmd = computeSpeedCmd(requestedSpeed_, remainingAbs);

  // Spin direction comes from sign of remaining
  const float wheelCmd = copysignf(speedCmd, remaining) * cfg_.motorTurnSign;

  // In-place turn
  motorDrive(wheelCmd, -wheelCmd);

  return false;
}

void TurnToAngle::cancel() {
  stopMotors();
  state_ = State::Cancelled;
}

void TurnToAngle::reset() {
  stopMotors();
  state_ = State::Idle;
  remainingDeg_ = 0.0f;
  requestedSpeed_ = 0.0f;
  targetHeadingDegContinuous_ = 0.0f;
  startMs_ = 0;
}

bool TurnToAngle::active() const { return state_ == State::Running; }
bool TurnToAngle::succeeded() const { return state_ == State::Succeeded; }
bool TurnToAngle::timedOut() const { return state_ == State::TimedOut; }
TurnToAngle::State TurnToAngle::state() const { return state_; }
float TurnToAngle::remainingDeg() const { return remainingDeg_; }

void TurnToAngle::stopMotors() {
  motorDrive(0.0f, 0.0f);
}

float TurnToAngle::computeSpeedCmd(float requestedSpeed, float remainingAbs) const {
  // Cap to maxSpeed
  float speed = requestedSpeed;
  if (speed > cfg_.maxSpeed) speed = cfg_.maxSpeed;

  // If no tapering requested, return capped speed
  if (cfg_.slowDownDeg <= 0.0f) return speed;

  // Far from target: full capped speed
  if (remainingAbs >= cfg_.slowDownDeg) return speed;

  // Near target: taper down toward minSpeed (but never exceed speed)
  const float t = remainingAbs / cfg_.slowDownDeg; // 0..1
  float tapered = cfg_.minSpeed + t * (speed - cfg_.minSpeed);

  // Ensure bounds and don't exceed requested/capped speed
  if (tapered < 0.0f) tapered = 0.0f;
  if (tapered < cfg_.minSpeed) tapered = cfg_.minSpeed;
  if (tapered > speed) tapered = speed;

  return tapered;
}
