#include "actions/drive_by_distance.h"

#include <math.h>
#include "motor/motor.h"

static inline float clampSigned1(float x) {
  if (x < -1.0f) return -1.0f;
  if (x >  1.0f) return  1.0f;
  return x;
}

DriveByDistance::DriveByDistance()
: cfg_{},
  state_(State::Idle),
  infiniteDistance_(false),
  targetTravelCm_(0.0f),
  startTravelCm_(0.0f),
  remainingTravelCm_(0.0f),
  requestedSpeed_(0.0f),
  headingHoldDeg_(0.0f),
  headingErrorDeg_(0.0f),
  startMs_(0) {}

void DriveByDistance::setConfig(const Config& cfg) { cfg_ = cfg; }
const DriveByDistance::Config& DriveByDistance::config() const { return cfg_; }

void DriveByDistance::beginByDistance(float headingDegContinuous,
                                      float avgTravelCm,
                                      float targetTravelCm,
                                      float requestedSpeed) {
  stopMotors();

  if (!(targetTravelCm >= 0.0f)) targetTravelCm = 0.0f;
  infiniteDistance_ = false;

  requestedSpeed_    = clampSigned1(requestedSpeed);
  targetTravelCm_    = targetTravelCm;
  startTravelCm_     = avgTravelCm;
  remainingTravelCm_ = targetTravelCm;

  headingHoldDeg_    = headingDegContinuous;
  headingErrorDeg_   = 0.0f;

  startMs_ = millis();

  // WHY: Only allow immediate success when we actually have a distance target
  if (targetTravelCm_ <= cfg_.distanceToleranceCm) {
    state_ = State::Succeeded;
    return;
  }

  state_ = State::Running;
}

void DriveByDistance::beginContinuous(float headingDegContinuous,
                                      float avgTravelCm,
                                      float requestedSpeed) {
  stopMotors();

  infiniteDistance_ = true;
  requestedSpeed_ = clampSigned1(requestedSpeed);
  targetTravelCm_ = 0.0f;
  startTravelCm_ = avgTravelCm;
  remainingTravelCm_ = 0.0f;

  headingHoldDeg_ = headingDegContinuous;
  headingErrorDeg_ = 0.0f;
  startMs_ = millis();

  state_ = State::Running;
}

bool DriveByDistance::update(float headingDegContinuous, float avgTravelCm) {
  if (state_ != State::Running) return true;

  const uint32_t now = millis();
  if (now - startMs_ >= cfg_.timeoutMs) {
    stopMotors();
    state_ = State::TimedOut;
    return true;
  }

  // WHY: Distance stop condition only when not infinite.
  // WHY: Use ABS traveled so it behaves even if requestedSpeed is negative.
  if (!infiniteDistance_) {
    const float traveledCm = fabsf(avgTravelCm - startTravelCm_);
    remainingTravelCm_ = targetTravelCm_ - traveledCm;

    if (remainingTravelCm_ <= cfg_.distanceToleranceCm) {
      stopMotors();
      state_ = State::Succeeded;
      return true;
    }
  }

  // CONTRACT: Heading error uses continuous angle domain (no wrapping).
  headingErrorDeg_ = headingHoldDeg_ - headingDegContinuous;

  // WHY: Base forward speed with tapering
  const float speedMag = fabsf(requestedSpeed_);
  float speedCmd = speedMag;

  if (speedCmd > cfg_.maxSpeed) speedCmd = cfg_.maxSpeed;
  if (speedCmd > 0.0f && speedCmd < cfg_.minSpeed) speedCmd = cfg_.minSpeed;

  if (!infiniteDistance_) {
    const float remainingAbs = fabsf(remainingTravelCm_);
    speedCmd = computeForwardSpeed(speedCmd, remainingAbs);
  }

  const float dir = (requestedSpeed_ >= 0.0f) ? 1.0f : -1.0f;
  const float base = speedCmd * dir * cfg_.motorForwardSign;

  // WHY: Heading correction (deg -> speed)
  float corr = 0.0f;
  if (fabsf(headingErrorDeg_) > cfg_.headingDeadbandDeg) {
    corr = cfg_.kpHeading * cfg_.headingCorrectionSign * headingErrorDeg_;
  }
  if (corr >  cfg_.maxCorrection) corr =  cfg_.maxCorrection;
  if (corr < -cfg_.maxCorrection) corr = -cfg_.maxCorrection;

  // WHY: Differential steering
  float leftCmd  = base - corr;
  float rightCmd = base + corr;

  // CONTRACT: clamp outputs after correction so motorDrive never sees >1 or <-1.
  leftCmd  = clampSigned1(leftCmd);
  rightCmd = clampSigned1(rightCmd);

  motorDrive(leftCmd, rightCmd);
  return false;
}

void DriveByDistance::cancel() {
  stopMotors();
  state_ = State::Cancelled;
}

void DriveByDistance::reset() {
  stopMotors();
  state_ = State::Idle;

  infiniteDistance_ = false;

  targetTravelCm_ = 0.0f;
  startTravelCm_ = 0.0f;
  remainingTravelCm_ = 0.0f;

  requestedSpeed_ = 0.0f;
  headingHoldDeg_ = 0.0f;
  headingErrorDeg_ = 0.0f;

  startMs_ = 0;
}

bool DriveByDistance::active() const { return state_ == State::Running; }
bool DriveByDistance::succeeded() const { return state_ == State::Succeeded; }
bool DriveByDistance::timedOut() const { return state_ == State::TimedOut; }
DriveByDistance::State DriveByDistance::state() const { return state_; }

float DriveByDistance::remainingCm() const { return remainingTravelCm_; }
float DriveByDistance::headingErrorDeg() const { return headingErrorDeg_; }

void DriveByDistance::setRequestedSpeed(float requestedSpeed) {
  requestedSpeed_ = clampSigned1(requestedSpeed);
}

void DriveByDistance::setHeadingHoldDeg(float headingDegContinuous) {
  headingHoldDeg_ = headingDegContinuous;
}

void DriveByDistance::stopMotors() {
  motorDrive(0.0f, 0.0f);
}

float DriveByDistance::computeForwardSpeed(float requestedSpeed, float remainingCmAbs) const {
  float speed = requestedSpeed;
  if (speed > cfg_.maxSpeed) speed = cfg_.maxSpeed;

  if (cfg_.slowDownCm <= 0.0f) return speed;
  if (remainingCmAbs >= cfg_.slowDownCm) return speed;

  // WHY: Taper linearly to minSpeed
  const float t = remainingCmAbs / cfg_.slowDownCm; // 0..1
  float tapered = cfg_.minSpeed + t * (speed - cfg_.minSpeed);

  if (tapered < cfg_.minSpeed) tapered = cfg_.minSpeed;
  if (tapered > speed) tapered = speed;

  return tapered;
}

