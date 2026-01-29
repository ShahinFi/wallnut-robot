// actions/drive_straight.cpp
#include "actions/drive_straight.h"

#include <math.h>
#include "motor/motor.h"

static inline float clampSigned1(float x) {
  if (x < -1.0f) return -1.0f;
  if (x >  1.0f) return  1.0f;
  return x;
}

DriveStraight::DriveStraight()
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

void DriveStraight::setConfig(const Config& cfg) { cfg_ = cfg; }
const DriveStraight::Config& DriveStraight::config() const { return cfg_; }

void DriveStraight::begin(float headingDegContinuous,
                          float avgTravelCm,
                          float targetTravelCm,
                          float requestedSpeed) {
  stopMotors();

  // Allow "no distance" mode when targetTravelCm < 0
  infiniteDistance_ = (targetTravelCm < 0.0f);
  if (infiniteDistance_) targetTravelCm = 0.0f; // keep internal values benign

  requestedSpeed_    = clampSigned1(requestedSpeed);
  targetTravelCm_    = targetTravelCm;
  startTravelCm_     = avgTravelCm;
  remainingTravelCm_ = targetTravelCm;

  headingHoldDeg_    = headingDegContinuous;
  headingErrorDeg_   = 0.0f;

  startMs_ = millis();

  // Only allow immediate success when we actually have a distance target
  if (!infiniteDistance_ && targetTravelCm_ <= cfg_.distanceToleranceCm) {
    state_ = State::Succeeded;
    return;
  }

  state_ = State::Running;
}

bool DriveStraight::update(float headingDegContinuous, float avgTravelCm) {
  if (state_ != State::Running) return true;

  const uint32_t now = millis();
  if (now - startMs_ >= cfg_.timeoutMs) {
    stopMotors();
    state_ = State::TimedOut;
    return true;
  }

  // Distance stop condition only when not infinite.
  // Use ABS traveled so it behaves even if requestedSpeed is negative.
  if (!infiniteDistance_) {
    const float traveledCm = fabsf(avgTravelCm - startTravelCm_);
    remainingTravelCm_ = targetTravelCm_ - traveledCm;

    if (remainingTravelCm_ <= cfg_.distanceToleranceCm) {
      stopMotors();
      state_ = State::Succeeded;
      return true;
    }
  }

  // HEADING HOLD (continuous/unwrapped):
  // Do NOT wrap to [-180,180] because that will flip sign after 180° and "lose" heading.
  headingErrorDeg_ = headingHoldDeg_ - headingDegContinuous;

  // Base forward speed with tapering
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

  // Heading correction (deg -> speed)
  float corr = 0.0f;
  if (fabsf(headingErrorDeg_) > cfg_.headingDeadbandDeg) {
    corr = cfg_.kpHeading * cfg_.headingCorrectionSign * headingErrorDeg_;
  }
  if (corr >  cfg_.maxCorrection) corr =  cfg_.maxCorrection;
  if (corr < -cfg_.maxCorrection) corr = -cfg_.maxCorrection;

  // Differential steering
  float leftCmd  = base - corr;
  float rightCmd = base + corr;

  // IMPORTANT: clamp outputs after correction so motorDrive never sees >1 or <-1.
  leftCmd  = clampSigned1(leftCmd);
  rightCmd = clampSigned1(rightCmd);

  motorDrive(leftCmd, rightCmd);
  return false;
}

void DriveStraight::cancel() {
  stopMotors();
  state_ = State::Cancelled;
}

void DriveStraight::reset() {
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

bool DriveStraight::active() const { return state_ == State::Running; }
bool DriveStraight::succeeded() const { return state_ == State::Succeeded; }
bool DriveStraight::timedOut() const { return state_ == State::TimedOut; }
DriveStraight::State DriveStraight::state() const { return state_; }

float DriveStraight::remainingCm() const { return remainingTravelCm_; }
float DriveStraight::headingErrorDeg() const { return headingErrorDeg_; }

void DriveStraight::setRequestedSpeed(float requestedSpeed) {
  requestedSpeed_ = clampSigned1(requestedSpeed);
}

void DriveStraight::setHeadingHoldDeg(float headingDegContinuous) {
  headingHoldDeg_ = headingDegContinuous;
}

void DriveStraight::stopMotors() {
  motorDrive(0.0f, 0.0f);
}

float DriveStraight::computeForwardSpeed(float requestedSpeed, float remainingCmAbs) const {
  float speed = requestedSpeed;
  if (speed > cfg_.maxSpeed) speed = cfg_.maxSpeed;

  if (cfg_.slowDownCm <= 0.0f) return speed;
  if (remainingCmAbs >= cfg_.slowDownCm) return speed;

  // Taper linearly to minSpeed
  const float t = remainingCmAbs / cfg_.slowDownCm; // 0..1
  float tapered = cfg_.minSpeed + t * (speed - cfg_.minSpeed);

  if (tapered < cfg_.minSpeed) tapered = cfg_.minSpeed;
  if (tapered > speed) tapered = speed;

  return tapered;
}
