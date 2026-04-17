#include "lidar/turret/turret_angle_tracker.h"

#include <math.h>

TurretAngleTracker::TurretAngleTracker()
: cfg_{},
  hasPrev_(false),
  prevTicksAbs_(0),
  angleDegAcc_(0.0f),
  zeroAngleDegAcc_(0.0f),
  ticksPerRevPos_(0),
  ticksPerRevNeg_(0) {}

void TurretAngleTracker::setConfig(const Config& cfg) {
  cfg_ = cfg;
  if (cfg_.angleSign < 0) cfg_.angleSign = -1;
  else cfg_.angleSign = 1;
  if (!isfinite(cfg_.cmdDeadband) || cfg_.cmdDeadband < 0.0f) cfg_.cmdDeadband = 0.0f;
}

const TurretAngleTracker::Config& TurretAngleTracker::config() const { return cfg_; }

void TurretAngleTracker::setTicksPerRev(uint32_t ticksPerRev) {
  setTicksPerRevPosNeg(ticksPerRev, ticksPerRev);
}

void TurretAngleTracker::setTicksPerRevPosNeg(uint32_t ticksPerRevPos, uint32_t ticksPerRevNeg) {
  ticksPerRevPos_ = ticksPerRevPos;
  ticksPerRevNeg_ = ticksPerRevNeg;
}

uint32_t TurretAngleTracker::ticksPerRevPos() const { return ticksPerRevPos_; }
uint32_t TurretAngleTracker::ticksPerRevNeg() const { return ticksPerRevNeg_; }

uint32_t TurretAngleTracker::ticksPerRevForDirSign(int dirSign) const {
  return (dirSign < 0) ? ticksPerRevNeg_ : ticksPerRevPos_;
}

void TurretAngleTracker::reset() {
  hasPrev_ = false;
  prevTicksAbs_ = 0;
  angleDegAcc_ = 0.0f;
  zeroAngleDegAcc_ = 0.0f;
}

void TurretAngleTracker::setZero() {
  zeroAngleDegAcc_ = angleDegAcc_;
}

void TurretAngleTracker::update(long ticksAbsNow, float motorCmd) {
  if (!hasPrev_) {
    hasPrev_ = true;
    prevTicksAbs_ = ticksAbsNow;
    return;
  }

  const long dt = ticksAbsNow - prevTicksAbs_;
  prevTicksAbs_ = ticksAbsNow;

  // Handle hard reset / wrap (ticks should be monotonic).
  if (dt < 0) {
    hasPrev_ = true;
    angleDegAcc_ = 0.0f;
    zeroAngleDegAcc_ = 0.0f;
    return;
  }
  if (dt == 0) return;

  // With single-channel encoder, direction is unknown unless the motor is being driven.
  if (!isfinite(motorCmd) || fabsf(motorCmd) <= cfg_.cmdDeadband) {
    return;  // ignore manual/unknown-direction motion
  }

  const int dirSign = (motorCmd > 0.0f) ? 1 : -1;
  const uint32_t tpr = ticksPerRevForDirSign(dirSign);
  if (tpr == 0) return;
  const float degPerTick = 360.0f / (float)tpr;
  angleDegAcc_ += (float)cfg_.angleSign * (float)dirSign * (float)dt * degPerTick;
}

float TurretAngleTracker::angleDegSigned() const {
  return angleDegAcc_ - zeroAngleDegAcc_;
}

float TurretAngleTracker::wrapDeg360_(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float TurretAngleTracker::angleDegWrapped360() const {
  return wrapDeg360_(angleDegSigned());
}
