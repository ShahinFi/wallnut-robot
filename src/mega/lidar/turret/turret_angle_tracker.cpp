#include "lidar/turret/turret_angle_tracker.h"

#include <math.h>

TurretAngleTracker::TurretAngleTracker()
: cfg_{},
  hasPrev_(false),
  prevTicksAbs_(0),
  signedTicksAcc_(0),
  zeroSignedTicks_(0),
  ticksPerRev_(0) {}

void TurretAngleTracker::setConfig(const Config& cfg) {
  cfg_ = cfg;
  if (cfg_.angleSign < 0) cfg_.angleSign = -1;
  else cfg_.angleSign = 1;
  if (!isfinite(cfg_.cmdDeadband) || cfg_.cmdDeadband < 0.0f) cfg_.cmdDeadband = 0.0f;
}

const TurretAngleTracker::Config& TurretAngleTracker::config() const { return cfg_; }

void TurretAngleTracker::setTicksPerRev(uint32_t ticksPerRev) {
  ticksPerRev_ = ticksPerRev;
}

uint32_t TurretAngleTracker::ticksPerRev() const { return ticksPerRev_; }

void TurretAngleTracker::reset() {
  hasPrev_ = false;
  prevTicksAbs_ = 0;
  signedTicksAcc_ = 0;
  zeroSignedTicks_ = 0;
}

void TurretAngleTracker::setZero() {
  zeroSignedTicks_ = signedTicksAcc_;
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
    signedTicksAcc_ = 0;
    zeroSignedTicks_ = 0;
    return;
  }
  if (dt == 0) return;

  // With single-channel encoder, direction is unknown unless the motor is being driven.
  if (!isfinite(motorCmd) || fabsf(motorCmd) <= cfg_.cmdDeadband) {
    return;  // ignore manual/unknown-direction motion
  }

  const long dir = (motorCmd > 0.0f) ? 1L : -1L;
  signedTicksAcc_ += (long)cfg_.angleSign * dir * dt;
}

float TurretAngleTracker::angleDegSigned() const {
  if (ticksPerRev_ == 0) return 0.0f;
  const long rel = signedTicksAcc_ - zeroSignedTicks_;
  return (float)rel * (360.0f / (float)ticksPerRev_);
}

float TurretAngleTracker::wrapDeg360_(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float TurretAngleTracker::angleDegWrapped360() const {
  return wrapDeg360_(angleDegSigned());
}

