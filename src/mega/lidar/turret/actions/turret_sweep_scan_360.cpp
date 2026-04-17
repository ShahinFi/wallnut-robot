#include "lidar/turret/actions/turret_sweep_scan_360.h"

#include "lidar/turret/turret_motor.h"
#include "lidar/turret/turret_angle_tracker.h"

#include <math.h>

TurretSweepScan360::TurretSweepScan360()
: cfg_{},
  state_(State::Idle),
  motor_(nullptr),
  angle_(nullptr),
  dirSign_(1),
  startTicksAbs_(0),
  lastSampleTicksAbs_(0),
  ticksPerRev_(0),
  sampleEveryTicks_(1),
  seq_(0),
  lastCmd_(0.0f),
  startMs_(0),
  cb_(nullptr),
  cbUser_(nullptr) {}

void TurretSweepScan360::setConfig(const Config& cfg) {
  cfg_ = cfg;
  if (!isfinite(cfg_.cmdAbs) || cfg_.cmdAbs < 0.0f) cfg_.cmdAbs = 0.0f;
  if (cfg_.cmdAbs > 1.0f) cfg_.cmdAbs = 1.0f;
  if (cfg_.timeoutMs < 1000U) cfg_.timeoutMs = 1000U;
  if (cfg_.targetSamplesPerRev == 0) cfg_.targetSamplesPerRev = 1;
}

const TurretSweepScan360::Config& TurretSweepScan360::config() const { return cfg_; }

void TurretSweepScan360::setSampleCallback(SampleCallback cb, void* user) {
  cb_ = cb;
  cbUser_ = user;
}

void TurretSweepScan360::begin(TurretMotor* motor, TurretAngleTracker* angleTracker,
                               int dirSign, long ticksAbsNow, uint32_t nowMs) {
  motor_ = motor;
  angle_ = angleTracker;
  dirSign_ = (dirSign < 0) ? -1 : 1;

  if (!motor_ || !angle_) {
    state_ = State::Cancelled;
    return;
  }

  ticksPerRev_ = angle_->ticksPerRevForDirSign(dirSign_);
  if (ticksPerRev_ == 0) {
    state_ = State::NoCalibration;
    stopMotor_();
    return;
  }

  // Auto sampling cadence: ~targetSamplesPerRev per revolution.
  sampleEveryTicks_ = ticksPerRev_ / (uint32_t)cfg_.targetSamplesPerRev;
  if (sampleEveryTicks_ == 0) sampleEveryTicks_ = 1;

  state_ = State::Running;
  startMs_ = nowMs;
  startTicksAbs_ = ticksAbsNow;
  // Force the first update() call to emit a sample immediately with a real LiDAR value.
  lastSampleTicksAbs_ = ticksAbsNow - (long)sampleEveryTicks_;
  seq_ = 0;

  lastCmd_ = (float)dirSign_ * cfg_.cmdAbs;
  motor_->setCmd(lastCmd_);
}

bool TurretSweepScan360::update(long ticksAbsNow, float lidarDistanceCm, uint32_t nowMs) {
  if (state_ != State::Running) return true;
  if (!motor_ || !angle_) {
    state_ = State::Cancelled;
    return true;
  }

  if ((uint32_t)(nowMs - startMs_) >= cfg_.timeoutMs) {
    stopMotor_();
    state_ = State::TimedOut;
    return true;
  }

  const long dt = ticksAbsNow - startTicksAbs_;
  if (dt < 0) {
    // Absolute ticks should be monotonic; treat as a hard reset event.
    stopMotor_();
    state_ = State::Cancelled;
    return true;
  }

  if ((uint32_t)dt >= ticksPerRev_) {
    stopMotor_();
    emitSample_(ticksAbsNow, lidarDistanceCm, nowMs);
    state_ = State::Succeeded;
    return true;
  }

  if ((ticksAbsNow - lastSampleTicksAbs_) >= (long)sampleEveryTicks_) {
    emitSample_(ticksAbsNow, lidarDistanceCm, nowMs);
    lastSampleTicksAbs_ = ticksAbsNow;
  }

  return false;
}

void TurretSweepScan360::cancel() {
  if (state_ == State::Running) stopMotor_();
  state_ = State::Cancelled;
}

void TurretSweepScan360::reset() {
  if (state_ == State::Running) stopMotor_();
  state_ = State::Idle;
  motor_ = nullptr;
  angle_ = nullptr;
  seq_ = 0;
}

bool TurretSweepScan360::active() const { return state_ == State::Running; }
TurretSweepScan360::State TurretSweepScan360::state() const { return state_; }

void TurretSweepScan360::stopMotor_() {
  if (motor_) motor_->stop();
}

void TurretSweepScan360::emitSample_(long ticksAbsNow, float lidarDistanceCm, uint32_t nowMs) {
  if (!cb_ || !angle_) return;

  Sample s;
  s.seq = seq_++;
  s.ticksAbs = ticksAbsNow;
  s.angleDeg = angle_->angleDegWrapped360();
  s.distanceCm = lidarDistanceCm;
  s.ms = nowMs;
  cb_(s, cbUser_);
}
