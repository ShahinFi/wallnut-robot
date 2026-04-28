#include "lidar/turret/actions/turret_sweep_scan_360.h"

#include "lidar/lidar.h"
#include "lidar/turret/turret_motor.h"
#include "lidar/turret/turret_angle_tracker.h"

#include <math.h>

TurretSweepScan360::TurretSweepScan360()
: cfg_{},
  state_(State::Idle),
  motor_(nullptr),
  angle_(nullptr),
  lidar_(nullptr),
  dirSign_(1),
  startTicksAbs_(0),
  lastSampleTicksAbs_(0),
  startAngleDegWrapped_(0.0f),
  ticksPerRev_(0),
  sampleEveryTicks_(1),
  seq_(0),
  lastCmd_(0.0f),
  startMs_(0),
  lidarInFlight_(false),
  lidarTicksStart_(0),
  finishGraceStartMs_(0),
  cb_(nullptr),
  cbUser_(nullptr) {}

void TurretSweepScan360::setConfig(const Config& cfg) {
  cfg_ = cfg;
  if (!isfinite(cfg_.cmdAbs) || cfg_.cmdAbs < 0.0f) cfg_.cmdAbs = 0.0f;
  if (cfg_.cmdAbs > 1.0f) cfg_.cmdAbs = 1.0f;
  if (cfg_.timeoutMs < 1000U) cfg_.timeoutMs = 1000U;
  // WHY: 0 means "auto" (computed from targetSamplesPerRev).
  // WHY: Otherwise it's an explicit tick-domain downsampling factor.
  if (cfg_.targetSamplesPerRev == 0) cfg_.targetSamplesPerRev = 1;
}

const TurretSweepScan360::Config& TurretSweepScan360::config() const { return cfg_; }

void TurretSweepScan360::setSampleCallback(SampleCallback cb, void* user) {
  cb_ = cb;
  cbUser_ = user;
}

void TurretSweepScan360::begin(TurretMotor* motor, TurretAngleTracker* angleTracker,
                               Lidar* lidar, int dirSign, long ticksAbsNow, uint32_t nowMs) {
  motor_ = motor;
  angle_ = angleTracker;
  lidar_ = lidar;
  dirSign_ = (dirSign < 0) ? -1 : 1;

  if (!motor_ || !angle_ || !lidar_) {
    state_ = State::Cancelled;
    return;
  }

  ticksPerRev_ = angle_->ticksPerRevForDirSign(dirSign_);
  if (ticksPerRev_ == 0) {
    state_ = State::NoCalibration;
    stopMotor_();
    return;
  }

  // WHY: Sampling cadence:
  // WHY: - if explicitly set, use it
  // WHY: - else auto-select ~targetSamplesPerRev per revolution.
  if (cfg_.sampleEveryTicks > 0) {
    sampleEveryTicks_ = (uint32_t)cfg_.sampleEveryTicks;
  } else {
    sampleEveryTicks_ = ticksPerRev_ / (uint32_t)cfg_.targetSamplesPerRev;
    if (sampleEveryTicks_ == 0) sampleEveryTicks_ = 1;
  }

  state_ = State::Running;
  startMs_ = nowMs;
  startTicksAbs_ = ticksAbsNow;
  startAngleDegWrapped_ = angle_->angleDegWrapped360();
  // WHY: Force the first update() call to emit a sample immediately with a real LiDAR value.
  lastSampleTicksAbs_ = ticksAbsNow - (long)sampleEveryTicks_;
  seq_ = 0;
  lidarInFlight_ = false;
  lidarTicksStart_ = 0;
  finishGraceStartMs_ = 0;

  lastCmd_ = (float)dirSign_ * cfg_.cmdAbs;
  motor_->setCmd(lastCmd_);

  // WHY: Ensure scan owns the LiDAR pipeline (no leftover in-flight measurement from non-scan loop).
  lidar_->abortRange();
  if (lidar_->startRange()) {
    lidarInFlight_ = true;
    lidarTicksStart_ = ticksAbsNow;
  }
}

static float wrap360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

bool TurretSweepScan360::update(long ticksAbsNow, uint32_t nowMs) {
  if (state_ != State::Running) return true;
  if (!motor_ || !angle_ || !lidar_) {
    state_ = State::Cancelled;
    return true;
  }

  const long dt = ticksAbsNow - startTicksAbs_;
  if (dt < 0) {
    // WHY: Absolute ticks should be monotonic; treat as a hard reset event.
    stopMotor_();
    state_ = State::Cancelled;
    return true;
  }

  const bool reachedRev = ((uint32_t)dt >= ticksPerRev_);
  if (reachedRev && finishGraceStartMs_ == 0) {
    // WHY: Stop rotation at 1 rev; allow a short grace period to collect the in-flight LiDAR sample.
    stopMotor_();
    finishGraceStartMs_ = nowMs;
  }

  if ((uint32_t)(nowMs - startMs_) >= cfg_.timeoutMs) {
    stopMotor_();
    state_ = State::TimedOut;
    return true;
  }

  // WHY: LiDAR measurement pipeline:
  if (!lidarInFlight_) {
    if (lidar_->startRange()) {
      lidarInFlight_ = true;
      lidarTicksStart_ = ticksAbsNow;
    }
  }

  float distCm = 0.0f;
  if (lidarInFlight_ && lidar_->pollRange(distCm)) {
    lidarInFlight_ = false;

    const long ticksEnd = ticksAbsNow;
    const long ticksMid = (lidarTicksStart_ + ticksEnd) / 2L;

    if ((ticksMid - lastSampleTicksAbs_) >= (long)sampleEveryTicks_) {
      emitSample_(ticksMid, distCm, nowMs);
      lastSampleTicksAbs_ = ticksMid;
    }
  }

  if (reachedRev) {
    // WHY: Finish once we're past 1 rev and there's no in-flight measurement,
    // WHY: or after a small grace period.
    const uint32_t kGraceMs = 300;
    if (!lidarInFlight_) {
      state_ = State::Succeeded;
      return true;
    }
    if (finishGraceStartMs_ && (uint32_t)(nowMs - finishGraceStartMs_) >= kGraceMs) {
      state_ = State::Succeeded;
      return true;
    }
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
  lidar_ = nullptr;
  seq_ = 0;
}

bool TurretSweepScan360::active() const { return state_ == State::Running; }
TurretSweepScan360::State TurretSweepScan360::state() const { return state_; }

void TurretSweepScan360::stopMotor_() {
  if (motor_) motor_->stop();
}

void TurretSweepScan360::emitSample_(long ticksAbsNow, float lidarDistanceCm, uint32_t nowMs) {
  if (!cb_ || !angle_) return;

  // WHY: Convert ticks (midpoint-stamped) to angle using the scan's constant direction model.
  // WHY: We intentionally don't query the tracker "at past ticks" (it can't),
  // WHY: and instead use the same calibrated deg/tick for this scan direction.
  const uint32_t tpr = angle_->ticksPerRevForDirSign(dirSign_);
  if (tpr == 0) return;
  const float degPerTick = 360.0f / (float)tpr;
  const float angleSign = (float)angle_->config().angleSign;
  const float deltaDeg = angleSign * (float)dirSign_ * (float)(ticksAbsNow - startTicksAbs_) * degPerTick;
  const float angleDeg = startAngleDegWrapped_ + deltaDeg;

  Sample s;
  s.seq = seq_++;
  s.ticksAbs = ticksAbsNow;
  s.angleDeg = wrap360(angleDeg);
  s.distanceCm = lidarDistanceCm;
  s.ms = nowMs;
  cb_(s, cbUser_);
}
