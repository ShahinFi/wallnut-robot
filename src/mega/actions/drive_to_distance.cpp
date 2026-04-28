#include "actions/drive_to_distance.h"

#include <math.h>

namespace {
static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}
}  // namespace

DriveToDistance::DriveToDistance()
: cfg_(),
  state_(State::Idle),
  drive_(),
  targetLidarCm_(0.0f),
  speedAbs_(0.0f),
  errorCm_(0.0f),
  startMs_(0) {}

void DriveToDistance::setConfig(const Config& cfg) { cfg_ = cfg; }
const DriveToDistance::Config& DriveToDistance::config() const { return cfg_; }

void DriveToDistance::setDriveConfig(const DriveByDistance::Config& cfg) {
  drive_.setConfig(cfg);
}
const DriveByDistance::Config& DriveToDistance::driveConfig() const {
  return drive_.config();
}

void DriveToDistance::begin(float headingDegContinuous, float avgTravelCm,
                            float targetLidarCm, float speedAbs) {
  reset();

  targetLidarCm_ = targetLidarCm;
  speedAbs_ = clamp01(fabsf(speedAbs));
  errorCm_ = 0.0f;

  // WHY: Start moving in "continuous" mode; direction is decided each update from LiDAR error.
  drive_.beginContinuous(headingDegContinuous, avgTravelCm, 0.0f);
  drive_.setHeadingHoldDeg(headingDegContinuous);

  startMs_ = millis();
  setState_(State::Running);
}

void DriveToDistance::setSpeedAbs(float speedAbs) {
  speedAbs_ = clamp01(fabsf(speedAbs));
}

bool DriveToDistance::update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ != State::Running) return true;

  const uint32_t now = millis();
  if (now - startMs_ >= cfg_.timeoutMs) {
    drive_.cancel();
    setState_(State::TimedOut);
    return true;
  }

  const bool valid = isfinite(lidarAvgCm) && lidarAvgCm >= cfg_.minValidCm && lidarAvgCm <= cfg_.maxValidCm;
  if (!valid || !isfinite(targetLidarCm_) || targetLidarCm_ <= 0.0f) {
    drive_.cancel();
    setState_(State::Failed);
    return true;
  }

  errorCm_ = lidarAvgCm - targetLidarCm_;
  if (fabsf(errorCm_) <= cfg_.toleranceCm) {
    drive_.cancel();
    setState_(State::Succeeded);
    return true;
  }

  const float signedSpeed = (errorCm_ >= 0.0f) ? speedAbs_ : -speedAbs_;
  drive_.setRequestedSpeed(signedSpeed);
  drive_.update(headingDegContinuous, avgTravelCm);
  return false;
}

void DriveToDistance::cancel() {
  drive_.cancel();
  setState_(State::Cancelled);
}

void DriveToDistance::reset() {
  drive_.reset();
  targetLidarCm_ = 0.0f;
  speedAbs_ = 0.0f;
  errorCm_ = 0.0f;
  startMs_ = 0;
  setState_(State::Idle);
}

bool DriveToDistance::active() const { return state_ == State::Running; }
bool DriveToDistance::succeeded() const { return state_ == State::Succeeded; }
bool DriveToDistance::timedOut() const { return state_ == State::TimedOut; }
bool DriveToDistance::failed() const { return state_ == State::Failed; }
DriveToDistance::State DriveToDistance::state() const { return state_; }

float DriveToDistance::targetDistanceCm() const { return targetLidarCm_; }
float DriveToDistance::errorCm() const { return errorCm_; }

void DriveToDistance::setState_(State s) { state_ = s; }
