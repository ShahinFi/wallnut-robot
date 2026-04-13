#include "tasks/follow_distance/follow_distance_task.h"

#include <math.h>

namespace {
const float kDefaultTargetCm   = 30.0f;
const float kToleranceCm       = 2.0f;
const float kMinValidCm        = 5.0f;
const float kMaxValidCm        = 800.0f;
const float kFollowSpeed       = 0.50f;
const uint32_t kTimeoutMs      = 0xFFFFFFFFUL;
static float computeSpeedFromError(float errorAbs) {
  if (errorAbs <= 0.0f) return 0.0f;
  return kFollowSpeed;
}
}  // namespace

FollowDistanceTask::FollowDistanceTask()
: targetDistanceCm_(kDefaultTargetCm),
  state_(State::Idle),
  drive_(),
  ui_(),
  lastErrorCm_(0.0f) {
  DriveStraight::Config cfg = drive_.config();
  cfg.distanceToleranceCm = kToleranceCm;
  cfg.slowDownCm          = 0.0f;
  cfg.minSpeed            = kFollowSpeed;
  cfg.maxSpeed            = kFollowSpeed;
  cfg.timeoutMs           = kTimeoutMs;
  drive_.setConfig(cfg);
}

void FollowDistanceTask::setTargetDistanceCm(float targetDistanceCm) {
  if (!isfinite(targetDistanceCm) || targetDistanceCm <= 0.0f) return;
  targetDistanceCm_ = targetDistanceCm;
}

float FollowDistanceTask::targetDistanceCm() const {
  return targetDistanceCm_;
}

void FollowDistanceTask::begin(float headingDegContinuous, float avgTravelCm) {
  ui_.begin();
  ui_.showRunning(targetDistanceCm_, 0.0f, 0.0f,
                  headingDegContinuous, headingDegContinuous,
                  FollowDistanceUI::FollowStatus::Hold);

  drive_.reset();
  drive_.begin(headingDegContinuous, avgTravelCm, -1.0f, 0.0f);
  drive_.setHeadingHoldDeg(headingDegContinuous);
  headingHoldDeg_ = headingDegContinuous;

  lastErrorCm_ = 0.0f;
  state_ = State::Running;
}

bool FollowDistanceTask::update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ != State::Running) return true;

  FollowDistanceUI::FollowStatus status = FollowDistanceUI::FollowStatus::Hold;

  if (!isfinite(lidarAvgCm) || lidarAvgCm < kMinValidCm || lidarAvgCm > kMaxValidCm) {
    drive_.setRequestedSpeed(0.0f);
    status = FollowDistanceUI::FollowStatus::Invalid;
    lastErrorCm_ = 0.0f;
  } else {
    const float error = lidarAvgCm - targetDistanceCm_;
    lastErrorCm_ = error;

    const float errorAbs = fabsf(error);
    if (errorAbs <= kToleranceCm) {
      drive_.setRequestedSpeed(0.0f);
      status = FollowDistanceUI::FollowStatus::Hold;
    } else {
      const float speed = computeSpeedFromError(errorAbs);
      const float signedSpeed = (error >= 0.0f) ? speed : -speed;
      drive_.setRequestedSpeed(signedSpeed);
      status = (error >= 0.0f) ? FollowDistanceUI::FollowStatus::Forward
                               : FollowDistanceUI::FollowStatus::Backward;
    }
  }

  const bool done = drive_.update(headingDegContinuous, avgTravelCm);
  if (done && drive_.timedOut()) {
    state_ = State::TimedOut;
    return true;
  }

  ui_.showRunning(targetDistanceCm_, lidarAvgCm, lastErrorCm_,
                  headingHoldDeg_, headingDegContinuous, status);

  return false;
}

void FollowDistanceTask::cancel() {
  drive_.cancel();
  ui_.showIdle(targetDistanceCm_);
  state_ = State::Cancelled;
}

void FollowDistanceTask::reset() {
  drive_.reset();
  ui_.begin();
  ui_.showIdle(targetDistanceCm_);
  lastErrorCm_ = 0.0f;
  state_ = State::Idle;
}

bool FollowDistanceTask::active() const { return state_ == State::Running; }
FollowDistanceTask::State FollowDistanceTask::state() const { return state_; }
