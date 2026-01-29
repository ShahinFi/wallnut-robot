#include "tasks/wall_sequence/wall_sequence_task.h"

#include <math.h>

namespace {
const float    kTarget30Cm        = 30.0f;
const float    kTarget15Cm        = 15.0f;
const float    kToleranceCm       = 2.0f;
const float    kMinValidCm        = 5.0f;
const float    kMaxValidCm        = 800.0f;
const float    kDriveSpeed        = 0.5f;
const float    kTurnSpeed         = 0.35f;
const uint32_t kDriveTimeoutMs    = 120000;
const uint32_t kTurnTimeoutMs     = 120000;
const float    kTurnLeftDeg       = -90.0f;
}  // namespace

WallSequenceTask::WallSequenceTask()
: state_(State::Idle),
  drive_(),
  turn_(),
  ui_(),
  driveActive_(false),
  targetCm_(kTarget30Cm),
  startAvgCm_(0.0f),
  totalDrivenCm_(0.0f),
  lastAvgCm_(0.0f) {
  DriveStraight::Config dcfg = drive_.config();
  dcfg.distanceToleranceCm = kToleranceCm;
  dcfg.slowDownCm          = 0.0f;
  dcfg.minSpeed            = kDriveSpeed;
  dcfg.maxSpeed            = kDriveSpeed;
  dcfg.timeoutMs           = kDriveTimeoutMs;
  drive_.setConfig(dcfg);

  TurnToAngle::Config tcfg = turn_.config();
  tcfg.timeoutMs = kTurnTimeoutMs;
  turn_.setConfig(tcfg);
}

void WallSequenceTask::begin(float headingDegContinuous, float avgTravelCm) {
  ui_.begin();
  ui_.showIdle();
  drive_.reset();
  turn_.reset();
  driveActive_ = false;
  startAvgCm_ = avgTravelCm;
  totalDrivenCm_ = 0.0f;
  lastAvgCm_ = avgTravelCm;
  startApproach_(headingDegContinuous, avgTravelCm, kTarget30Cm);
  setState_(State::Approach30_1);
}

bool WallSequenceTask::update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ == State::Succeeded || state_ == State::Failed || state_ == State::Cancelled) return true;

  const bool inApproach =
      state_ == State::Approach30_1 || state_ == State::Approach15_2 ||
      state_ == State::Approach30_3 || state_ == State::Approach15_4;

  if (inApproach && avgTravelCm >= lastAvgCm_) {
    totalDrivenCm_ += (avgTravelCm - lastAvgCm_);
  }
  lastAvgCm_ = avgTravelCm;

  switch (state_) {
    case State::Approach30_1:
      ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::Approach30_1);
      return approachUpdate_(headingDegContinuous, avgTravelCm, lidarAvgCm,
                             kTarget30Cm, State::TurnLeft_1);

    case State::TurnLeft_1: {
      ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::TurnLeft_1);
      const bool done = turn_.update(headingDegContinuous);
      if (!done) return false;
      if (turn_.timedOut() || !turn_.succeeded()) {
        ui_.showFailed("Turn failed");
        setState_(State::Failed);
        return true;
      }
      startApproach_(headingDegContinuous, avgTravelCm, kTarget15Cm);
      setState_(State::Approach15_2);
      return false;
    }

    case State::Approach15_2:
      ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::Approach15_2);
      return approachUpdate_(headingDegContinuous, avgTravelCm, lidarAvgCm,
                             kTarget15Cm, State::TurnLeft_2);

    case State::TurnLeft_2: {
      ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::TurnLeft_2);
      const bool done = turn_.update(headingDegContinuous);
      if (!done) return false;
      if (turn_.timedOut() || !turn_.succeeded()) {
        ui_.showFailed("Turn failed");
        setState_(State::Failed);
        return true;
      }
      startApproach_(headingDegContinuous, avgTravelCm, kTarget30Cm);
      setState_(State::Approach30_3);
      return false;
    }

    case State::Approach30_3:
      ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::Approach30_3);
      return approachUpdate_(headingDegContinuous, avgTravelCm, lidarAvgCm,
                             kTarget30Cm, State::TurnLeft_3);

    case State::TurnLeft_3: {
      ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::TurnLeft_3);
      const bool done = turn_.update(headingDegContinuous);
      if (!done) return false;
      if (turn_.timedOut() || !turn_.succeeded()) {
        ui_.showFailed("Turn failed");
        setState_(State::Failed);
        return true;
      }
      startApproach_(headingDegContinuous, avgTravelCm, kTarget15Cm);
      setState_(State::Approach15_4);
      return false;
    }

    case State::Approach15_4:
      ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::Approach15_4);
      if (approachUpdate_(headingDegContinuous, avgTravelCm, lidarAvgCm,
                          kTarget15Cm, State::Succeeded)) {
        ui_.showRunning(lidarAvgCm, totalDrivenCm_, WallSequenceUI::Phase::Done);
        setState_(State::Succeeded);
        return true;
      }
      return false;

    default:
      return true;
  }
}

void WallSequenceTask::cancel() {
  drive_.cancel();
  turn_.cancel();
  driveActive_ = false;
  ui_.showFailed("Cancelled");
  setState_(State::Cancelled);
}

void WallSequenceTask::reset() {
  drive_.reset();
  turn_.reset();
  driveActive_ = false;
  ui_.begin();
  ui_.showIdle();
  setState_(State::Idle);
}

bool WallSequenceTask::active() const {
  return state_ == State::Approach30_1 || state_ == State::TurnLeft_1 ||
         state_ == State::Approach15_2 || state_ == State::TurnLeft_2 ||
         state_ == State::Approach30_3 || state_ == State::TurnLeft_3 ||
         state_ == State::Approach15_4;
}

WallSequenceTask::State WallSequenceTask::state() const { return state_; }

void WallSequenceTask::setState_(State s) {
  state_ = s;
}

void WallSequenceTask::startApproach_(float headingDegContinuous, float avgTravelCm, float targetCm) {
  targetCm_ = targetCm;
  if (!driveActive_) {
    drive_.begin(headingDegContinuous, avgTravelCm, -1.0f, 0.0f);
    drive_.setHeadingHoldDeg(headingDegContinuous);
    driveActive_ = true;
  }
}

bool WallSequenceTask::approachUpdate_(float headingDegContinuous, float avgTravelCm, float lidarAvgCm,
                                      float targetCm, State nextAfterTurn) {
  const bool valid = isfinite(lidarAvgCm) && lidarAvgCm >= kMinValidCm && lidarAvgCm <= kMaxValidCm;
  if (!valid) {
    drive_.cancel();
    ui_.showFailed("Invalid LiDAR");
    setState_(State::Failed);
    return true;
  }

  const float error = lidarAvgCm - targetCm;
  if (fabsf(error) <= kToleranceCm) {
    drive_.cancel();
    driveActive_ = false;
    if (nextAfterTurn == State::Succeeded) return true;
    turn_.begin(headingDegContinuous, kTurnLeftDeg, kTurnSpeed);
    setState_(nextAfterTurn);
    return false;
  }

  const float signedSpeed = (error >= 0.0f) ? kDriveSpeed : -kDriveSpeed;
  drive_.setRequestedSpeed(signedSpeed);

  const bool done = drive_.update(headingDegContinuous, avgTravelCm);
  if (done && drive_.timedOut()) {
    ui_.showFailed("Drive timeout");
    setState_(State::Failed);
    return true;
  }
  return false;
}
