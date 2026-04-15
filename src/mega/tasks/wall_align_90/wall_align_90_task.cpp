#include "tasks/wall_align_90/wall_align_90_task.h"

#include <math.h>

namespace {
const float    kTargetDistanceCm   = 15.0f;
const float    kNoWallThresholdCm  = 100.0f;
const float    kToleranceCm        = 2.0f;
const float    kMinValidCm         = 5.0f;
const float    kMaxValidCm         = 800.0f;
const float    kDriveSpeed         = 0.5f;
const float    kTurnSpeed          = 0.35f;
const uint32_t kDriveTimeoutMs     = 8000;
const uint32_t kTurnTimeoutMs      = 8000;
const float    kTurnRightDeg       = 90.0f;
}  // namespace

WallAlign90Task::WallAlign90Task()
: state_(State::Idle),
  drive_(),
  turn_(),
  ui_(),
  driveActive_(false) {
  DriveByDistance::Config dcfg = drive_.config();
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

void WallAlign90Task::begin(float headingDegContinuous, float avgTravelCm) {
  ui_.begin();
  ui_.showRunning(0.0f, WallAlign90UI::Phase::Check);
  drive_.reset();
  turn_.reset();
  driveActive_ = false;
  (void)headingDegContinuous;
  (void)avgTravelCm;
  setState_(State::CheckFirstWall);
}

bool WallAlign90Task::update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ == State::Succeeded || state_ == State::Failed || state_ == State::Cancelled) return true;

  const bool valid =
      isfinite(lidarAvgCm) && lidarAvgCm >= kMinValidCm && lidarAvgCm <= kMaxValidCm;

  switch (state_) {
    case State::CheckFirstWall: {
      ui_.showRunning(lidarAvgCm, WallAlign90UI::Phase::Check);
      if (!valid) {
        ui_.showFailed("Invalid LiDAR");
        setState_(State::Failed);
        return true;
      }
      if (lidarAvgCm > kNoWallThresholdCm) {
        ui_.showFailed("No wall");
        setState_(State::Failed);
        return true;
      }
      startDrive_(headingDegContinuous, avgTravelCm);
      setState_(State::ApproachFirstWall);
      return false;
    }

    case State::ApproachFirstWall:
    case State::ApproachSecondWall: {
      ui_.showRunning(lidarAvgCm,
                      state_ == State::ApproachFirstWall ? WallAlign90UI::Phase::Approach1
                                                          : WallAlign90UI::Phase::Approach2);
      if (!valid) {
        drive_.cancel();
        ui_.showFailed("Invalid LiDAR");
        setState_(State::Failed);
        return true;
      }
      if (lidarAvgCm > kNoWallThresholdCm) {
        drive_.cancel();
        ui_.showFailed("No wall");
        setState_(State::Failed);
        return true;
      }

      const float error = lidarAvgCm - kTargetDistanceCm;
      if (fabsf(error) <= kToleranceCm) {
        drive_.cancel();
        driveActive_ = false;
        if (state_ == State::ApproachFirstWall) {
          ui_.showRunning(lidarAvgCm, WallAlign90UI::Phase::Turn);
          turn_.begin(headingDegContinuous, kTurnRightDeg, kTurnSpeed);
          setState_(State::TurnRight90);
          return false;
        }
        ui_.showRunning(lidarAvgCm, WallAlign90UI::Phase::Done);
        setState_(State::Succeeded);
        return true;
      }

      const float signedSpeed = (error >= 0.0f) ? kDriveSpeed : -kDriveSpeed;
      drive_.setRequestedSpeed(signedSpeed);

      const bool done = drive_.update(headingDegContinuous, avgTravelCm);
      if (done && drive_.timedOut()) {
        setState_(State::Failed);
        return true;
      }
      return false;
    }

    case State::TurnRight90: {
      ui_.showRunning(lidarAvgCm, WallAlign90UI::Phase::Turn);
      const bool done = turn_.update(headingDegContinuous);
      if (!done) return false;
      if (turn_.timedOut() || !turn_.succeeded()) {
        ui_.showFailed("Turn failed");
        setState_(State::Failed);
        return true;
      }
      startDrive_(headingDegContinuous, avgTravelCm);
      setState_(State::ApproachSecondWall);
      return false;
    }

    default:
      return true;
  }
}

void WallAlign90Task::cancel() {
  drive_.cancel();
  turn_.cancel();
  driveActive_ = false;
  ui_.showFailed("Cancelled");
  setState_(State::Cancelled);
}

void WallAlign90Task::reset() {
  drive_.reset();
  turn_.reset();
  driveActive_ = false;
  ui_.begin();
  ui_.showIdle();
  setState_(State::Idle);
}

bool WallAlign90Task::active() const { return state_ == State::ApproachFirstWall ||
                                              state_ == State::ApproachSecondWall ||
                                              state_ == State::TurnRight90 ||
                                              state_ == State::CheckFirstWall; }

WallAlign90Task::State WallAlign90Task::state() const { return state_; }

void WallAlign90Task::setState_(State s) {
  state_ = s;
}

void WallAlign90Task::startDrive_(float headingDegContinuous, float avgTravelCm) {
  if (!driveActive_) {
    drive_.beginContinuous(headingDegContinuous, avgTravelCm, 0.0f);
    drive_.setHeadingHoldDeg(headingDegContinuous);
    driveActive_ = true;
  }
}
