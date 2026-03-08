#include "tasks/sequence_executor/sequence_executor_task.h"

#include <math.h>

namespace {
const float    kToleranceCm    = 2.0f;
const float    kMinValidCm     = 5.0f;
const float    kMaxValidCm     = 800.0f;
const float    kMoveSpeed      = 0.9f;
const float    kTurnSpeed      = 0.6f;
const uint32_t kStepTimeoutMs  = 300000; // 5 minutes per step

// Obstacle-aware MOVE speed policy (ESP MOVE only, TURN unaffected)
const float kStopDistanceCm  = 10.0f;
const float kSlowDistanceCm  = 30.0f;
const float kSlowSpeedScale  = 0.30f;
const float kFastSpeedScale  = 0.70f;
const float kGuardTurnDeg    = 90.0f;
const uint8_t kGuardMaxTurns = 4;

static float computeMoveSpeedScale(float lidarAvgCm) {
  if (!isfinite(lidarAvgCm)) return 0.0f;
  if (lidarAvgCm < kStopDistanceCm) return 0.0f;
  if (lidarAvgCm < kSlowDistanceCm) return kSlowSpeedScale;
  return kFastSpeedScale;
}
}

SequenceExecutorTask::SequenceExecutorTask()
: steps_(nullptr),
  stepIndex_(0),
  totalSteps_(0),
  state_(State::Idle),
  drive_(),
  turn_(),
  ui_(),
  driveActive_(false),
  targetCm_(0.0f),
  moveByCm_(0.0f),
  stepStartMs_(0),
  totalDrivenCm_(0.0f),
  lastAvgCm_(0.0f),
  alignEnabled_(false),
  aligning_(false),
  alignHeadingDeg_(0.0f),
  moveGuardState_(MoveGuardState::None),
  moveGuardTurnsDone_(0) {
  DriveStraight::Config dcfg = drive_.config();
  dcfg.slowDownCm = 0.0f;
  dcfg.minSpeed   = kMoveSpeed;
  dcfg.maxSpeed   = kMoveSpeed;
  drive_.setConfig(dcfg);
}

void SequenceExecutorTask::setSequence(const SequenceStep* steps) {
  steps_ = steps;
  totalSteps_ = 0;  // no max steps; optional total display only
}

void SequenceExecutorTask::setAlignHeading(float headingDegContinuous) {
  alignHeadingDeg_ = headingDegContinuous;
  alignEnabled_ = true;
}

void SequenceExecutorTask::clearAlignHeading() {
  alignEnabled_ = false;
  aligning_ = false;
}

void SequenceExecutorTask::begin(float headingDegContinuous, float avgTravelCm) {
  if (!steps_) return;
  ui_.begin();
  ui_.showIdle();
  drive_.reset();
  turn_.reset();
  driveActive_ = false;
  moveGuardState_ = MoveGuardState::None;
  moveGuardTurnsDone_ = 0;
  stepIndex_ = 0;
  totalDrivenCm_ = 0.0f;
  lastAvgCm_ = avgTravelCm;
  setState_(State::Running);
  if (alignEnabled_) {
    const float delta = wrapDegDiff180_(alignHeadingDeg_, headingDegContinuous);
    turn_.begin(headingDegContinuous, delta, kTurnSpeed);
    stepStartMs_ = millis();
    aligning_ = true;
  } else {
    startStep_(headingDegContinuous, avgTravelCm);
  }
}

bool SequenceExecutorTask::update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ != State::Running) return true;

  if (!steps_) {
    setState_(State::Failed);
    return true;
  }

  if (aligning_) {
    ui_.showRunning(lidarAvgCm, totalDrivenCm_, 0, totalSteps_, SequenceExecutorUI::StepLabel::Turn);
    if (stepTimedOut_()) {
      ui_.showFailed("Align timeout");
      setState_(State::Failed);
      return true;
    }
    const bool done = turn_.update(headingDegContinuous);
    if (!done) return false;
    if (turn_.timedOut() || !turn_.succeeded()) {
      ui_.showFailed("Align failed");
      setState_(State::Failed);
      return true;
    }
    aligning_ = false;
    startStep_(headingDegContinuous, avgTravelCm);
    return false;
  }

  const SequenceStep& step = steps_[stepIndex_];
  if (step.type == SequenceStepType::End) {
    ui_.showRunning(lidarAvgCm, totalDrivenCm_, stepIndex_ + 1, totalSteps_,
                    SequenceExecutorUI::StepLabel::End);
    setState_(State::Succeeded);
    return true;
  }

  if (stepTimedOut_()) {
    ui_.showFailed("Step timeout");
    setState_(State::Failed);
    return true;
  }

  const bool inMove = (step.type == SequenceStepType::MoveToDistance ||
                       step.type == SequenceStepType::MoveByDistance);
  if (inMove && avgTravelCm >= lastAvgCm_) {
    totalDrivenCm_ += (avgTravelCm - lastAvgCm_);
  }
  lastAvgCm_ = avgTravelCm;
  if (inMove) {
    if (handleMoveGuard_(headingDegContinuous, avgTravelCm, lidarAvgCm)) {
      return state_ != State::Running;
    }
  }

  if (step.type == SequenceStepType::MoveToDistance) {
    ui_.showRunning(lidarAvgCm, totalDrivenCm_, stepIndex_ + 1, totalSteps_,
                    SequenceExecutorUI::StepLabel::Move);
    const bool valid = isfinite(lidarAvgCm) && lidarAvgCm >= kMinValidCm && lidarAvgCm <= kMaxValidCm;
    if (!valid) {
      drive_.cancel();
      ui_.showFailed("Invalid LiDAR");
      setState_(State::Failed);
      return true;
    }
    const float error = lidarAvgCm - targetCm_;
    if (fabsf(error) <= kToleranceCm) {
      drive_.cancel();
      driveActive_ = false;
      ++stepIndex_;
      startStep_(headingDegContinuous, avgTravelCm);
      return false;
    }
    const float signedSpeed = (error >= 0.0f) ? kMoveSpeed : -kMoveSpeed;
    const float scaled = signedSpeed * computeMoveSpeedScale(lidarAvgCm);
    drive_.setRequestedSpeed(scaled);
    drive_.update(headingDegContinuous, avgTravelCm);
    return false;
  }

  if (step.type == SequenceStepType::MoveByDistance) {
    ui_.showRunning(lidarAvgCm, totalDrivenCm_, stepIndex_ + 1, totalSteps_,
                    SequenceExecutorUI::StepLabel::Move);
    const float scale = computeMoveSpeedScale(lidarAvgCm);
    const float baseSpeed = (moveByCm_ >= 0.0f) ? kMoveSpeed : -kMoveSpeed;
    drive_.setRequestedSpeed(baseSpeed * scale);
    const bool done = drive_.update(headingDegContinuous, avgTravelCm);
    if (done) {
      if (drive_.timedOut()) {
        ui_.showFailed("Drive timeout");
        setState_(State::Failed);
        return true;
      }
      driveActive_ = false;
      ++stepIndex_;
      startStep_(headingDegContinuous, avgTravelCm);
    }
    return false;
  }

  if (step.type == SequenceStepType::TurnDeg) {
    ui_.showRunning(lidarAvgCm, totalDrivenCm_, stepIndex_ + 1, totalSteps_,
                    SequenceExecutorUI::StepLabel::Turn);
    const bool done = turn_.update(headingDegContinuous);
    if (!done) return false;
    if (turn_.timedOut() || !turn_.succeeded()) {
      ui_.showFailed("Turn failed");
      setState_(State::Failed);
      return true;
    }
    ++stepIndex_;
    startStep_(headingDegContinuous, avgTravelCm);
    return false;
  }

  return false;
}

void SequenceExecutorTask::cancel() {
  drive_.cancel();
  turn_.cancel();
  driveActive_ = false;
  ui_.showFailed("Cancelled");
  setState_(State::Cancelled);
}

void SequenceExecutorTask::reset() {
  drive_.reset();
  turn_.reset();
  driveActive_ = false;
  moveGuardState_ = MoveGuardState::None;
  moveGuardTurnsDone_ = 0;
  ui_.begin();
  ui_.showIdle();
  setState_(State::Idle);
}

bool SequenceExecutorTask::active() const { return state_ == State::Running; }
SequenceExecutorTask::State SequenceExecutorTask::state() const { return state_; }

void SequenceExecutorTask::setState_(State s) {
  state_ = s;
}

void SequenceExecutorTask::startStep_(float headingDegContinuous, float avgTravelCm) {
  if (!steps_) return;
  stepStartMs_ = millis();
  const SequenceStep& step = steps_[stepIndex_];
  if (step.type == SequenceStepType::MoveToDistance) {
    targetCm_ = step.value;
    startMove_(headingDegContinuous, avgTravelCm);
  } else if (step.type == SequenceStepType::MoveByDistance) {
    moveByCm_ = step.value;
    startMove_(headingDegContinuous, avgTravelCm);
  } else if (step.type == SequenceStepType::TurnDeg) {
    startTurn_(headingDegContinuous);
  }
}

void SequenceExecutorTask::startMove_(float headingDegContinuous, float avgTravelCm) {
  if (!driveActive_) {
    const SequenceStep& step = steps_[stepIndex_];
    if (step.type == SequenceStepType::MoveByDistance) {
      const float dist = fabsf(moveByCm_);
      const float dir = (moveByCm_ >= 0.0f) ? 1.0f : -1.0f;
      drive_.begin(headingDegContinuous, avgTravelCm, dist, dir * kMoveSpeed);
    } else {
      drive_.begin(headingDegContinuous, avgTravelCm, -1.0f, 0.0f);
    }
    drive_.setHeadingHoldDeg(headingDegContinuous);
    driveActive_ = true;
  }
}

void SequenceExecutorTask::startTurn_(float headingDegContinuous) {
  turn_.begin(headingDegContinuous, steps_[stepIndex_].value, kTurnSpeed);
}

bool SequenceExecutorTask::stepTimedOut_() const {
  return (millis() - stepStartMs_) >= kStepTimeoutMs;
}

float SequenceExecutorTask::wrapDegDiff180_(float targetDeg, float currentDeg) {
  float d = targetDeg - currentDeg;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

bool SequenceExecutorTask::handleMoveGuard_(float headingDegContinuous,
                                            float avgTravelCm,
                                            float lidarAvgCm) {
  if (lidarAvgCm >= kStopDistanceCm) {
    moveGuardState_ = MoveGuardState::None;
    moveGuardTurnsDone_ = 0;
    return false;
  }

  if (moveGuardState_ == MoveGuardState::None) {
    drive_.cancel();
    driveActive_ = false;
    turn_.begin(headingDegContinuous, kGuardTurnDeg, kTurnSpeed);
    moveGuardState_ = MoveGuardState::Turning;
    return true;
  }

  const bool done = turn_.update(headingDegContinuous);
  if (!done) return true;

  if (turn_.timedOut() || !turn_.succeeded()) {
    ui_.showFailed("Guard turn fail");
    setState_(State::Failed);
    return true;
  }

  moveGuardTurnsDone_++;
  if (lidarAvgCm >= kStopDistanceCm) {
    moveGuardState_ = MoveGuardState::None;
    moveGuardTurnsDone_ = 0;
    return false;
  }

  if (moveGuardTurnsDone_ >= kGuardMaxTurns) {
    ui_.showFailed("Blocked 360");
    setState_(State::Failed);
    return true;
  }

  turn_.begin(headingDegContinuous, kGuardTurnDeg, kTurnSpeed);
  return true;
}
