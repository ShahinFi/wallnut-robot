#include "tasks/sequence_executor/sequence_executor_task.h"

#include <math.h>

namespace {
const float    kToleranceCm    = 2.0f;
const float    kMinValidCm     = 5.0f;
const float    kMaxValidCm     = 800.0f;
const float    kMoveSpeed      = 1.0f;
const float    kTurnSpeed      = 0.6f;
const uint32_t kStepTimeoutMs  = 300000; // 5 minutes per step
const uint32_t kMoveRampMs     = 1000;   // 1s accel ramp for straight MoveByDistance

// Obstacle-aware MOVE speed policy (ESP MOVE only, TURN unaffected)
const float kStopDistanceCm  = 10.0f;
const float kClearDistanceCm = 12.0f;
const float kSlowDistanceCm  = 30.0f;
const float kSlowSpeedScale  = 0.35f;
const float kFastSpeedScale  = 0.50f; // default forward speed (can be overridden by color latch)
const float kGuardTurnDeg    = 90.0f;
const uint8_t kGuardMaxTurns = 4;
const uint8_t kGuardClearStreakNeeded = 3;

static float computeMoveSpeedScale(float lidarAvgCm, float forwardScale) {
  float fast = forwardScale;
  if (!isfinite(fast)) fast = kFastSpeedScale;
  if (fast < 0.0f) fast = 0.0f;
  if (fast > 1.0f) fast = 1.0f;
  // LiDAR can be temporarily invalid (startup / sensor glitch / during turret scan
  // transitions). Invalid readings must never stall an in-flight motion command,
  // otherwise the browser will wait for CMDOK until timeout.
  //
  // Safety is still enforced by the dedicated reflex stop logic in main.cpp.
  if (!isfinite(lidarAvgCm) || lidarAvgCm <= 0.0f) return fast;
  if (lidarAvgCm < kStopDistanceCm) return 0.0f;
  if (lidarAvgCm < kSlowDistanceCm) return (fast < kSlowSpeedScale) ? fast : kSlowSpeedScale;
  return fast;
}
}

SequenceExecutorTask::SequenceExecutorTask()
: steps_(nullptr),
  stepIndex_(0),
  totalSteps_(0),
  state_(State::Idle),
  driveBy_(),
  driveTo_(),
  turn_(),
  ui_(),
  driveActive_(false),
  targetCm_(0.0f),
  moveByCm_(0.0f),
  stepStartMs_(0),
  drivenCmAbs_(0.0f),
  alignEnabled_(false),
  aligning_(false),
  alignHeadingDeg_(0.0f),
  forwardSpeedScale_(kFastSpeedScale),
  moveRampStartMs_(0),
  moveRampActive_(false),
  moveGuardState_(MoveGuardState::None),
  moveGuardTurnsDone_(0),
  moveGuardClearStreak_(0) {
  DriveByDistance::Config dcfg = driveBy_.config();
  dcfg.slowDownCm = 0.0f;
  // Allow obstacle-based scaling (30%/70%) instead of clamping to full speed.
  dcfg.minSpeed   = kMoveSpeed * kSlowSpeedScale;
  dcfg.maxSpeed   = kMoveSpeed;
  driveBy_.setConfig(dcfg);
  driveTo_.setDriveConfig(dcfg);

  DriveToDistance::Config tcfg = driveTo_.config();
  tcfg.toleranceCm = kToleranceCm;
  tcfg.minValidCm  = kMinValidCm;
  tcfg.maxValidCm  = kMaxValidCm;
  tcfg.timeoutMs   = kStepTimeoutMs;
  driveTo_.setConfig(tcfg);
}

void SequenceExecutorTask::setSequence(const SequenceStep* steps) {
  steps_ = steps;
  totalSteps_ = 0;  // no max steps; optional total display only
}

void SequenceExecutorTask::setForwardSpeedScale(float scale01) {
  float s = scale01;
  if (!isfinite(s)) return;
  if (s < 0.0f) s = 0.0f;
  if (s > 1.0f) s = 1.0f;
  forwardSpeedScale_ = s;
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
  driveBy_.reset();
  driveTo_.reset();
  turn_.reset();
  driveActive_ = false;
  moveRampStartMs_ = 0;
  moveRampActive_ = false;
  moveGuardState_ = MoveGuardState::None;
  moveGuardTurnsDone_ = 0;
  moveGuardClearStreak_ = 0;
  stepIndex_ = 0;
  drivenCmAbs_ = 0.0f;
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

bool SequenceExecutorTask::update(float headingDegContinuous, float avgTravelCm, float avgTravelCmAbs, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ != State::Running) return true;

  if (!steps_) {
    setState_(State::Failed);
    return true;
  }

  if (aligning_) {
    ui_.showRunning(lidarAvgCm, drivenCmAbs_, 0, totalSteps_, SequenceExecutorUI::StepLabel::Turn);
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
  drivenCmAbs_ = avgTravelCmAbs;
  if (step.type == SequenceStepType::End) {
    ui_.showRunning(lidarAvgCm, drivenCmAbs_, stepIndex_ + 1, totalSteps_,
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
  if (inMove) drivenCmAbs_ = avgTravelCmAbs;
  if (inMove) {
    if (handleMoveGuard_(headingDegContinuous, avgTravelCm, lidarAvgCm)) {
      return state_ != State::Running;
    }
  }

  if (step.type == SequenceStepType::MoveToDistance) {
    ui_.showRunning(lidarAvgCm, drivenCmAbs_, stepIndex_ + 1, totalSteps_,
                    SequenceExecutorUI::StepLabel::Move);
    // Forward motion uses the obstacle-aware scaling; backing up does not.
    const float error = lidarAvgCm - targetCm_;
    const float speedAbs =
        (error >= 0.0f) ? (kMoveSpeed * computeMoveSpeedScale(lidarAvgCm, forwardSpeedScale_))
                        : (kMoveSpeed * kFastSpeedScale);
    if (!driveTo_.active()) {
      driveTo_.begin(headingDegContinuous, avgTravelCm, targetCm_, speedAbs);
    } else {
      driveTo_.setSpeedAbs(speedAbs);
    }
    const bool done = driveTo_.update(headingDegContinuous, avgTravelCm, lidarAvgCm);
    if (done) {
      if (driveTo_.timedOut()) {
        ui_.showFailed("Drive timeout");
        setState_(State::Failed);
        return true;
      }
      if (!driveTo_.succeeded()) {
        ui_.showFailed("Invalid LiDAR");
        setState_(State::Failed);
        return true;
      }
      ++stepIndex_;
      startStep_(headingDegContinuous, avgTravelCm);
    }
    return false;
  }

  if (step.type == SequenceStepType::MoveByDistance) {
    ui_.showRunning(lidarAvgCm, drivenCmAbs_, stepIndex_ + 1, totalSteps_,
                    SequenceExecutorUI::StepLabel::Move);
    if (!driveActive_) {
      startMove_(headingDegContinuous, avgTravelCm);
    }
    // Obstacle-aware scaling is only meaningful for forward motion. For backing
    // up (e.g. reflex backoff), do not let a close-forward wall stall the move.
    const float scale = (moveByCm_ >= 0.0f) ? computeMoveSpeedScale(lidarAvgCm, forwardSpeedScale_) : kFastSpeedScale;
    const float baseSpeed = (moveByCm_ >= 0.0f) ? kMoveSpeed : -kMoveSpeed;

    // Accel ramp for straight moves only (no ramp on stop/decel; turns unaffected).
    float ramp = 1.0f;
    if (moveRampActive_) {
      const uint32_t dt = (uint32_t)(millis() - moveRampStartMs_);
      if (dt < kMoveRampMs) ramp = (float)dt / (float)kMoveRampMs;
    }
    driveBy_.setRequestedSpeed(baseSpeed * scale * ramp);
    const bool done = driveBy_.update(headingDegContinuous, avgTravelCm);
    if (done) {
      if (driveBy_.timedOut()) {
        ui_.showFailed("Drive timeout");
        setState_(State::Failed);
        return true;
      }
      driveActive_ = false;
      moveRampActive_ = false;
      ++stepIndex_;
      startStep_(headingDegContinuous, avgTravelCm);
    }
    return false;
  }

  if (step.type == SequenceStepType::TurnDeg) {
    ui_.showRunning(lidarAvgCm, drivenCmAbs_, stepIndex_ + 1, totalSteps_,
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

bool SequenceExecutorTask::reverseMoveActive() const {
  if (state_ != State::Running) return false;
  if (!steps_) return false;
  const SequenceStep& step = steps_[stepIndex_];
  if (step.type != SequenceStepType::MoveByDistance) return false;
  return moveByCm_ < 0.0f;
}

void SequenceExecutorTask::cancel() {
  driveBy_.cancel();
  driveTo_.cancel();
  turn_.cancel();
  driveActive_ = false;
  moveRampStartMs_ = 0;
  moveRampActive_ = false;
  ui_.showFailed("Cancelled");
  setState_(State::Cancelled);
}

void SequenceExecutorTask::reset() {
  driveBy_.reset();
  driveTo_.reset();
  turn_.reset();
  driveActive_ = false;
  moveRampStartMs_ = 0;
  moveRampActive_ = false;
  moveGuardState_ = MoveGuardState::None;
  moveGuardTurnsDone_ = 0;
  moveGuardClearStreak_ = 0;
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
    // DriveToDistance is started lazily in the update() loop.
    // Ensure no leftover "drive by distance" state leaks across steps.
    driveBy_.cancel();
    driveActive_ = false;
    driveTo_.reset();
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
    if (step.type != SequenceStepType::MoveByDistance) return;

    const float dist = fabsf(moveByCm_);
    const float dir = (moveByCm_ >= 0.0f) ? 1.0f : -1.0f;
    driveBy_.beginByDistance(headingDegContinuous, avgTravelCm, dist, dir * kMoveSpeed);
    driveBy_.setHeadingHoldDeg(headingDegContinuous);
    driveActive_ = true;
    moveRampStartMs_ = millis();
    moveRampActive_ = true;
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
  const SequenceStep& step = steps_[stepIndex_];
  const bool forwardMoveBy =
      (step.type == SequenceStepType::MoveByDistance) && (moveByCm_ > 0.0f);

  // Guard-turn behavior is only for forward MoveByDistance commands.
  if (!forwardMoveBy) return false;

  if (moveGuardState_ == MoveGuardState::None) {
    // Only guard on valid LiDAR. If LiDAR is invalid, do not stall or turn;
    // rely on reflex stop once readings resume.
    if (!isfinite(lidarAvgCm) || lidarAvgCm <= 0.0f) return false;
    if (lidarAvgCm >= kStopDistanceCm) return false;

    // Preserve remaining distance before entering guard turn.
    if (forwardMoveBy && driveActive_) {
      const float remaining = driveBy_.remainingCm();
      if (isfinite(remaining) && remaining > 0.0f) moveByCm_ = remaining;
    }

    driveBy_.cancel();
    driveActive_ = false;
    moveGuardClearStreak_ = 0;
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

  if (lidarAvgCm >= kClearDistanceCm) {
    if (moveGuardClearStreak_ < 255) moveGuardClearStreak_++;
    if (moveGuardClearStreak_ >= kGuardClearStreakNeeded) {
      moveGuardState_ = MoveGuardState::None;
      moveGuardTurnsDone_ = 0;
      moveGuardClearStreak_ = 0;
      startMove_(headingDegContinuous, avgTravelCm);
      return false;
    }
    // Keep checking clear samples without moving.
    return true;
  }

  moveGuardClearStreak_ = 0;
  moveGuardTurnsDone_++;
  if (moveGuardTurnsDone_ >= kGuardMaxTurns) {
    ui_.showFailed("Blocked 360");
    setState_(State::Failed);
    return true;
  }

  moveGuardClearStreak_ = 0;
  turn_.begin(headingDegContinuous, kGuardTurnDeg, kTurnSpeed);
  return true;
}
