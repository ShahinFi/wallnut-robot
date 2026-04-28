#include "tasks/sequence_executor/sequence_executor_task.h"

#include <math.h>

namespace {
const float    kToleranceCm    = 2.0f;
const float    kMinValidCm     = 5.0f;
const float    kMaxValidCm     = 800.0f;
const float    kMoveSpeed      = 1.0f;
const float    kTurnSpeed      = 0.6f;
// WHY: Step timeout budget is five minutes per step.
const uint32_t kStepTimeoutMs  = 300000;
// WHY: Acceleration ramp duration for straight MoveByDistance steps.
const uint32_t kMoveRampMs     = 2000;

// WHY: Obstacle-aware MOVE speed policy (forward MOVE only, TURN unaffected).
// CONTRACT: hard safety stop/reflex is handled in main.cpp (FRONTSTOP cancels the step).
// WHY: The executor should not perform autonomous guard-turns, since that causes
// WHY: uncommanded rotations (e.g. cumulative 270°) and makes mission behavior non-deterministic.
const float kSlowDistanceCm  = 30.0f;
const float kSlowSpeedScale  = 0.35f;
// WHY: Default forward speed scale, overridable by color-latched speed policy.
const float kFastSpeedScale  = 0.50f;

static float computeMoveSpeedScale(float lidarAvgCm, float forwardScale) {
  float fast = forwardScale;
  if (!isfinite(fast)) fast = kFastSpeedScale;
  if (fast < 0.0f) fast = 0.0f;
  if (fast > 1.0f) fast = 1.0f;
  // WHY: LiDAR can be temporarily invalid (startup / sensor glitch / during turret scan
  // WHY: transitions). Invalid readings must never stall an in-flight motion command,
  // WHY: otherwise the browser will wait for CMDOK until timeout.
  // WHY: Safety is still enforced by the dedicated reflex stop logic in main.cpp.
  if (!isfinite(lidarAvgCm) || lidarAvgCm <= 0.0f) return fast;
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
  moveRampActive_(false) {
  DriveByDistance::Config dcfg = driveBy_.config();
  dcfg.slowDownCm = 0.0f;
  // WHY: Allow obstacle-based scaling (30%/70%) instead of clamping to full speed.
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
  totalSteps_ = 0;
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
  driveBy_.reset();
  driveTo_.reset();
  turn_.reset();
  driveActive_ = false;
  moveRampStartMs_ = 0;
  moveRampActive_ = false;
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
    if (stepTimedOut_()) {
      setState_(State::Failed);
      return true;
    }
    const bool done = turn_.update(headingDegContinuous);
    if (!done) return false;
    if (turn_.timedOut() || !turn_.succeeded()) {
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
    setState_(State::Succeeded);
    return true;
  }

  if (stepTimedOut_()) {
    setState_(State::Failed);
    return true;
  }

  const bool inMove = (step.type == SequenceStepType::MoveToDistance ||
                       step.type == SequenceStepType::MoveByDistance);
  if (inMove) drivenCmAbs_ = avgTravelCmAbs;

  if (step.type == SequenceStepType::MoveToDistance) {
    // WHY: Forward motion uses the obstacle-aware scaling; backing up does not.
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
        setState_(State::Failed);
        return true;
      }
      if (!driveTo_.succeeded()) {
        setState_(State::Failed);
        return true;
      }
      ++stepIndex_;
      startStep_(headingDegContinuous, avgTravelCm);
    }
    return false;
  }

  if (step.type == SequenceStepType::MoveByDistance) {
    if (!driveActive_) {
      startMove_(headingDegContinuous, avgTravelCm);
    }
    // WHY: Obstacle-aware scaling is only meaningful for forward motion. For backing
    // WHY: up (e.g. reflex backoff), do not let a close-forward wall stall the move.
    const float scale = (moveByCm_ >= 0.0f) ? computeMoveSpeedScale(lidarAvgCm, forwardSpeedScale_) : kFastSpeedScale;
    const float baseSpeed = (moveByCm_ >= 0.0f) ? kMoveSpeed : -kMoveSpeed;

    // WHY: Accel ramp for straight moves only (no ramp on stop/decel; turns unaffected).
    float ramp = 1.0f;
    if (moveRampActive_) {
      const uint32_t dt = (uint32_t)(millis() - moveRampStartMs_);
      if (dt < kMoveRampMs) ramp = (float)dt / (float)kMoveRampMs;
    }
    driveBy_.setRequestedSpeed(baseSpeed * scale * ramp);
    const bool done = driveBy_.update(headingDegContinuous, avgTravelCm);
    if (done) {
      if (driveBy_.timedOut()) {
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

  if (step.type == SequenceStepType::TurnDeg || step.type == SequenceStepType::TurnDegShortest) {
    const bool done = turn_.update(headingDegContinuous);
    if (!done) return false;
    if (turn_.timedOut() || !turn_.succeeded()) {
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
  setState_(State::Cancelled);
}

void SequenceExecutorTask::reset() {
  driveBy_.reset();
  driveTo_.reset();
  turn_.reset();
  driveActive_ = false;
  moveRampStartMs_ = 0;
  moveRampActive_ = false;
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
    // WHY: DriveToDistance is started lazily in the update() loop.
    // WHY: Ensure no leftover "drive by distance" state leaks across steps.
    driveBy_.cancel();
    driveActive_ = false;
    driveTo_.reset();
  } else if (step.type == SequenceStepType::MoveByDistance) {
    moveByCm_ = step.value;
    startMove_(headingDegContinuous, avgTravelCm);
  } else if (step.type == SequenceStepType::TurnDeg || step.type == SequenceStepType::TurnDegShortest) {
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
  const SequenceStep& step = steps_[stepIndex_];
  if (step.type == SequenceStepType::TurnDegShortest) {
    turn_.beginShortestDelta(headingDegContinuous, step.value, kTurnSpeed);
  } else {
    turn_.begin(headingDegContinuous, step.value, kTurnSpeed);
  }
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
