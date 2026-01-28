#include "tasks/room_measure/sweep/room_sweep_360.h"

#include <math.h>

RoomSweep360::RoomSweep360()
: cfg_{},
  state_(State::Idle),
  turn_(),
  startHeadingDegContinuous_(0.0f),
  sweepDegMonotonic_(0.0f) {}

void RoomSweep360::setConfig(const Config& cfg) {
  cfg_ = cfg;

  // Configure underlying TurnToAngle timeout
  TurnToAngle::Config tcfg = turn_.config();
  tcfg.timeoutMs = cfg_.timeoutMs;
  turn_.setConfig(tcfg);
}

const RoomSweep360::Config& RoomSweep360::config() const { return cfg_; }

void RoomSweep360::begin(float headingDegContinuous) {
  turn_.reset();

  startHeadingDegContinuous_ = headingDegContinuous;
  sweepDegMonotonic_ = 0.0f;

  // Always command a +360 sweep. This assumes your continuous heading increases
  // in the direction produced by your motor wiring / TurnToAngle motorTurnSign.
  // If your heading decreases during the turn, set motorTurnSign in TurnToAngle config.
  turn_.begin(headingDegContinuous, cfg_.sweepDegTarget, cfg_.turnSpeed);

  state_ = State::Running;
}

bool RoomSweep360::update(float headingDegContinuous) {
  if (state_ == State::Idle) return true;
  if (state_ != State::Running) return true;

  // Monotonic progress (protects against minor heading noise/backstep)
  float sweepNow = headingDegContinuous - startHeadingDegContinuous_;
  if (sweepNow < 0.0f) sweepNow = 0.0f;
  if (sweepNow > sweepDegMonotonic_) sweepDegMonotonic_ = sweepNow;

  const bool done = turn_.update(headingDegContinuous);
  if (!done) return false;

  if (turn_.timedOut()) {
    state_ = State::TimedOut;
    return true;
  }
  if (turn_.succeeded()) {
    state_ = State::Succeeded;
    return true;
  }

  state_ = State::Cancelled;
  return true;
}

void RoomSweep360::cancel() {
  turn_.cancel();
  state_ = State::Cancelled;
}

void RoomSweep360::reset() {
  turn_.reset();
  state_ = State::Idle;
  startHeadingDegContinuous_ = 0.0f;
  sweepDegMonotonic_ = 0.0f;
}

bool RoomSweep360::active() const { return state_ == State::Running; }
bool RoomSweep360::succeeded() const { return state_ == State::Succeeded; }
bool RoomSweep360::timedOut() const { return state_ == State::TimedOut; }
RoomSweep360::State RoomSweep360::state() const { return state_; }

float RoomSweep360::sweepDeg() const { return sweepDegMonotonic_; }
