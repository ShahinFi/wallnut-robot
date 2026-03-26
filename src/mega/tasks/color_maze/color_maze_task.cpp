#include "tasks/color_maze/color_maze_task.h"

#include <math.h>

ColorMazeTask::ColorMazeTask()
: cfg_{},
  state_(State::Idle),
  refs_{},
  refsValid_(false),
  drive_(),
  turn_(),
  ui_(),
  pendingTurn_(TurnDir::None),
  cooldownUntilMs_(0) {}

void ColorMazeTask::setConfig(const Config& cfg) { cfg_ = cfg; }

void ColorMazeTask::setCalibration(const ColorRgb refs[3], bool valid) {
  refsValid_ = valid;
  if (refs && valid) {
    refs_[0] = refs[0];
    refs_[1] = refs[1];
    refs_[2] = refs[2];
  }
}

void ColorMazeTask::begin(float headingDegContinuous, float avgTravelCm) {
  ui_.begin();
  if (!refsValid_) {
    ui_.showFailed("No calib");
    setState_(State::Failed);
    return;
  }

  DriveStraight::Config dcfg = drive_.config();
  dcfg.distanceToleranceCm = 1.0f;
  dcfg.slowDownCm = 0.0f;
  dcfg.minSpeed = cfg_.driveSpeed;
  dcfg.maxSpeed = cfg_.driveSpeed;
  dcfg.timeoutMs = 60000;
  drive_.setConfig(dcfg);

  TurnToAngle::Config tcfg = turn_.config();
  tcfg.timeoutMs = 10000;
  turn_.setConfig(tcfg);

  drive_.reset();
  turn_.reset();
  pendingTurn_ = TurnDir::None;
  cooldownUntilMs_ = 0;

  drive_.begin(headingDegContinuous, avgTravelCm, -1.0f, cfg_.driveSpeed);
  drive_.setHeadingHoldDeg(headingDegContinuous);
  setState_(State::Running);
}

bool ColorMazeTask::update(float headingDegContinuous, float avgTravelCm,
                           const ColorRgb* live, bool liveValid) {
  if (state_ == State::Idle) return true;
  if (state_ == State::Done || state_ == State::Failed) return true;

  const uint32_t now = millis();

  if (state_ == State::Backoff) {
    ui_.showBackoff(cfg_.backoffCm);
    const bool done = drive_.update(headingDegContinuous, avgTravelCm);
    if (!done) return false;
    if (drive_.timedOut()) {
      ui_.showFailed("Backoff timeout");
      setState_(State::Failed);
      return true;
    }
    const float turnDeg = (pendingTurn_ == TurnDir::Right) ? cfg_.turnDeg : -cfg_.turnDeg;
    turn_.begin(headingDegContinuous, turnDeg, cfg_.driveSpeed);
    setState_(State::Turn);
    return false;
  }

  if (state_ == State::Turn) {
    ui_.showTurn((pendingTurn_ == TurnDir::Right) ? cfg_.turnDeg : -cfg_.turnDeg);
    const bool done = turn_.update(headingDegContinuous);
    if (!done) return false;
    if (turn_.timedOut() || !turn_.succeeded()) {
      ui_.showFailed("Turn failed");
      setState_(State::Failed);
      return true;
    }
    pendingTurn_ = TurnDir::None;
    cooldownUntilMs_ = now + cfg_.cooldownMs;
    drive_.begin(headingDegContinuous, avgTravelCm, -1.0f, cfg_.driveSpeed);
    drive_.setHeadingHoldDeg(headingDegContinuous);
    setState_(State::Running);
    return false;
  }

  if (state_ == State::Running) {
    ColorClass c = ColorClass::Unknown;
    if (live && liveValid) c = classify_(*live);

    const char* label = "UNK";
    if (c == ColorClass::Left) label = "LEFT";
    else if (c == ColorClass::Right) label = "RIGHT";
    else if (c == ColorClass::End) label = "END";
    else if (c == ColorClass::Floor) label = "FLOOR";

    ui_.showRunning(label, (int)(cfg_.driveSpeed * 100.0f + 0.5f));

    if (c == ColorClass::End) {
      drive_.cancel();
      ui_.showDone();
      setState_(State::Done);
      return true;
    }

    drive_.setRequestedSpeed(cfg_.driveSpeed);
    drive_.update(headingDegContinuous, avgTravelCm);

    if (now < cooldownUntilMs_) return false;

    if (c == ColorClass::Left) {
      drive_.cancel();
      pendingTurn_ = TurnDir::Right;
      drive_.begin(headingDegContinuous, avgTravelCm, cfg_.backoffCm, -cfg_.driveSpeed);
      drive_.setHeadingHoldDeg(headingDegContinuous);
      setState_(State::Backoff);
      return false;
    }
    if (c == ColorClass::Right) {
      drive_.cancel();
      pendingTurn_ = TurnDir::Left;
      drive_.begin(headingDegContinuous, avgTravelCm, cfg_.backoffCm, -cfg_.driveSpeed);
      drive_.setHeadingHoldDeg(headingDegContinuous);
      setState_(State::Backoff);
      return false;
    }
  }

  return false;
}

void ColorMazeTask::cancel() {
  drive_.cancel();
  turn_.cancel();
  ui_.showFailed("Cancelled");
  setState_(State::Failed);
}

void ColorMazeTask::reset() {
  drive_.reset();
  turn_.reset();
  ui_.begin();
  ui_.showIdle();
  pendingTurn_ = TurnDir::None;
  cooldownUntilMs_ = 0;
  setState_(State::Idle);
}

bool ColorMazeTask::active() const { return state_ == State::Running ||
                                            state_ == State::Backoff ||
                                            state_ == State::Turn; }
ColorMazeTask::State ColorMazeTask::state() const { return state_; }

void ColorMazeTask::setState_(State s) { state_ = s; }

ColorMazeTask::ColorClass ColorMazeTask::classify_(const ColorRgb& rgb) const {
  const float dLeft = distSq_(rgb.r, rgb.g, rgb.b, refs_[0], cfg_.useNormalized);
  const float dRight = distSq_(rgb.r, rgb.g, rgb.b, refs_[1], cfg_.useNormalized);
  const float dEnd = distSq_(rgb.r, rgb.g, rgb.b, refs_[2], cfg_.useNormalized);

  const bool leftInRange = dLeft <= (cfg_.leftThreshold * cfg_.leftThreshold);
  const bool rightInRange = dRight <= (cfg_.rightThreshold * cfg_.rightThreshold);
  const bool endInRange = dEnd <= (cfg_.endThreshold * cfg_.endThreshold);

  float best = INFINITY;
  ColorClass cls = ColorClass::Floor;

  if (leftInRange && dLeft < best) {
    best = dLeft;
    cls = ColorClass::Left;
  }
  if (rightInRange && dRight < best) {
    best = dRight;
    cls = ColorClass::Right;
  }
  if (endInRange && dEnd < best) {
    best = dEnd;
    cls = ColorClass::End;
  }

  return cls;
}

float ColorMazeTask::distSq_(float r, float g, float b, const ColorRgb& ref, bool normalized) {
  float rr = r, gg = g, bb = b;
  float r2 = ref.r, g2 = ref.g, b2 = ref.b;
  if (normalized) {
    const float sum1 = rr + gg + bb;
    const float sum2 = r2 + g2 + b2;
    if (sum1 > 0.0f) { rr /= sum1; gg /= sum1; bb /= sum1; }
    if (sum2 > 0.0f) { r2 /= sum2; g2 /= sum2; b2 /= sum2; }
  }
  const float dr = rr - r2;
  const float dg = gg - g2;
  const float db = bb - b2;
  return dr * dr + dg * dg + db * db;
}
