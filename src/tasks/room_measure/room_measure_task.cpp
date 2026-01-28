#include "tasks/room_measure/room_measure_task.h"

#include <string.h>

RoomMeasureTask::RoomMeasureTask()
: cfg_{},
  state_(State::Idle),
  sweep_(),
  collect_(),
  ui_(),
  lastUiUpdateMs_(0),
  wallCm_{0, 0, 0, 0},
  estimate_{} {

  // ---------------- Internal algorithm defaults ----------------
  // These are NOT exposed to main. This task owns its tuning.

  RoomSweep360::Config sweepCfg;
  sweepCfg.turnSpeed      = 0.25f;
  sweepCfg.timeoutMs      = 60000;
  sweepCfg.sweepDegTarget = 360.0f;
  sweep_.setConfig(sweepCfg);

  RoomScanCollector::Config collectCfg;
  collectCfg.binSizeDeg = 5;
  collectCfg.minValidCm = 5.0f;
  collectCfg.maxValidCm = 800.0f;
  collect_.setConfig(collectCfg);

  // UI will be started when task begins or resets
  estimate_.valid = false;
}

void RoomMeasureTask::setConfig(const Config& cfg) {
  // Only physical/environment parameters are set from outside.
  cfg_ = cfg;
}

const RoomMeasureTask::Config& RoomMeasureTask::config() const { return cfg_; }

void RoomMeasureTask::begin(float headingDegContinuous) {
  // Task-local UI owns the LCD for this task
  ui_.begin();
  ui_.showRunning(0.0f, 0.0f);

  // Reset task data
  collect_.reset();
  sweep_.begin(headingDegContinuous);

  lastUiUpdateMs_ = millis();

  memset(wallCm_, 0, sizeof(wallCm_));
  estimate_ = RoomRectEstimateOutput{};
  estimate_.valid = false;

  setState_(State::Running);
}

bool RoomMeasureTask::update(float headingDegContinuous, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ != State::Running) return true;

  // Progress UI (0.5s cadence)
  const uint32_t now = millis();
  if (now - lastUiUpdateMs_ >= 500UL) {
    lastUiUpdateMs_ = now;
    ui_.showRunning(sweep_.sweepDeg(), lidarAvgCm);
  }

  // Collect data during the sweep (lidarAvgCm is MOVING-AVERAGED)
  collect_.push(sweep_.sweepDeg(), lidarAvgCm);

  // Update sweep (uses your TurnToAngle internally)
  const bool sweepDone = sweep_.update(headingDegContinuous);
  if (!sweepDone) return false;

  // Sweep finished: handle outcome
  if (sweep_.timedOut()) {
    ui_.showFailed("Sweep timeout");
    setState_(State::TimedOut);
    return true;
  }
  if (!sweep_.succeeded()) {
    ui_.showFailed("Sweep cancelled");
    setState_(State::Cancelled);
    return true;
  }

  // Extract 4 minima (one per quadrant)
  if (!collect_.quadrantLocalMinima(wallCm_)) {
    ui_.showFailed("No minima in quad");
    setState_(State::Failed);
    return true;
  }

  // Estimate rectangle dimensions / area / volume
  RoomRectEstimateInput in;
  for (int i = 0; i < 4; ++i) in.wallDistanceCm[i] = wallCm_[i];

  in.lidarToCenterOffsetCm = cfg_.lidarToCenterOffsetCm;
  in.ceilingHeightCm       = cfg_.ceilingHeightCm;

  estimate_ = RoomRectEstimator::compute(in);
  if (!estimate_.valid) {
    ui_.showFailed("Estimation failed");
    setState_(State::Failed);
    return true;
  }

  // Show results on LCD
  float wallAdjCm[4];
  const float off = cfg_.lidarToCenterOffsetCm;
  for (int i = 0; i < 4; ++i) wallAdjCm[i] = wallCm_[i] + off;

  ui_.showSucceeded(wallAdjCm, estimate_.widthCm, estimate_.lengthCm,
                    estimate_.areaM2, estimate_.volumeM3);
  setState_(State::Succeeded);
  return true;
}

void RoomMeasureTask::cancel() {
  sweep_.cancel();
  ui_.showFailed("Cancelled");
  setState_(State::Cancelled);
}

void RoomMeasureTask::reset() {
  sweep_.reset();
  collect_.reset();

  ui_.begin();
  ui_.showIdle();

  lastUiUpdateMs_ = millis();

  memset(wallCm_, 0, sizeof(wallCm_));
  estimate_ = RoomRectEstimateOutput{};
  estimate_.valid = false;

  setState_(State::Idle);
}

bool RoomMeasureTask::active() const { return state_ == State::Running; }
bool RoomMeasureTask::succeeded() const { return state_ == State::Succeeded; }
bool RoomMeasureTask::timedOut() const { return state_ == State::TimedOut; }
RoomMeasureTask::State RoomMeasureTask::state() const { return state_; }

void RoomMeasureTask::setState_(State s) {
  state_ = s;
}
