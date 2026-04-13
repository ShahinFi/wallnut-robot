#pragma once

#include <Arduino.h>

#include "tasks/room_measure/sweep/room_sweep_360.h"
#include "tasks/room_measure/collect/room_scan_collector.h"
#include "tasks/room_measure/estimate/room_rect_estimator.h"
#include "tasks/room_measure/ui/room_measure_ui.h"

// RoomMeasureTask: orchestrates sweep + collect + estimate + UI.
// Responsibility: sequencing only (glue code).
class RoomMeasureTask {
public:
  // Only PHYSICAL parameters are configured from outside.
  // All algorithm/tuning defaults are owned internally by this task.
  struct Config {
    float lidarToCenterOffsetCm = 0.0f;   // LiDAR forward offset from rotation center (cm)
    float ceilingHeightCm       = 100.0f; // fixed by assignment (cm)
  };

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Failed, Cancelled };

  RoomMeasureTask();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // Start a full 360 sweep from the given current continuous heading.
  void begin(float headingDegContinuous);

  // Non-blocking tick: feed continuous heading and MOVING-AVERAGED lidar distance (cm).
  // Returns true when finished (Succeeded / TimedOut / Failed / Cancelled / Idle).
  bool update(float headingDegContinuous, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;
  State state() const;

private:
  void setState_(State s);

  Config cfg_;
  State  state_;

  RoomSweep360      sweep_;
  RoomScanCollector collect_;
  RoomMeasureUI     ui_;

  // Result cache for UI
  float wallCm_[4];
  RoomRectEstimateOutput estimate_;
};
