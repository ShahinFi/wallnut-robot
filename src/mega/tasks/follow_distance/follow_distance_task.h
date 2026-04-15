#pragma once

#include <Arduino.h>

#include "actions/drive_by_distance.h"
#include "tasks/follow_distance/ui/follow_distance_ui.h"

// FollowDistanceTask: maintains a fixed distance to an object/wall.
// Responsibility: glue logic only (uses DriveByDistance for motion).
class FollowDistanceTask {
public:
  enum class State : uint8_t { Idle, Running, TimedOut, Cancelled };

  FollowDistanceTask();

  // Only external parameter: target distance (cm)
  void  setTargetDistanceCm(float targetDistanceCm);
  float targetDistanceCm() const;

  void begin(float headingDegContinuous, float avgTravelCm);
  bool update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  State state() const;

private:
  float targetDistanceCm_;
  State state_;

  DriveByDistance    drive_;
  FollowDistanceUI  ui_;

  float lastErrorCm_;
  float headingHoldDeg_;
};
