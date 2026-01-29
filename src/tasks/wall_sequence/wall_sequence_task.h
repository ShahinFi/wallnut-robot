#pragma once

#include <Arduino.h>

#include "actions/drive_straight.h"
#include "actions/turn_to_angle.h"
#include "tasks/wall_sequence/ui/wall_sequence_ui.h"

// WallSequenceTask: approach alternating distances with left turns between steps.
// Responsibility: sequencing only (glue over DriveStraight + TurnToAngle).
class WallSequenceTask {
public:
  enum class State : uint8_t {
    Idle,
    Approach30_1,
    TurnLeft_1,
    Approach15_2,
    TurnLeft_2,
    Approach30_3,
    TurnLeft_3,
    Approach15_4,
    Succeeded,
    Failed,
    Cancelled
  };

  WallSequenceTask();

  void begin(float headingDegContinuous, float avgTravelCm);
  bool update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  State state() const;

private:
  void setState_(State s);
  void startApproach_(float headingDegContinuous, float avgTravelCm, float targetCm);
  bool approachUpdate_(float headingDegContinuous, float avgTravelCm, float lidarAvgCm,
                      float targetCm, State nextAfterTurn);

  State state_;
  DriveStraight drive_;
  TurnToAngle   turn_;
  WallSequenceUI ui_;

  bool  driveActive_;
  float targetCm_;
  float startAvgCm_;
  float totalDrivenCm_;
  float lastAvgCm_;
};
