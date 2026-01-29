#pragma once

#include <Arduino.h>

#include "actions/drive_straight.h"
#include "actions/turn_to_angle.h"
#include "tasks/wall_align_90/ui/wall_align_90_ui.h"

// WallAlign90Task: approach wall to fixed distance, turn right 90°, approach again.
// Responsibility: sequencing only (glue over DriveStraight + TurnToAngle).
class WallAlign90Task {
public:
  enum class State : uint8_t {
    Idle,
    CheckFirstWall,
    ApproachFirstWall,
    TurnRight90,
    ApproachSecondWall,
    Succeeded,
    Failed,
    Cancelled
  };

  WallAlign90Task();

  void begin(float headingDegContinuous, float avgTravelCm);
  bool update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  State state() const;

private:
  void setState_(State s);
  void startDrive_(float headingDegContinuous, float avgTravelCm);

  State state_;

  DriveStraight drive_;
  TurnToAngle   turn_;
  WallAlign90UI ui_;

  bool driveActive_;
};
