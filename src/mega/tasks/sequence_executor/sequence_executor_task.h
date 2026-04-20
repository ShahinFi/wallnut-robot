#pragma once

#include <Arduino.h>

#include "actions/drive_by_distance.h"
#include "actions/drive_to_distance.h"
#include "actions/turn_to_angle.h"
#include "tasks/sequence_executor/ui/sequence_executor_ui.h"

enum class SequenceStepType : uint8_t { MoveToDistance, MoveByDistance, TurnDeg, End };

struct SequenceStep {
  SequenceStepType type;
  float value;  // distance cm or turn deg depending on type
};

// SequenceExecutorTask: executes a sequence of move/turn commands.
class SequenceExecutorTask {
public:
  enum class State : uint8_t { Idle, Running, Succeeded, Failed, Cancelled };

  SequenceExecutorTask();

  void setSequence(const SequenceStep* steps);
  // Scale applied to forward MOVE commands only (0..1). Latched by color sensor.
  void setForwardSpeedScale(float scale01);
  void setAlignHeading(float headingDegContinuous);
  void clearAlignHeading();
  void begin(float headingDegContinuous, float avgTravelCm);
  bool update(float headingDegContinuous, float avgTravelCm, float avgTravelCmAbs, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  State state() const;
  // True only when executing a backward MoveByDistance (e.g. reflex backoff).
  // This is used by reflex logic to avoid canceling a commanded backoff just
  // because the color sensor is still over the red tile.
  bool reverseMoveActive() const;

private:
  void setState_(State s);
  void startStep_(float headingDegContinuous, float avgTravelCm);
  void startMove_(float headingDegContinuous, float avgTravelCm);
  void startTurn_(float headingDegContinuous);
  bool stepTimedOut_() const;
  static float wrapDegDiff180_(float targetDeg, float currentDeg);
  bool handleMoveGuard_(float headingDegContinuous, float avgTravelCm, float lidarAvgCm);

  const SequenceStep* steps_;
  uint16_t stepIndex_;
  uint16_t totalSteps_;

  State state_;
  DriveByDistance driveBy_;
  DriveToDistance driveTo_;
  TurnToAngle   turn_;
  SequenceExecutorUI ui_;

  bool     driveActive_;
  float    targetCm_;
  float    moveByCm_;
  uint32_t stepStartMs_;

  float drivenCmAbs_;

  bool  alignEnabled_; 
  bool  aligning_;
  float alignHeadingDeg_;

  float forwardSpeedScale_;

  // Move acceleration ramp (applied only to straight MoveByDistance steps).
  // No ramp on decel/stop; turns are unaffected.
  uint32_t moveRampStartMs_;
  bool     moveRampActive_;

  enum class MoveGuardState : uint8_t { None, Turning };
  MoveGuardState moveGuardState_;
  uint8_t moveGuardTurnsDone_;
  uint8_t moveGuardClearStreak_;
};
