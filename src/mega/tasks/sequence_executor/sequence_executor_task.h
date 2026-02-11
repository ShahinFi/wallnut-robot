#pragma once

#include <Arduino.h>

#include "actions/drive_straight.h"
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
  void setAlignHeading(float headingDegContinuous);
  void clearAlignHeading();
  void begin(float headingDegContinuous, float avgTravelCm);
  bool update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  State state() const;

private:
  void setState_(State s);
  void startStep_(float headingDegContinuous, float avgTravelCm);
  void startMove_(float headingDegContinuous, float avgTravelCm);
  void startTurn_(float headingDegContinuous);
  bool stepTimedOut_() const;
  static float wrapDegDiff180_(float targetDeg, float currentDeg);

  const SequenceStep* steps_;
  uint16_t stepIndex_;
  uint16_t totalSteps_;

  State state_;
  DriveStraight drive_;
  TurnToAngle   turn_;
  SequenceExecutorUI ui_;

  bool     driveActive_;
  float    targetCm_;
  float    moveByCm_;
  uint32_t stepStartMs_;

  float totalDrivenCm_;
  float lastAvgCm_;

  bool  alignEnabled_; 
  bool  aligning_;
  float alignHeadingDeg_;
};
