#pragma once

#include <Arduino.h>

// SECTION: Distance-drive primitive with continuous-heading hold.
// WHY: Supports fixed-distance and continuous modes with a shared steering loop.
class DriveByDistance {
public:
  struct Config {
    float    distanceToleranceCm = 2.0f;
    float    slowDownCm          = 20.0f;
    float    minSpeed            = 0.2f;
    float    maxSpeed            = 0.8f;
    float    kpHeading           = 0.02f;
    float    headingDeadbandDeg  = 1.5f;
    float    headingCorrectionSign = -1.0f;
    float    maxCorrection       = 0.3f;
    uint32_t timeoutMs           = 8000;
    float    motorForwardSign    = 1.0f;
  };

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Cancelled };

  DriveByDistance();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // CONTRACT: `targetTravelCm` must be non-negative; `requestedSpeed` is signed [-1..1].
  void beginByDistance(float headingDegContinuous, float avgTravelCm,
                       float targetTravelCm, float requestedSpeed);

  // CONTRACT: Continuous mode runs until cancelled or timed out.
  void beginContinuous(float headingDegContinuous, float avgTravelCm,
                       float requestedSpeed);

  // CONTRACT: Returns true only when the action is in a terminal state.
  bool update(float headingDegContinuous, float avgTravelCm);

  void cancel();
  void reset();

  // WHY: Optional runtime retuning for higher-level controllers.
  void setRequestedSpeed(float requestedSpeed);
  void setHeadingHoldDeg(float headingDegContinuous);

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;
  State state() const;

  float remainingCm() const;
  float headingErrorDeg() const;

private:
  void stopMotors();
  float computeForwardSpeed(float requestedSpeed, float remainingCmAbs) const;

  Config   cfg_;
  State    state_;

  bool     infiniteDistance_;

  float    targetTravelCm_;
  float    startTravelCm_;
  float    remainingTravelCm_;

  float    requestedSpeed_;
  float    headingHoldDeg_;
  float    headingErrorDeg_;

  uint32_t startMs_;
};
