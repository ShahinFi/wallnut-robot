#pragma once

#include <Arduino.h>

// SECTION: In-place turn primitive on continuous heading.
class TurnToAngle {
public:
  struct Config {
    float    toleranceDeg    = 2.0f;
    float    slowDownDeg     = 15.0f;
    float    minSpeed        = 0.2f;
    float    maxSpeed        = 0.6f;
    uint32_t timeoutMs       = 5000;
    float    motorTurnSign   = 1.0f;
  };

  enum class State : uint8_t {
    Idle,
    Running,
    Succeeded,
    TimedOut,
    Cancelled
  };

  TurnToAngle();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // CONTRACT: `requestedSpeed` is normalized [0..1].
  void begin(float currentHeadingDegContinuous, float deltaDeg, float requestedSpeed);

  // WHY: Normalizes delta to shortest equivalent turn in [-180, 180].
  void beginShortestDelta(float currentHeadingDegContinuous, float deltaDeg, float requestedSpeed);

  // CONTRACT: Returns true only when action is terminal or idle.
  bool update(float currentHeadingDegContinuous);

  void cancel();
  void reset();

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;

  State state() const;

  // WHY: Positive remaining means CCW left to turn; negative means CW.
  float remainingDeg() const;

private:
  void stopMotors();
  float computeSpeedCmd(float requestedSpeed, float remainingAbs) const;

  Config   cfg_;
  State    state_;

  float    targetHeadingDegContinuous_;
  float    remainingDeg_;
  float    requestedSpeed_;
  uint32_t startMs_;
};
