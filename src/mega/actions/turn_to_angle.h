#pragma once

#include <Arduino.h>

// Turn in place by a given delta angle using continuous heading (unwrapped degrees).
class TurnToAngle {
public:
  struct Config {
    float    toleranceDeg    = 2.0f;     // stop when |remaining| <= tolerance
    float    slowDownDeg     = 15.0f;    // taper speed for |remaining| < slowDownDeg
    float    minSpeed        = 0.2f;     // minimum speed during taper (prevents stall)
    float    maxSpeed        = 0.6f;     // cap speed
    uint32_t timeoutMs       = 5000;     // safety timeout
    float    motorTurnSign   = 1.0f;     // +1 or -1 to match wiring (flip if turn direction is wrong)
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

  // Start a turn by deltaDeg (positive = CCW, negative = CW) with requested speed (0..1).
  // currentHeadingDegContinuous is the latest continuous heading (unwrapped).
  void begin(float currentHeadingDegContinuous, float deltaDeg, float requestedSpeed);

  // Start a turn by deltaDeg, but always take the shortest equivalent delta in [-180, +180].
  // This preserves the caller's intent for "turn by delta" while avoiding long spins
  // when deltaDeg was expressed as e.g. 270 instead of -90.
  void beginShortestDelta(float currentHeadingDegContinuous, float deltaDeg, float requestedSpeed);

  // Non-blocking tick. Drives motors.
  // Returns true when finished (Succeeded / TimedOut / Cancelled / Idle).
  bool update(float currentHeadingDegContinuous);

  void cancel();     // stop immediately; state becomes Cancelled
  void reset();      // stop and go to Idle

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;

  State state() const;

  // Last computed remaining degrees: positive means CCW remaining, negative means CW remaining.
  float remainingDeg() const;

private:
  void stopMotors();
  float computeSpeedCmd(float requestedSpeed, float remainingAbs) const;

  Config   cfg_;
  State    state_;

  float    targetHeadingDegContinuous_;
  float    remainingDeg_;          // last computed remaining
  float    requestedSpeed_;        // 0..1 (caller input)
  uint32_t startMs_;
};
