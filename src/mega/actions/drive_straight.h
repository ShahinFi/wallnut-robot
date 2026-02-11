#pragma once

#include <Arduino.h>

// Drive forward a set distance while holding heading using continuous heading (unwrapped).
// Assumes encoders provide unsigned travel (avgTravelCm increases when moving).
// If targetTravelCm < 0: drive indefinitely until cancel() or timeout.
class DriveStraight {
public:
  struct Config {
    float    distanceToleranceCm = 2.0f;   // stop when remaining <= tolerance
    float    slowDownCm          = 20.0f;  // taper speed for remaining < slowDownCm
    float    minSpeed            = 0.2f;   // minimum during taper (prevents stall)
    float    maxSpeed            = 0.8f;   // cap speed
    float    kpHeading           = 0.02f;  // (deg -> speed) proportional correction
    float    headingDeadbandDeg  = 1.5f;  // no correction within this error
    float    headingCorrectionSign = -1.0f; // +1 or -1 to match turn direction
    float    maxCorrection       = 0.3f;   // clamp correction magnitude
    uint32_t timeoutMs           = 8000;   // safety timeout
    float    motorForwardSign    = 1.0f;   // +1 or -1 depending on wiring
  };

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Cancelled };

  DriveStraight();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // Begin forward drive:
  // - targetTravelCm >= 0 : drive that distance
  // - targetTravelCm < 0  : drive indefinitely (no distance stop condition)
  // headingDegContinuous: current continuous heading (unwrapped degrees).
  // avgTravelCm: current unsigned travel reading (cm), typically odometry.avgCm.
  // requestedSpeed: -1..1 (sign = direction)
  void begin(float headingDegContinuous, float avgTravelCm, float targetTravelCm, float requestedSpeed);

  // Tick with latest sensors; drives motors. Returns true when finished.
  bool update(float headingDegContinuous, float avgTravelCm);

  void cancel();   // stop, state=Cancelled
  void reset();    // stop, state=Idle

  // Update controls while running (optional)
  void setRequestedSpeed(float requestedSpeed);   // -1..1
  void setHeadingHoldDeg(float headingDegContinuous);

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;
  State state() const;

  float remainingCm() const;       // remaining forward travel (cm)
  float headingErrorDeg() const;   // target - current (wrapped to [-180,180])

private:
  void stopMotors();
  float computeForwardSpeed(float requestedSpeed, float remainingCmAbs) const;

  static float wrapDegDiff180(float targetDeg, float currentDeg);

  Config   cfg_;
  State    state_;

  bool     infiniteDistance_;  // NEW: true when targetTravelCm < 0

  float    targetTravelCm_;
  float    startTravelCm_;
  float    remainingTravelCm_;

  float    requestedSpeed_;
  float    headingHoldDeg_;
  float    headingErrorDeg_;

  uint32_t startMs_;
};
