#pragma once

#include <Arduino.h>

// Drive by encoder/odometry distance while holding heading (continuous/unwrapped degrees).
//
// Two start modes:
// - beginByDistance(): stop when traveled distance meets target
// - beginContinuous(): drive indefinitely (caller decides when to cancel)
class DriveByDistance {
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

  DriveByDistance();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // Drive a fixed distance (cm) while holding the initial heading.
  // - targetTravelCm must be >= 0
  // - requestedSpeed is -1..1 (sign = direction)
  void beginByDistance(float headingDegContinuous, float avgTravelCm,
                       float targetTravelCm, float requestedSpeed);

  // Drive indefinitely while holding the initial heading.
  // - requestedSpeed is -1..1 (sign = direction)
  void beginContinuous(float headingDegContinuous, float avgTravelCm,
                       float requestedSpeed);

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
  float headingErrorDeg() const;   // target - current (continuous, unwrapped)

private:
  void stopMotors();
  float computeForwardSpeed(float requestedSpeed, float remainingCmAbs) const;

  Config   cfg_;
  State    state_;

  bool     infiniteDistance_;  // true when started via beginContinuous()

  float    targetTravelCm_;
  float    startTravelCm_;
  float    remainingTravelCm_;

  float    requestedSpeed_;
  float    headingHoldDeg_;
  float    headingErrorDeg_;

  uint32_t startMs_;
};
