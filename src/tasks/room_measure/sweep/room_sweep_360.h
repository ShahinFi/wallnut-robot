#pragma once

#include <Arduino.h>
#include "actions/turn_to_angle.h"

// RoomSweep360: performs a single 360° in-place sweep using TurnToAngle.
// Responsibility: rotation only, no lidar, no bins, no estimation.
class RoomSweep360 {
public:
  struct Config {
    float    turnSpeed      = 0.3f;   // 0..1 passed into TurnToAngle
    uint32_t timeoutMs      = 15000;   // must be long enough for 360°
    float    sweepDegTarget = 360.0f;  // normally 360
  };

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Cancelled };

  RoomSweep360();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // Start sweep from current heading (continuous).
  void begin(float headingDegContinuous);

  // Tick with latest heading (continuous). Returns true when finished.
  bool update(float headingDegContinuous);

  void cancel();
  void reset();

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;
  State state() const;

  // Sweep progress angle relative to start (monotonic, >= 0).
  float sweepDeg() const;

private:
  Config cfg_;
  State  state_;

  TurnToAngle turn_;
  float startHeadingDegContinuous_;
  float sweepDegMonotonic_;
};
