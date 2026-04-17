#pragma once

#include <Arduino.h>

// TurretAngleTracker: signed relative turret angle tracker driven by:
// - single-channel encoder absolute ticks (monotonic)
// - the last commanded motor direction (via cmd sign)
//
// This is intended for motor-driven motion. If the turret is rotated manually
// while the motor command is near zero, direction cannot be inferred, so the
// tracker ignores that motion for the signed angle accumulator.
class TurretAngleTracker {
public:
  struct Config {
    // Convention for output angle sign:
    // +1 => positive angle when cmd>0 (logical direction)
    // -1 => flip sign (swap positive/negative angles in software)
    int angleSign = 1;

    // If |cmd| <= deadband, treat direction as unknown and ignore tick deltas
    // for the signed accumulator.
    float cmdDeadband = 0.05f;
  };

  TurretAngleTracker();

  void setConfig(const Config& cfg);
  const Config& config() const;

  void setTicksPerRev(uint32_t ticksPerRev);               // sets both directions
  void setTicksPerRevPosNeg(uint32_t ticksPerRevPos, uint32_t ticksPerRevNeg);
  uint32_t ticksPerRevPos() const;
  uint32_t ticksPerRevNeg() const;
  uint32_t ticksPerRevForDirSign(int dirSign) const;

  // Resets internal accumulation and sets current angle to 0.
  void reset();

  // Sets current signed angle as 0 (keeps accumulation history).
  void setZero();

  // Update from latest absolute ticks and logical motor command [-1..1].
  void update(long ticksAbsNow, float motorCmd);

  // Signed relative angle in degrees since last setZero()/reset().
  float angleDegSigned() const;

  // Wrapped [0,360) version of the signed angle.
  float angleDegWrapped360() const;

private:
  static float wrapDeg360_(float deg);

  Config cfg_;

  bool hasPrev_;
  long prevTicksAbs_;

  float angleDegAcc_;      // absolute signed angle accumulator (deg)
  float zeroAngleDegAcc_;  // zero reference (deg)

  uint32_t ticksPerRevPos_;
  uint32_t ticksPerRevNeg_;
};
