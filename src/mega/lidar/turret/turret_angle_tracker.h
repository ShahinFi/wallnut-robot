#pragma once

#include <Arduino.h>

// WHY: Tracks signed turret angle from monotonic encoder ticks plus commanded motor direction.
// CONTRACT: When command magnitude is in deadband, direction is treated as unknown and signed accumulation is skipped.
class TurretAngleTracker {
public:
  struct Config {
    // WHY: Selects output sign convention relative to positive motor command.
    int angleSign = 1;

    // CONTRACT: If |cmd| <= deadband, tick deltas do not contribute to signed angle.
    float cmdDeadband = 0.05f;
  };

  TurretAngleTracker();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // CONTRACT: Sets both direction calibrations to the same ticks-per-revolution.
  void setTicksPerRev(uint32_t ticksPerRev);
  void setTicksPerRevPosNeg(uint32_t ticksPerRevPos, uint32_t ticksPerRevNeg);
  uint32_t ticksPerRevPos() const;
  uint32_t ticksPerRevNeg() const;
  uint32_t ticksPerRevForDirSign(int dirSign) const;

  // CONTRACT: Clears history and sets current signed angle to zero.
  void reset();

  // WHY: Re-zeros relative angle while preserving accumulated history.
  void setZero();

  // CONTRACT: Update consumes latest absolute ticks and logical motor command in [-1,1].
  void update(long ticksAbsNow, float motorCmd);

  // WHY: Signed relative angle in degrees since last zero reference.
  float angleDegSigned() const;

  // WHY: Wrapped [0,360) representation of the signed relative angle.
  float angleDegWrapped360() const;

private:
  static float wrapDeg360_(float deg);

  Config cfg_;

  bool hasPrev_;
  long prevTicksAbs_;

  // SECTION: Angle Accumulators
  float angleDegAcc_;
  float zeroAngleDegAcc_;

  uint32_t ticksPerRevPos_;
  uint32_t ticksPerRevNeg_;
};
