#pragma once

#include <Arduino.h>

// WHY: Encoder-only turret calibration stores ticks-per-revolution in EEPROM.
// CONTRACT: Manual one-turn calibration is direction-agnostic because encoder ticks are monotonic.
class TurretEncoderCal {
public:
  TurretEncoderCal();

  // WHY: Persistence
  bool loadFromEeprom();
  bool hasCalibration() const;
  // CONTRACT: setTicksPerRev writes the same calibration value for both directions.
  bool setTicksPerRev(uint32_t ticksPerRev);
  bool setTicksPerRevPos(uint32_t ticksPerRevPos);
  bool setTicksPerRevNeg(uint32_t ticksPerRevNeg);

  // SECTION: Manual Calibration Session
  void start(long ticksAbsNow);
  // CONTRACT: Returns true only when a valid calibration is persisted.
  bool finish(long ticksAbsNow);
  bool active() const;

  // SECTION: Calibration Values
  uint32_t ticksPerRevPos() const;
  uint32_t ticksPerRevNeg() const;
  // CONTRACT: dirSign < 0 selects negative-direction calibration; otherwise positive.
  uint32_t ticksPerRevForDirSign(int dirSign) const;
  float    degPerTickPos() const;
  float    degPerTickNeg() const;

  // SECTION: Zeroing and Angle Readout
  void  setZeroTicks(long ticksAbsNow);
  long  zeroTicks() const;
  float angleDeg(long ticksAbsNow) const;

private:
  bool saveToEeprom_(uint32_t ticksPerRevPos, uint32_t ticksPerRevNeg);

  bool     hasCalibration_;
  uint32_t ticksPerRevPos_;
  uint32_t ticksPerRevNeg_;

  bool     active_;
  long     startTicksAbs_;

  long     zeroTicksAbs_;
};
