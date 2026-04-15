#pragma once

#include <Arduino.h>

// TurretEncoderCal: encoder-only calibration for turret relative angle.
//
// Calibration procedure (manual, encoder-only):
// 1) Start: capture current absolute ticks.
// 2) Manually rotate turret exactly 360 degrees in any direction.
// 3) Done: capture ticks again; ticksPerRev = deltaTicks; save to EEPROM.
//
// Notes:
// - This is intentionally direction-agnostic; it yields a monotonic angle that
//   increases with ticks (single-channel encoder counts up regardless of direction).
// - A separate "zero" can be set at any time to define where angleDeg=0.
class TurretEncoderCal {
public:
  TurretEncoderCal();

  // Persistence
  bool loadFromEeprom();
  bool hasCalibration() const;

  // Manual calibration session
  void start(long ticksAbsNow);
  bool finish(long ticksAbsNow);  // returns true if saved successfully
  bool active() const;

  // Current calibration value
  uint32_t ticksPerRev() const;
  float    degPerTick() const;

  // Zeroing + angle (monotonic, wrap360)
  void  setZeroTicks(long ticksAbsNow);
  long  zeroTicks() const;
  float angleDeg(long ticksAbsNow) const;

private:
  bool saveToEeprom_(uint32_t ticksPerRev);

  bool     hasCalibration_;
  uint32_t ticksPerRev_;

  bool     active_;
  long     startTicksAbs_;

  long     zeroTicksAbs_;
};

