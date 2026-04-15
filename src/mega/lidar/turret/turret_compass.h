#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "compass/compass.h"

// TurretCompass: second compass mounted on the LiDAR turret.
//
// I2C:
// - Address: 0x61 (as configured by hardware)
// - Bearing register: 0x01 (same as base Compass default)
//
// The underlying Compass implementation applies headingOffsetDeg before exporting
// headingDegWrapped/Continuous; higher-level code can set/zero that offset as needed.
class TurretCompass {
public:
  TurretCompass();

  bool begin(TwoWire& wire = Wire);
  bool read(CompassData& out);

  // Offset controls (pass-through to Compass)
  void  setHeadingOffsetDeg(float headingOffsetDeg);
  float headingOffsetDeg() const;
  bool  zeroHeadingAtCurrent();
  void  resetHeadingContinuous();

private:
  Compass compass_;
};

