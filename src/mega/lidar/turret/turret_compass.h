#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "compass/compass.h"

// WHY: Provides a dedicated compass instance for turret-mounted heading sensing.
// CONTRACT: Uses fixed turret compass I2C address (0x61) and standard bearing register mapping.
class TurretCompass {
public:
  TurretCompass();

  bool begin(TwoWire& wire = Wire);
  bool read(CompassData& out);

  // WHY: Exposes underlying Compass for advanced configuration and zeroing flows.
  Compass& raw();
  const Compass& raw() const;

private:
  Compass compass_;
};
