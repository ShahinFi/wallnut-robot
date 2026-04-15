#include "lidar/turret/turret_compass.h"

namespace {
static const uint8_t kTurretCompassAddr = 0x61;
}  // namespace

TurretCompass::TurretCompass() : compass_(kTurretCompassAddr) {}

bool TurretCompass::begin(TwoWire& wire) {
  return compass_.begin(wire);
}

bool TurretCompass::read(CompassData& out) {
  return compass_.read(out);
}

Compass& TurretCompass::raw() { return compass_; }

const Compass& TurretCompass::raw() const { return compass_; }
