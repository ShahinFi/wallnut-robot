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

void TurretCompass::setHeadingOffsetDeg(float headingOffsetDeg) {
  compass_.setHeadingOffsetDeg(headingOffsetDeg);
}

float TurretCompass::headingOffsetDeg() const {
  return compass_.headingOffsetDeg();
}

bool TurretCompass::zeroHeadingAtCurrent() {
  return compass_.zeroHeadingAtCurrent();
}

void TurretCompass::resetHeadingContinuous() {
  compass_.resetHeadingContinuous();
}
