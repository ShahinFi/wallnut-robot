#include "lidar/turret/turret_encoder_cal.h"

#include <EEPROM.h>
#include <math.h>

namespace {
static const uint16_t kMagic = 0x7ECA;  // Turret Encoder CAl
static const int kEepromAddr = 64;      // keep distinct from other modules using EEPROM
static const uint32_t kMinTicksPerRev = 10;      // sanity: must rotate enough to measure
static const uint32_t kMaxTicksPerRev = 200000;  // sanity: avoid accidental multi-turn save

static float wrapDeg360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}
}  // namespace

TurretEncoderCal::TurretEncoderCal()
: hasCalibration_(false),
  ticksPerRev_(0),
  active_(false),
  startTicksAbs_(0),
  zeroTicksAbs_(0) {}

bool TurretEncoderCal::loadFromEeprom() {
  uint16_t magic = 0;
  EEPROM.get(kEepromAddr, magic);
  if (magic != kMagic) {
    hasCalibration_ = false;
    ticksPerRev_ = 0;
    return false;
  }

  uint32_t tpr = 0;
  EEPROM.get(kEepromAddr + (int)sizeof(magic), tpr);
  if (tpr < kMinTicksPerRev || tpr > kMaxTicksPerRev) {
    hasCalibration_ = false;
    ticksPerRev_ = 0;
    return false;
  }

  hasCalibration_ = true;
  ticksPerRev_ = tpr;
  return true;
}

bool TurretEncoderCal::hasCalibration() const { return hasCalibration_; }

void TurretEncoderCal::start(long ticksAbsNow) {
  active_ = true;
  startTicksAbs_ = ticksAbsNow;
}

bool TurretEncoderCal::finish(long ticksAbsNow) {
  if (!active_) return false;
  active_ = false;

  const long dt = ticksAbsNow - startTicksAbs_;
  if (dt <= 0) return false;

  const uint32_t tpr = (uint32_t)dt;
  if (tpr < kMinTicksPerRev || tpr > kMaxTicksPerRev) return false;

  if (!saveToEeprom_(tpr)) return false;

  hasCalibration_ = true;
  ticksPerRev_ = tpr;
  return true;
}

bool TurretEncoderCal::active() const { return active_; }

uint32_t TurretEncoderCal::ticksPerRev() const { return ticksPerRev_; }

float TurretEncoderCal::degPerTick() const {
  if (!hasCalibration_ || ticksPerRev_ == 0) return 0.0f;
  return 360.0f / (float)ticksPerRev_;
}

void TurretEncoderCal::setZeroTicks(long ticksAbsNow) {
  zeroTicksAbs_ = ticksAbsNow;
}

long TurretEncoderCal::zeroTicks() const { return zeroTicksAbs_; }

float TurretEncoderCal::angleDeg(long ticksAbsNow) const {
  if (!hasCalibration_ || ticksPerRev_ == 0) return 0.0f;
  const long dt = ticksAbsNow - zeroTicksAbs_;
  const float deg = (float)dt * degPerTick();
  return wrapDeg360(deg);
}

bool TurretEncoderCal::saveToEeprom_(uint32_t ticksPerRev) {
  EEPROM.put(kEepromAddr, kMagic);
  EEPROM.put(kEepromAddr + (int)sizeof(kMagic), ticksPerRev);
  return true;
}
