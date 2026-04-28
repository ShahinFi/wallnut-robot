#include "lidar/turret/turret_encoder_cal.h"

#include <EEPROM.h>
#include <math.h>

namespace {
// SECTION: EEPROM layout and calibration bounds
static const uint16_t kMagicV1 = 0x7ECA;
static const uint16_t kMagicV2 = 0x7ECC;
static const int kEepromAddr = 64;
static const uint32_t kMinTicksPerRev = 10;
static const uint32_t kMaxTicksPerRev = 200000;

static float wrapDeg360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}
}

TurretEncoderCal::TurretEncoderCal()
: hasCalibration_(false),
  ticksPerRevPos_(0),
  ticksPerRevNeg_(0),
  active_(false),
  startTicksAbs_(0),
  zeroTicksAbs_(0) {}

bool TurretEncoderCal::loadFromEeprom() {
  uint16_t magic = 0;
  EEPROM.get(kEepromAddr, magic);

  if (magic == kMagicV1) {
    // WHY: Legacy single-value layout.
    uint32_t tpr = 0;
    EEPROM.get(kEepromAddr + (int)sizeof(magic), tpr);
    if (tpr < kMinTicksPerRev || tpr > kMaxTicksPerRev) {
      hasCalibration_ = false;
      ticksPerRevPos_ = 0;
      ticksPerRevNeg_ = 0;
      return false;
    }

    hasCalibration_ = true;
    ticksPerRevPos_ = tpr;
    ticksPerRevNeg_ = tpr;
    return true;
  }

  if (magic != kMagicV2) {
    hasCalibration_ = false;
    ticksPerRevPos_ = 0;
    ticksPerRevNeg_ = 0;
    return false;
  }

  uint32_t tprPos = 0;
  uint32_t tprNeg = 0;
  int addr = kEepromAddr + (int)sizeof(magic);
  EEPROM.get(addr, tprPos);
  addr += (int)sizeof(tprPos);
  EEPROM.get(addr, tprNeg);

  if (tprPos < kMinTicksPerRev || tprPos > kMaxTicksPerRev ||
      tprNeg < kMinTicksPerRev || tprNeg > kMaxTicksPerRev) {
    hasCalibration_ = false;
    ticksPerRevPos_ = 0;
    ticksPerRevNeg_ = 0;
    return false;
  }

  hasCalibration_ = true;
  ticksPerRevPos_ = tprPos;
  ticksPerRevNeg_ = tprNeg;
  return true;
}

bool TurretEncoderCal::hasCalibration() const { return hasCalibration_; }

bool TurretEncoderCal::setTicksPerRev(uint32_t ticksPerRev) {
  // WHY: Manual override persists one shared value for both directions.
  if (ticksPerRev < kMinTicksPerRev || ticksPerRev > kMaxTicksPerRev) return false;
  active_ = false;
  if (!saveToEeprom_(ticksPerRev, ticksPerRev)) return false;
  hasCalibration_ = true;
  ticksPerRevPos_ = ticksPerRev;
  ticksPerRevNeg_ = ticksPerRev;
  return true;
}

bool TurretEncoderCal::setTicksPerRevPos(uint32_t ticksPerRevPos) {
  if (ticksPerRevPos < kMinTicksPerRev || ticksPerRevPos > kMaxTicksPerRev) return false;
  const uint32_t neg = ticksPerRevNeg_ ? ticksPerRevNeg_ : ticksPerRevPos;
  active_ = false;
  if (!saveToEeprom_(ticksPerRevPos, neg)) return false;
  hasCalibration_ = true;
  ticksPerRevPos_ = ticksPerRevPos;
  ticksPerRevNeg_ = neg;
  return true;
}

bool TurretEncoderCal::setTicksPerRevNeg(uint32_t ticksPerRevNeg) {
  if (ticksPerRevNeg < kMinTicksPerRev || ticksPerRevNeg > kMaxTicksPerRev) return false;
  const uint32_t pos = ticksPerRevPos_ ? ticksPerRevPos_ : ticksPerRevNeg;
  active_ = false;
  if (!saveToEeprom_(pos, ticksPerRevNeg)) return false;
  hasCalibration_ = true;
  ticksPerRevPos_ = pos;
  ticksPerRevNeg_ = ticksPerRevNeg;
  return true;
}

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

  // WHY: Manual encoder-only calibration is direction-agnostic.
  if (!saveToEeprom_(tpr, tpr)) return false;

  hasCalibration_ = true;
  ticksPerRevPos_ = tpr;
  ticksPerRevNeg_ = tpr;
  return true;
}

bool TurretEncoderCal::active() const { return active_; }

uint32_t TurretEncoderCal::ticksPerRevPos() const { return ticksPerRevPos_; }
uint32_t TurretEncoderCal::ticksPerRevNeg() const { return ticksPerRevNeg_; }

uint32_t TurretEncoderCal::ticksPerRevForDirSign(int dirSign) const {
  return (dirSign < 0) ? ticksPerRevNeg_ : ticksPerRevPos_;
}

float TurretEncoderCal::degPerTickPos() const {
  if (!hasCalibration_ || ticksPerRevPos_ == 0) return 0.0f;
  return 360.0f / (float)ticksPerRevPos_;
}

float TurretEncoderCal::degPerTickNeg() const {
  if (!hasCalibration_ || ticksPerRevNeg_ == 0) return 0.0f;
  return 360.0f / (float)ticksPerRevNeg_;
}

void TurretEncoderCal::setZeroTicks(long ticksAbsNow) {
  zeroTicksAbs_ = ticksAbsNow;
}

long TurretEncoderCal::zeroTicks() const { return zeroTicksAbs_; }

float TurretEncoderCal::angleDeg(long ticksAbsNow) const {
  // CONTRACT: Monotonic angle uses absolute tick delta and positive-direction calibration.
  if (!hasCalibration_ || ticksPerRevPos_ == 0) return 0.0f;
  const long dt = ticksAbsNow - zeroTicksAbs_;
  const float deg = (float)dt * degPerTickPos();
  return wrapDeg360(deg);
}

bool TurretEncoderCal::saveToEeprom_(uint32_t ticksPerRevPos, uint32_t ticksPerRevNeg) {
  EEPROM.put(kEepromAddr, kMagicV2);
  int addr = kEepromAddr + (int)sizeof(kMagicV2);
  EEPROM.put(addr, ticksPerRevPos);
  addr += (int)sizeof(ticksPerRevPos);
  EEPROM.put(addr, ticksPerRevNeg);
  return true;
}
