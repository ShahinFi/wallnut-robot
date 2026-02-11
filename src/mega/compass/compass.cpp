#include "compass.h"
#include <math.h>

Compass::Compass(uint8_t i2cAddress, uint8_t bearingReg)
: wire_(nullptr),
  i2cAddress_(i2cAddress),
  bearingReg_(bearingReg),
  headingOffsetDeg_(0.0f),
  maxDeltaHeadingDeg_(60.0f),
  state_{} {
  resetHeadingContinuous();
}

bool Compass::begin(TwoWire& wire) {
  wire_ = &wire;
  wire_->begin();

  resetHeadingContinuous();
  return zeroHeadingAtCurrent();   // begin == ready + zeroed
}

bool Compass::read(CompassData& out) {
  if (!readHeadingDegWrapped(out)) return false;
  updateHeadingDegContinuous(out);
  return true;
}

// ---------------- Configuration ----------------

void Compass::setHeadingOffsetDeg(float headingOffsetDeg) {
  headingOffsetDeg_ = headingOffsetDeg;
}

float Compass::headingOffsetDeg() const {
  return headingOffsetDeg_;
}

void Compass::setMaxDeltaHeadingDeg(float maxDeltaHeadingDeg) {
  maxDeltaHeadingDeg_ = maxDeltaHeadingDeg;
}

float Compass::maxDeltaHeadingDeg() const {
  return maxDeltaHeadingDeg_;
}

// ---------------- Wrapped read (ONLY wrapped fields) ----------------

bool Compass::readHeadingDegWrapped(CompassData& out) {
  float headingDegRaw = 0.0f;
  uint8_t bearing8 = 0;

  if (!readHeadingDegRaw(headingDegRaw, bearing8)) return false;

  const float headingDegWrapped = wrapDeg360(headingDegRaw + headingOffsetDeg_);

  out.bearing8 = bearing8;

  out.headingDegWrapped = headingDegWrapped;

  int headingDegRounded = (int)(headingDegWrapped + 0.5f);
  if (headingDegRounded >= 360) headingDegRounded -= 360;
  out.headingDegRounded = headingDegRounded;

  out.headingDirLabel = dirLabelFromDeg(out.headingDegRounded);

  // IMPORTANT: do NOT touch out.headingDegContinuous / out.deltaHeadingDeg here.
  return true;
}

// ---------------- Continuous update (ONLY continuous fields) ----------------

void Compass::updateHeadingDegContinuous(CompassData& io) {
  // Requires io.headingDegWrapped already filled.
  if (!state_.hasPrev) {
    state_.hasPrev = true;
    state_.prevHeadingDegWrapped = io.headingDegWrapped;
    state_.headingDegContinuous  = io.headingDegWrapped;
    state_.deltaHeadingDeg       = 0.0f;
  } else {
    const float deltaHeadingDeg =
        wrapDegDiff180(io.headingDegWrapped, state_.prevHeadingDegWrapped);

    if (fabs(deltaHeadingDeg) > maxDeltaHeadingDeg_) {
      // Glitch: do not integrate, just re-baseline to current wrapped
      state_.deltaHeadingDeg = 0.0f;
      state_.prevHeadingDegWrapped = io.headingDegWrapped;
    } else {
      state_.deltaHeadingDeg = deltaHeadingDeg;
      state_.headingDegContinuous += deltaHeadingDeg;
      state_.prevHeadingDegWrapped = io.headingDegWrapped;
    }
  }

  // Export results into the reading struct (single source of truth for user)
  io.headingDegContinuous = state_.headingDegContinuous;
  io.deltaHeadingDeg      = state_.deltaHeadingDeg;
}

// ---------------- Zeroing / state ----------------

bool Compass::zeroHeadingAtCurrent() {
  CompassData data;
  if (!readHeadingDegWrapped(data)) return false;

  headingOffsetDeg_ = -data.headingDegWrapped;
  resetHeadingContinuous();
  return true;
}

void Compass::resetHeadingContinuous() {
  state_.hasPrev = false;
  state_.prevHeadingDegWrapped = 0.0f;
  state_.headingDegContinuous  = 0.0f;
  state_.deltaHeadingDeg       = 0.0f;
}

const CompassContinuousState& Compass::continuousState() const {
  return state_;
}

// ---------------- Hardware read ----------------

bool Compass::readReg8(uint8_t reg, uint8_t& valOut) {
  if (!wire_) return false;

  wire_->beginTransmission(i2cAddress_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;

  if (wire_->requestFrom((int)i2cAddress_, 1) != 1) return false;
  valOut = wire_->read();
  return true;
}

bool Compass::readHeadingDegRaw(float& headingDegRawOut, uint8_t& bearing8Out) {
  uint8_t bearing8 = 0;
  if (!readReg8(bearingReg_, bearing8)) return false;

  bearing8Out = bearing8;
  headingDegRawOut = (float)bearing8 * (360.0f / 256.0f);
  return true;
}

// ---------------- Math helpers ----------------

float Compass::wrapDeg360(float headingDeg) {
  while (headingDeg < 0.0f)   headingDeg += 360.0f;
  while (headingDeg >= 360.0f) headingDeg -= 360.0f;
  return headingDeg;
}

float Compass::wrapDegDiff180(float aDeg, float bDeg) {
  float d = aDeg - bDeg;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

const char* Compass::dirLabelFromDeg(int headingDeg) {
  headingDeg = (headingDeg % 360 + 360) % 360;
  if (headingDeg >= 23 && headingDeg < 68)  return "NE";
  if (headingDeg >= 68 && headingDeg < 113) return "E";
  if (headingDeg >= 113 && headingDeg < 158) return "SE";
  if (headingDeg >= 158 && headingDeg < 203) return "S";
  if (headingDeg >= 203 && headingDeg < 248) return "SW";
  if (headingDeg >= 248 && headingDeg < 293) return "W";
  if (headingDeg >= 293 && headingDeg < 338) return "NW";
  return "N";
}
