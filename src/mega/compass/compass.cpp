#include "compass.h"
#include <math.h>

Compass::Compass(uint8_t i2cAddress, uint8_t bearingReg)
: wire_(nullptr),
  i2cAddress_(i2cAddress),
  bearingReg_(bearingReg),
  headingOffsetDeg_(0.0f),
  state_{} {
  resetHeadingContinuous();
}

bool Compass::begin(TwoWire& wire) {
  wire_ = &wire;
  wire_->begin();

  resetHeadingContinuous();
  // CONTRACT: Begin succeeds only after current heading is captured as the zero reference.
  return zeroHeadingAtCurrent();
}

bool Compass::read(CompassData& out) {
  if (!readHeadingDegWrapped(out)) return false;
  updateHeadingDegContinuous(out);
  return true;
}

// SECTION: Configuration

void Compass::setHeadingOffsetDeg(float headingOffsetDeg) {
  headingOffsetDeg_ = headingOffsetDeg;
}

float Compass::headingOffsetDeg() const {
  return headingOffsetDeg_;
}


// SECTION: Wrapped read (ONLY wrapped fields)

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

  // CONTRACT: wrapped read must not modify continuous fields.
  return true;
}

// SECTION: Continuous update (ONLY continuous fields)

void Compass::updateHeadingDegContinuous(CompassData& io) {
  // WHY: Requires io.headingDegWrapped already filled.
  if (!state_.hasPrev) {
    state_.hasPrev = true;
    state_.prevHeadingDegWrapped = io.headingDegWrapped;
    state_.headingDegContinuous  = io.headingDegWrapped;
    state_.deltaHeadingDeg       = 0.0f;
    state_.wrapCount             = 0;
  } else {
    const float prev = state_.prevHeadingDegWrapped;
    const float curr = io.headingDegWrapped;

    // WHY: Wrap tracking using thresholds:
    // WHY: - if we crossed from high->low, increment wrap count
    // WHY: - if we crossed from low->high, decrement wrap count
    if (prev > 300.0f && curr < 60.0f) {
      state_.wrapCount++;
    } else if (prev < 60.0f && curr > 300.0f) {
      state_.wrapCount--;
    }

    const float continuous = curr + 360.0f * (float)state_.wrapCount;
    state_.deltaHeadingDeg = continuous - state_.headingDegContinuous;
    state_.headingDegContinuous = continuous;
    state_.prevHeadingDegWrapped = curr;
  }

  // CONTRACT: Export results into the reading struct as the single readout payload.
  io.headingDegContinuous = state_.headingDegContinuous;
  io.deltaHeadingDeg      = state_.deltaHeadingDeg;
}

// SECTION: Zeroing / state

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
  state_.wrapCount             = 0;
}

const CompassContinuousState& Compass::continuousState() const {
  return state_;
}

// SECTION: Hardware read

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

// SECTION: Math helpers

float Compass::wrapDeg360(float headingDeg) {
  while (headingDeg < 0.0f)   headingDeg += 360.0f;
  while (headingDeg >= 360.0f) headingDeg -= 360.0f;
  return headingDeg;
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
