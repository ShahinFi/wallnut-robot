
## src/actions/drive_straight.cpp

```
#include "actions/drive_straight.h"

#include <math.h>
#include "motor/motor.h"

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

DriveStraight::DriveStraight()
: cfg_{},
  state_(State::Idle),
  infiniteDistance_(false),   // NEW
  targetTravelCm_(0.0f),
  startTravelCm_(0.0f),
  remainingTravelCm_(0.0f),
  requestedSpeed_(0.0f),
  headingHoldDeg_(0.0f),
  headingErrorDeg_(0.0f),
  startMs_(0) {}

void DriveStraight::setConfig(const Config& cfg) { cfg_ = cfg; }
const DriveStraight::Config& DriveStraight::config() const { return cfg_; }

void DriveStraight::begin(float headingDegContinuous,
                          float avgTravelCm,
                          float targetTravelCm,
                          float requestedSpeed) {
  stopMotors();

  // NEW: allow "no distance" mode when targetTravelCm < 0
  infiniteDistance_ = (targetTravelCm < 0.0f);
  if (infiniteDistance_) targetTravelCm = 0.0f; // keep internal values benign

  requestedSpeed_    = clamp01(requestedSpeed);
  targetTravelCm_    = targetTravelCm;
  startTravelCm_     = avgTravelCm;
  remainingTravelCm_ = targetTravelCm;

  headingHoldDeg_    = headingDegContinuous;
  headingErrorDeg_   = 0.0f;

  startMs_ = millis();

  // NEW: only allow immediate success when we actually have a distance target
  if (!infiniteDistance_ && targetTravelCm_ <= cfg_.distanceToleranceCm) {
    state_ = State::Succeeded;
    return;
  }

  state_ = State::Running;
}

bool DriveStraight::update(float headingDegContinuous, float avgTravelCm) {
  if (state_ != State::Running) return true;

  const uint32_t now = millis();
  if (now - startMs_ >= cfg_.timeoutMs) {
    stopMotors();
    state_ = State::TimedOut;
    return true;
  }

  // NEW: distance stop condition only when not infinite
  if (!infiniteDistance_) {
    const float traveledCm = avgTravelCm - startTravelCm_;
    remainingTravelCm_ = targetTravelCm_ - traveledCm;

    if (remainingTravelCm_ <= cfg_.distanceToleranceCm) {
      stopMotors();
      state_ = State::Succeeded;
      return true;
    }
  }

  // Heading hold: error = target - current (shortest signed)
  headingErrorDeg_ = wrapDegDiff180(headingHoldDeg_, headingDegContinuous);

  // Base forward speed with tapering
  const float remainingAbs = fabsf(remainingTravelCm_);
  const float speedCmd = computeForwardSpeed(requestedSpeed_, remainingAbs);

  const float base = speedCmd * cfg_.motorForwardSign;

  // Heading correction (deg -> speed)
  float corr = cfg_.kpHeading * headingErrorDeg_;
  if (corr >  cfg_.maxCorrection) corr =  cfg_.maxCorrection;
  if (corr < -cfg_.maxCorrection) corr = -cfg_.maxCorrection;

  // Differential steering
  const float leftCmd  = base - corr;
  const float rightCmd = base + corr;

  motorDrive(leftCmd, rightCmd);
  return false;
}

void DriveStraight::cancel() {
  stopMotors();
  state_ = State::Cancelled;
}

void DriveStraight::reset() {
  stopMotors();
  state_ = State::Idle;

  infiniteDistance_ = false; // NEW

  targetTravelCm_ = 0.0f;
  startTravelCm_ = 0.0f;
  remainingTravelCm_ = 0.0f;

  requestedSpeed_ = 0.0f;
  headingHoldDeg_ = 0.0f;
  headingErrorDeg_ = 0.0f;

  startMs_ = 0;
}

bool DriveStraight::active() const { return state_ == State::Running; }
bool DriveStraight::succeeded() const { return state_ == State::Succeeded; }
bool DriveStraight::timedOut() const { return state_ == State::TimedOut; }
DriveStraight::State DriveStraight::state() const { return state_; }

float DriveStraight::remainingCm() const { return remainingTravelCm_; }
float DriveStraight::headingErrorDeg() const { return headingErrorDeg_; }

void DriveStraight::stopMotors() {
  motorDrive(0.0f, 0.0f);
}

float DriveStraight::computeForwardSpeed(float requestedSpeed, float remainingCmAbs) const {
  float speed = requestedSpeed;
  if (speed > cfg_.maxSpeed) speed = cfg_.maxSpeed;

  if (cfg_.slowDownCm <= 0.0f) return speed;
  if (remainingCmAbs >= cfg_.slowDownCm) return speed;

  // Taper linearly to minSpeed
  const float t = remainingCmAbs / cfg_.slowDownCm; // 0..1
  float tapered = cfg_.minSpeed + t * (speed - cfg_.minSpeed);

  if (tapered < cfg_.minSpeed) tapered = cfg_.minSpeed;
  if (tapered > speed) tapered = speed;

  return tapered;
}

// Returns shortest signed (target - current) in [-180, +180]
float DriveStraight::wrapDegDiff180(float targetDeg, float currentDeg) {
  float d = targetDeg - currentDeg;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}
```

---
## src/actions/drive_straight.h

```
#pragma once

#include <Arduino.h>

// Drive forward a set distance while holding heading using continuous heading (unwrapped).
// Assumes encoders provide unsigned travel (avgTravelCm increases when moving).
// If targetTravelCm < 0: drive indefinitely until cancel() or timeout.
class DriveStraight {
public:
  struct Config {
    float    distanceToleranceCm = 2.0f;   // stop when remaining <= tolerance
    float    slowDownCm          = 20.0f;  // taper speed for remaining < slowDownCm
    float    minSpeed            = 0.2f;   // minimum during taper (prevents stall)
    float    maxSpeed            = 0.8f;   // cap speed
    float    kpHeading           = 0.02f;  // (deg -> speed) proportional correction
    float    maxCorrection       = 0.3f;   // clamp correction magnitude
    uint32_t timeoutMs           = 8000;   // safety timeout
    float    motorForwardSign    = 1.0f;   // +1 or -1 depending on wiring
  };

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Cancelled };

  DriveStraight();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // Begin forward drive:
  // - targetTravelCm >= 0 : drive that distance
  // - targetTravelCm < 0  : drive indefinitely (no distance stop condition)
  // headingDegContinuous: current continuous heading (unwrapped degrees).
  // avgTravelCm: current unsigned travel reading (cm), typically odometry.avgCm.
  // requestedSpeed: 0..1
  void begin(float headingDegContinuous, float avgTravelCm, float targetTravelCm, float requestedSpeed);

  // Tick with latest sensors; drives motors. Returns true when finished.
  bool update(float headingDegContinuous, float avgTravelCm);

  void cancel();   // stop, state=Cancelled
  void reset();    // stop, state=Idle

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;
  State state() const;

  float remainingCm() const;       // remaining forward travel (cm)
  float headingErrorDeg() const;   // target - current (wrapped to [-180,180])

private:
  void stopMotors();
  float computeForwardSpeed(float requestedSpeed, float remainingCmAbs) const;

  static float wrapDegDiff180(float targetDeg, float currentDeg);

  Config   cfg_;
  State    state_;

  bool     infiniteDistance_;  // NEW: true when targetTravelCm < 0

  float    targetTravelCm_;
  float    startTravelCm_;
  float    remainingTravelCm_;

  float    requestedSpeed_;
  float    headingHoldDeg_;
  float    headingErrorDeg_;

  uint32_t startMs_;
};
```

---
## src/actions/turn_to_angle.cpp

```
#include "actions/turn_to_angle.h"

#include <math.h>
#include "motor/motor.h"

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

TurnToAngle::TurnToAngle()
: cfg_{},
  state_(State::Idle),
  targetHeadingDegContinuous_(0.0f),
  remainingDeg_(0.0f),
  requestedSpeed_(0.0f),
  startMs_(0) {}

void TurnToAngle::setConfig(const Config& cfg) { cfg_ = cfg; }
const TurnToAngle::Config& TurnToAngle::config() const { return cfg_; }

void TurnToAngle::begin(float currentHeadingDegContinuous, float deltaDeg, float requestedSpeed) {
  stopMotors();

  requestedSpeed_ = clamp01(requestedSpeed);
  targetHeadingDegContinuous_ = currentHeadingDegContinuous + deltaDeg;
  remainingDeg_ = deltaDeg;
  startMs_ = millis();

  // If already basically at target, finish immediately
  if (fabsf(deltaDeg) <= cfg_.toleranceDeg) {
    state_ = State::Succeeded;
    return;
  }

  state_ = State::Running;
}

bool TurnToAngle::update(float currentHeadingDegContinuous) {
  // “Finished” means caller doesn't need to keep calling.
  if (state_ != State::Running) return true;

  const uint32_t now = millis();
  if (now - startMs_ >= cfg_.timeoutMs) {
    stopMotors();
    state_ = State::TimedOut;
    return true;
  }

  const float remaining = targetHeadingDegContinuous_ - currentHeadingDegContinuous;
  const float remainingAbs = fabsf(remaining);
  remainingDeg_ = remaining;

  if (remainingAbs <= cfg_.toleranceDeg) {
    stopMotors();
    state_ = State::Succeeded;
    return true;
  }

  const float speedCmd = computeSpeedCmd(requestedSpeed_, remainingAbs);

  // Spin direction comes from sign of remaining
  const float wheelCmd = copysignf(speedCmd, remaining) * cfg_.motorTurnSign;

  // In-place turn
  motorDrive(wheelCmd, -wheelCmd);

  return false;
}

void TurnToAngle::cancel() {
  stopMotors();
  state_ = State::Cancelled;
}

void TurnToAngle::reset() {
  stopMotors();
  state_ = State::Idle;
  remainingDeg_ = 0.0f;
  requestedSpeed_ = 0.0f;
  targetHeadingDegContinuous_ = 0.0f;
  startMs_ = 0;
}

bool TurnToAngle::active() const { return state_ == State::Running; }
bool TurnToAngle::succeeded() const { return state_ == State::Succeeded; }
bool TurnToAngle::timedOut() const { return state_ == State::TimedOut; }
TurnToAngle::State TurnToAngle::state() const { return state_; }
float TurnToAngle::remainingDeg() const { return remainingDeg_; }

void TurnToAngle::stopMotors() {
  motorDrive(0.0f, 0.0f);
}

float TurnToAngle::computeSpeedCmd(float requestedSpeed, float remainingAbs) const {
  // Cap to maxSpeed
  float speed = requestedSpeed;
  if (speed > cfg_.maxSpeed) speed = cfg_.maxSpeed;

  // If no tapering requested, return capped speed
  if (cfg_.slowDownDeg <= 0.0f) return speed;

  // Far from target: full capped speed
  if (remainingAbs >= cfg_.slowDownDeg) return speed;

  // Near target: taper down toward minSpeed (but never exceed speed)
  const float t = remainingAbs / cfg_.slowDownDeg; // 0..1
  float tapered = cfg_.minSpeed + t * (speed - cfg_.minSpeed);

  // Ensure bounds and don't exceed requested/capped speed
  if (tapered < 0.0f) tapered = 0.0f;
  if (tapered < cfg_.minSpeed) tapered = cfg_.minSpeed;
  if (tapered > speed) tapered = speed;

  return tapered;
}

```

---
## src/actions/turn_to_angle.h

```
#pragma once

#include <Arduino.h>

// Turn in place by a given delta angle using continuous heading (unwrapped degrees).
class TurnToAngle {
public:
  struct Config {
    float    toleranceDeg    = 2.0f;     // stop when |remaining| <= tolerance
    float    slowDownDeg     = 15.0f;    // taper speed for |remaining| < slowDownDeg
    float    minSpeed        = 0.2f;     // minimum speed during taper (prevents stall)
    float    maxSpeed        = 0.6f;     // cap speed
    uint32_t timeoutMs       = 5000;     // safety timeout
    float    motorTurnSign   = 1.0f;     // +1 or -1 to match wiring (flip if turn direction is wrong)
  };

  enum class State : uint8_t {
    Idle,
    Running,
    Succeeded,
    TimedOut,
    Cancelled
  };

  TurnToAngle();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // Start a turn by deltaDeg (positive = CCW, negative = CW) with requested speed (0..1).
  // currentHeadingDegContinuous is the latest continuous heading (unwrapped).
  void begin(float currentHeadingDegContinuous, float deltaDeg, float requestedSpeed);

  // Non-blocking tick. Drives motors.
  // Returns true when finished (Succeeded / TimedOut / Cancelled / Idle).
  bool update(float currentHeadingDegContinuous);

  void cancel();     // stop immediately; state becomes Cancelled
  void reset();      // stop and go to Idle

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;

  State state() const;

  // Last computed remaining degrees: positive means CCW remaining, negative means CW remaining.
  float remainingDeg() const;

private:
  void stopMotors();
  float computeSpeedCmd(float requestedSpeed, float remainingAbs) const;

  Config   cfg_;
  State    state_;

  float    targetHeadingDegContinuous_;
  float    remainingDeg_;          // last computed remaining
  float    requestedSpeed_;        // 0..1 (caller input)
  uint32_t startMs_;
};

```

---
## src/compass/compass.cpp

```
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

```

---
## src/compass/compass.h

```
#pragma once

#include <Arduino.h>
#include <Wire.h>

struct CompassData {
  // Sensor register
  uint8_t bearing8;               // 0..255

  // Wrapped heading (UI-friendly)
  float       headingDegWrapped;  // 0..360 after offset + wrap
  int         headingDegRounded;  // rounded 0..359
  const char* headingDirLabel;    // N/NE/E/...

  // Continuous heading (control-friendly)
  float headingDegContinuous;     // unwrapped (can grow +/-)
  float deltaHeadingDeg;          // last step (signed)
};

struct CompassContinuousState {
  bool  hasPrev;
  float prevHeadingDegWrapped;
  float headingDegContinuous;
  float deltaHeadingDeg;
};

class Compass {
public:
  Compass(uint8_t i2cAddress = 0x60, uint8_t bearingReg = 0x01);

  // Initializes I2C + zeros heading at current + resets continuous.
  // Returns false if the compass cannot be read.
  bool begin(TwoWire& wire = Wire);

  // Main operation (single call in loop):
  // - reads wrapped heading into out
  // - updates continuous heading into out
  bool read(CompassData& out);

  // Configuration
  void  setHeadingOffsetDeg(float headingOffsetDeg);
  float headingOffsetDeg() const;

  void  setMaxDeltaHeadingDeg(float maxDeltaHeadingDeg);
  float maxDeltaHeadingDeg() const;

  // Modular building blocks (kept public for testing/advanced use)
  bool readHeadingDegWrapped(CompassData& out);        // fills ONLY wrapped fields
  void updateHeadingDegContinuous(CompassData& io);    // fills ONLY continuous fields
  bool zeroHeadingAtCurrent();                         // sets offset so current becomes ~0°

  // State access
  void resetHeadingContinuous();
  const CompassContinuousState& continuousState() const;

private:
  // Hardware read
  bool readReg8(uint8_t reg, uint8_t& valOut);
  bool readHeadingDegRaw(float& headingDegRawOut, uint8_t& bearing8Out);

  // Math helpers
  static float wrapDeg360(float headingDeg);
  static float wrapDegDiff180(float aDeg, float bDeg);
  static const char* dirLabelFromDeg(int headingDeg);

  // Wiring/config
  TwoWire* wire_;
  uint8_t  i2cAddress_;
  uint8_t  bearingReg_;

  // Parameters
  float headingOffsetDeg_;
  float maxDeltaHeadingDeg_;

  // Continuous tracking state (history)
  CompassContinuousState state_;
};

```

---
## src/display/lcd.cpp

```
#include "lcd.h"

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <string.h>

// ---- LCD PINS / SIZE ----
static const int LCD_RS = 32;
static const int LCD_EN = 33;
static const int LCD_D4 = 34;
static const int LCD_D5 = 35;
static const int LCD_D6 = 36;
static const int LCD_D7 = 37;

static const uint8_t LCD_COLS = 20;
static const uint8_t LCD_ROWS = 4;

static LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void lcdInit() {
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
}

void lcdClear() {
  lcd.clear();
}

void lcdWrite(uint8_t row, uint8_t col, const char *text, bool clearToEOL) {
  if (row >= LCD_ROWS || col >= LCD_COLS || text == nullptr) return;

  lcd.setCursor(col, row);
  lcd.print(text);

  if (clearToEOL) {
    size_t len = strlen(text);
    for (size_t i = len; i < (LCD_COLS - col); ++i) lcd.print(' ');
  }
}

void lcdWriteInt(uint8_t row, uint8_t col, long value, bool clearToEOL) {
  char buf[21];
  snprintf(buf, sizeof(buf), "%ld", value);
  lcdWrite(row, col, buf, clearToEOL);
}

void lcdWriteFloat(uint8_t row, uint8_t col, float value, uint8_t decimals,
                   bool clearToEOL) {
  char buf[21];
  dtostrf(value, 0, decimals, buf);
  lcdWrite(row, col, buf, clearToEOL);
}

```

---
## src/display/lcd.h

```
#pragma once

#include <stdint.h>

void lcdInit();
void lcdClear();
void lcdWrite(uint8_t row, uint8_t col, const char *text, bool clearToEOL = true);
void lcdWriteInt(uint8_t row, uint8_t col, long value, bool clearToEOL = true);
void lcdWriteFloat(uint8_t row, uint8_t col, float value, uint8_t decimals = 2,
                   bool clearToEOL = true);

```

---
## src/encoder/encoder.cpp

```
#include "encoder.h"

static const int ENC_L_A_PIN = 2;
static const int ENC_R_A_PIN = 3;

volatile long encLeft = 0;
volatile long encRight = 0;

static void isrLeft()  { encLeft++; }
static void isrRight() { encRight++; }

void encoderInit() {
  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrRight, RISING);
}

long encoderGetLeft() {
  noInterrupts();
  long v = encLeft;
  interrupts();
  return v;
}

long encoderGetRight() {
  noInterrupts();
  long v = encRight;
  interrupts();
  return v;
}

void encoderReset() {
  noInterrupts();
  encLeft = 0;
  encRight = 0;
  interrupts();
}

```

---
## src/encoder/encoder.h

```
#pragma once
#include <Arduino.h>

void encoderInit();

long encoderGetLeft();
long encoderGetRight();
void encoderReset();

```

---
## src/joystick/joystick.cpp

```
#include "joystick.h"

#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct VelocityRatioMap3 {
  float start_angle;
  float mid_angle;
  float end_angle;
  float start_ratio;
  float mid_ratio;
  float end_ratio;
};

static JoystickConfig gCfg;

static inline float linmap(float x, float x0, float x1, float y0, float y1) {
  if (x1 == x0) return 0.5f * (y0 + y1);
  float t = (x - x0) / (x1 - x0);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return y0 + t * (y1 - y0);
}

static float map3PointAngle(float angle, const VelocityRatioMap3 &m) {
  if (angle <= m.mid_angle)
    return linmap(angle, m.start_angle, m.mid_angle, m.start_ratio, m.mid_ratio);
  return linmap(angle, m.mid_angle, m.end_angle, m.mid_ratio, m.end_ratio);
}

static float joystickAngle(const JoystickData &d) {
  const float x = static_cast<float>(d.rawX) - 512.0f;
  const float y = static_cast<float>(d.rawY) - 512.0f;
  float angle = atan2f(y, x);
  if (angle < 0.0f) angle += 2.0f * static_cast<float>(M_PI);
  return angle;
}

static float joystickMagnitude(const JoystickData &d) {
  const float x = static_cast<float>(d.rawX) - 512.0f;
  const float y = static_cast<float>(d.rawY) - 512.0f;
  float mag = sqrtf(x * x + y * y) / 512.0f;
  if (mag > 1.0f) mag = 1.0f;
  return mag;
}

void joystickInit(const JoystickConfig &cfg) {
  gCfg = cfg;
}

JoystickData joystickRead() {
  JoystickData d = {};
  d.rawX = analogRead(gCfg.pinX);
  d.rawY = analogRead(gCfg.pinY);
  return d;
}

JoystickCommand joystickDrive() {
  JoystickCommand cmd = {0.0f, 0.0f};
  const JoystickData d = joystickRead();
  const float magnitude = joystickMagnitude(d);
  if (magnitude < gCfg.activeThreshold) return cmd;

  const float angle = joystickAngle(d);
  const float midAngle = static_cast<float>(M_PI) / 4.0f;
  VelocityRatioMap3 m = {};

  if (angle >= 0.0f && angle < static_cast<float>(M_PI) / 2.0f) {
    cmd.left = magnitude;
    m = {0.0f, midAngle, static_cast<float>(M_PI) / 2.0f, -1.0f, 0.0f, 1.0f};
    cmd.right = cmd.left * map3PointAngle(angle, m);
  } else if (angle >= static_cast<float>(M_PI) / 2.0f &&
             angle < static_cast<float>(M_PI)) {
    cmd.right = magnitude;
    m = {static_cast<float>(M_PI) / 2.0f,
         static_cast<float>(M_PI) - midAngle,
         static_cast<float>(M_PI), 1.0f, 0.0f, -1.0f};
    cmd.left = cmd.right * map3PointAngle(angle, m);
  } else if (angle >= static_cast<float>(M_PI) &&
             angle < 3.0f * static_cast<float>(M_PI) / 2.0f) {
    cmd.left = -magnitude;
    m = {static_cast<float>(M_PI),
         static_cast<float>(M_PI) + midAngle,
         3.0f * static_cast<float>(M_PI) / 2.0f, -1.0f, 0.0f, 1.0f};
    cmd.right = cmd.left * map3PointAngle(angle, m);
  } else {
    cmd.right = -magnitude;
    m = {3.0f * static_cast<float>(M_PI) / 2.0f,
         2.0f * static_cast<float>(M_PI) - midAngle,
         2.0f * static_cast<float>(M_PI), 1.0f, 0.0f, -1.0f};
    cmd.left = cmd.right * map3PointAngle(angle, m);
  }

  return cmd;
}

```

---
## src/joystick/joystick.h

```
#pragma once

#include <stdint.h>

struct JoystickData {
  int rawX;
  int rawY;
};

struct JoystickConfig {
  uint8_t pinX;
  uint8_t pinY;
  float activeThreshold;
};

struct JoystickCommand {
  float left;
  float right;
};

void joystickInit(const JoystickConfig &cfg);
JoystickData joystickRead();
JoystickCommand joystickDrive();

```

---
## src/lidar/lidar.cpp

```
#include "lidar.h"

#include <Wire.h>
#include "LIDARLite_v4LED.h"

static LIDARLite_v4LED myLIDAR;

Lidar::Lidar() : measuring(false) {}

bool Lidar::begin() {
  Wire.begin();
  measuring = false;
  return myLIDAR.begin();
}

float Lidar::getDistance() {
  return myLIDAR.getDistance();
}

bool Lidar::update(float &distanceCm) {
  if (!measuring) {
    myLIDAR.takeRange();
    measuring = true;
    return false;
  }

  if (myLIDAR.getBusyFlag()) {
    return false;
  }

  distanceCm = static_cast<float>(myLIDAR.readDistance());
  measuring = false;
  return true;
}

```

---
## src/lidar/lidar.h

```
#pragma once

class Lidar {
public:
  Lidar();
  bool begin();
  float getDistance();
  bool update(float &distanceCm);

private:
  bool measuring;
};

```

---
## src/lidar/utils/moving_average.cpp

```
#include "moving_average.h"

MovingAverage::MovingAverage() {
  for (int i = 0; i < N; ++i) buf[i] = 0.0f;
}

void MovingAverage::push(float x) {
  for (int i = N - 1; i > 0; --i) {
    buf[i] = buf[i - 1];
  }
  buf[0] = x;
}

float MovingAverage::average() const {
  float sum = 0.0f;
  for (int i = 0; i < N; ++i) sum += buf[i];
  return sum / N;
}

```

---
## src/lidar/utils/moving_average.h

```
#pragma once

class MovingAverage {
public:
  static const int N = 10;

  MovingAverage();
  void push(float x);
  float average() const;

private:
  float buf[N];
};

```

---
## src/main.cpp

```
#include <Arduino.h>

#include "compass/compass.h"
#include "lidar/lidar.h"
#include "lidar/utils/moving_average.h"
#include "motor/motor.h"
#include "joystick/joystick.h"
#include "ui/display_ui.h"

static Compass compass;
static Lidar lidar;
static MovingAverage filter;
static DisplayUI ui;

void setup() {
  Serial.begin(115200);

  if (!compass.begin()) {
    Serial.println("Compass not responding! Freezing.");
    while (1) {}
  }

  if (!lidar.begin()) {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1) {}
  }

  motorInit();
  ui.begin();
  ui.setFieldHz(DisplayField::Average, 1);
}

void loop() {
  static DisplayData data = {};
  float distanceCm = 0.0f;
  if (lidar.update(distanceCm)) {
    filter.push(distanceCm);
    float avg = filter.average();

    Serial.print("New distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");

    Serial.print("Moving average: ");
    Serial.println(avg);
    Serial.println(" cm");

    data.averageCm = avg;
  }

  ui.update(data);
}

```

---
## src/motor/motor.cpp

```
#include "motor.h"

#include <Arduino.h>
#include <math.h>

// --- MOTORS ---
// H-bridge control: direction pins + PWM pins per wheel
static const int Motor_L_dir_pin = 7;
static const int Motor_R_dir_pin = 8;
static const int Motor_L_pwm_pin = 9;
static const int Motor_R_pwm_pin = 10;

static void driveWheel(int dirPin, int pwmPin, float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);
  int dir = (cmd >= 0.0f) ? HIGH : LOW;
  int pwm = static_cast<int>(fabs(cmd) * 255.0f + 0.5f);
  digitalWrite(dirPin, dir);
  analogWrite(pwmPin, pwm);
}

void motorInit() {
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);

  driveWheel(Motor_L_dir_pin, Motor_L_pwm_pin, 0.0f);
  driveWheel(Motor_R_dir_pin, Motor_R_pwm_pin, 0.0f);
}

void motorDrive(float leftCmd, float rightCmd) {
  driveWheel(Motor_L_dir_pin, Motor_L_pwm_pin, leftCmd);
  driveWheel(Motor_R_dir_pin, Motor_R_pwm_pin, rightCmd);
}

```

---
## src/motor/motor.h

```
#pragma once

void motorInit();
void motorDrive(float leftCmd, float rightCmd);

```

---
## src/odometry/odometry.cpp

```
#include "odometry/odometry.h"

#include <math.h>
#include "encoder/encoder.h"

Odometry::Odometry(float pulsesPerMeter)
: pulsesPerMeter_(pulsesPerMeter) {}

void Odometry::setPulsesPerMeter(float pulsesPerMeter) {
  pulsesPerMeter_ = pulsesPerMeter;
}

float Odometry::pulsesPerMeter() const {
  return pulsesPerMeter_;
}

float Odometry::pulsesToCm(long pulses) const {
  if (pulsesPerMeter_ <= 0.0f) return 0.0f;
  return (pulses * 100.0f) / pulsesPerMeter_;
}

long Odometry::cmToPulses(float cm) const {
  if (pulsesPerMeter_ <= 0.0f) return 0;
  const float p = (cm / 100.0f) * pulsesPerMeter_;
  return (long)(fabsf(p) + 0.5f);
}

OdometryData Odometry::read() const {
  const long l = encoderGetLeft();
  const long r = encoderGetRight();

  OdometryData d;
  d.leftPulses  = l;
  d.rightPulses = r;

  d.leftCm  = pulsesToCm(l);
  d.rightCm = pulsesToCm(r);
  d.avgCm   = 0.5f * (d.leftCm + d.rightCm);
  return d;
}

```

---
## src/odometry/odometry.h

```
#pragma once

#include <Arduino.h>

struct OdometryData {
  long  leftPulses;
  long  rightPulses;
  float leftCm;
  float rightCm;
  float avgCm;
};

class Odometry {
public:
  Odometry(float pulsesPerMeter);

  void  setPulsesPerMeter(float pulsesPerMeter);
  float pulsesPerMeter() const;

  // Reads current encoder counts and returns distances in cm.
  OdometryData read() const;

  // Convenience conversions (stateless)
  float pulsesToCm(long pulses) const;
  long  cmToPulses(float cm) const;

private:
  float pulsesPerMeter_;
};

```

---
## src/ui/display_ui.cpp

```
#include "display_ui.h"

#include <Arduino.h>

#include "display/lcd.h"

DisplayUI::DisplayUI() {
  timers[static_cast<uint8_t>(DisplayField::Average)] = {0, 500};
}

void DisplayUI::begin() {
  lcdInit();
  lcdWrite(0, 0, "Avg (cm):");
}

void DisplayUI::setFieldHz(DisplayField field, uint8_t hz) {
  if (hz == 0) return;
  const uint8_t idx = static_cast<uint8_t>(field);
  timers[idx].intervalMs = 1000UL / hz;
}

void DisplayUI::update(const DisplayData &data) {
  const uint32_t now = millis();

  FieldTimer &avgTimer = timers[static_cast<uint8_t>(DisplayField::Average)];
  if (now - avgTimer.lastMs >= avgTimer.intervalMs) {
    avgTimer.lastMs = now;
    const long avgRounded = lroundf(data.averageCm);
    lcdWriteInt(1, 0, avgRounded);
  }
}

```

---
## src/ui/display_ui.h

```
#pragma once

#include <stdint.h>

struct DisplayData {
  float averageCm;
};

enum class DisplayField : uint8_t {
  Average = 0,
  FieldCount
};

class DisplayUI {
public:
  DisplayUI();
  void begin();
  void setFieldHz(DisplayField field, uint8_t hz);
  void update(const DisplayData &data);

private:
  struct FieldTimer {
    uint32_t lastMs;
    uint32_t intervalMs;
  };

  FieldTimer timers[static_cast<uint8_t>(DisplayField::FieldCount)];
};

```
