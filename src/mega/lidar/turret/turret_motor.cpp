#include "lidar/turret/turret_motor.h"

#include <math.h>

namespace {
static const int kTurretDirPin = 14;
static const int kTurretPwmPin = 4;
static const int kTurretEncAPin = 18;

volatile long gTurretTicksAbs = 0;

static inline float clampSigned1(float x) {
  if (x < -1.0f) return -1.0f;
  if (x >  1.0f) return  1.0f;
  return x;
}
}  // namespace

TurretMotor::TurretMotor() : lastCmd_(0.0f) {}

void TurretMotor::begin() {
  pinMode(kTurretDirPin, OUTPUT);
  pinMode(kTurretPwmPin, OUTPUT);
  pinMode(kTurretEncAPin, INPUT_PULLUP);

  stop();

  // Encoder: count rising edges on channel A.
  attachInterrupt(digitalPinToInterrupt(kTurretEncAPin), TurretMotor::isr_, RISING);
}

void TurretMotor::setCmd(float cmd) {
  cmd = clampSigned1(cmd);
  lastCmd_ = cmd;

  const int dir = (cmd >= 0.0f) ? HIGH : LOW;
  const int pwm = (int)(fabsf(cmd) * 255.0f + 0.5f);

  digitalWrite(kTurretDirPin, dir);
  analogWrite(kTurretPwmPin, pwm);
}

void TurretMotor::stop() {
  setCmd(0.0f);
}

long TurretMotor::ticksAbs() const {
  noInterrupts();
  const long v = gTurretTicksAbs;
  interrupts();
  return v;
}

void TurretMotor::ticksResetHard() {
  noInterrupts();
  gTurretTicksAbs = 0;
  interrupts();
}

float TurretMotor::lastCmd() const { return lastCmd_; }

void TurretMotor::isr_() {
  gTurretTicksAbs++;
}
