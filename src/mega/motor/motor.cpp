#include "motor.h"

#include <Arduino.h>
#include <math.h>
#include "odometry/odometry.h"

// --- MOTORS ---
// H-bridge control: direction pins + PWM pins per wheel
static const int kMotorLeftDirPin  = 15;
static const int kMotorRightDirPin = 7;
static const int kMotorLeftPwmPin  = 5;
static const int kMotorRightPwmPin = 6;

// Per-wheel calibration scale factors (defaults: no scaling)
static float gLeftScale  = 0.5f;
static float gRightScale = 0.5f;

static float clampCmd(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}

static void driveWheel(int dirPin, int pwmPin, float cmd) {
  cmd = clampCmd(cmd);

  const int dir = (cmd >= 0.0f) ? HIGH : LOW;
  const int pwm = (int)(fabsf(cmd) * 255.0f + 0.5f);

  digitalWrite(dirPin, dir);
  analogWrite(pwmPin, pwm);
}

void motorInit() {
  pinMode(kMotorLeftDirPin, OUTPUT);
  pinMode(kMotorRightDirPin, OUTPUT);
  pinMode(kMotorLeftPwmPin, OUTPUT);
  pinMode(kMotorRightPwmPin, OUTPUT);

  // Stop motors
  driveWheel(kMotorLeftDirPin,  kMotorLeftPwmPin,  0.0f);
  driveWheel(kMotorRightDirPin, kMotorRightPwmPin, 0.0f);
}

void motorSetScale(float leftScale, float rightScale) {
  // Keep scales sane (avoid reversing here; direction is handled by cmd sign)
  if (!isfinite(leftScale)  || leftScale  < 0.0f) leftScale  = 0.0f;
  if (!isfinite(rightScale) || rightScale < 0.0f) rightScale = 0.0f;

  gLeftScale  = leftScale;
  gRightScale = rightScale;
}

float motorLeftScale()  { return gLeftScale; }
float motorRightScale() { return gRightScale; }

void motorDrive(float leftCmd, float rightCmd) {
  static int lastLeftSign = 1;
  static int lastRightSign = 1;

  if (leftCmd > 0.0f) lastLeftSign = 1;
  else if (leftCmd < 0.0f) lastLeftSign = -1;

  if (rightCmd > 0.0f) lastRightSign = 1;
  else if (rightCmd < 0.0f) lastRightSign = -1;

  odometrySetWheelDirection(lastLeftSign, lastRightSign);

  // Apply wheel compensation scaling, then clamp to [-1..1]
  const float left  = clampCmd(leftCmd  * gLeftScale);
  const float right = clampCmd(rightCmd * gRightScale);

  driveWheel(kMotorLeftDirPin,  kMotorLeftPwmPin,  left);
  driveWheel(kMotorRightDirPin, kMotorRightPwmPin, right);
}
