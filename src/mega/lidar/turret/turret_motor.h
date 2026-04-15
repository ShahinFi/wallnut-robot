#pragma once

#include <Arduino.h>

// TurretMotor: simple DIR+PWM DC motor with a single-channel encoder (A only).
//
// Pins (Arduino Mega 2560):
// - DIR: 14
// - PWM: 4
// - ENC_A: 18 (interrupt-capable)
//
// Notes:
// - Encoder is single-channel, so it cannot infer direction; ticks are absolute
//   (monotonic) since last hard reset.
// - Higher-level code can combine ticks with commanded direction if it needs a
//   signed estimate.
class TurretMotor {
public:
  TurretMotor();

  void begin();

  // Invert wiring in software:
  // - cmdSign = +1 (default): positive cmd drives "forward" direction
  // - cmdSign = -1: swaps directions without changing higher-level logic
  void setCmdSign(int cmdSign);
  int  cmdSign() const;

  // Normalized command in [-1..1]. Sign selects direction; magnitude selects PWM.
  void setCmd(float cmd);
  void stop();

  // Absolute encoder ticks since last ticksResetHard().
  long ticksAbs() const;

  // Hard reset of tick counter to 0 (rare; intended for calibration/debug).
  void ticksResetHard();

  float lastCmd() const;

private:
  static void isr_();

  int cmdSign_;
  float lastCmd_;
};
