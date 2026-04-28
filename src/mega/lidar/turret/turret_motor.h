#pragma once

#include <Arduino.h>

// WHY: Controls turret DC motor (DIR+PWM) and tracks absolute encoder ticks.
// CONTRACT: Single-channel encoder reports monotonic ticks and cannot infer direction by itself.
class TurretMotor {
public:
  TurretMotor();

  void begin();

  // WHY: Inverts direction mapping in software to match motor wiring.
  void setCmdSign(int cmdSign);
  int  cmdSign() const;

  // CONTRACT: Command is normalized to [-1,1], where sign selects direction and magnitude selects PWM.
  void setCmd(float cmd);
  void stop();

  // WHY: Returns absolute monotonic ticks since last hard reset.
  long ticksAbs() const;

  // CONTRACT: Hard-resets encoder tick counter to zero.
  void ticksResetHard();

  float lastCmd() const;

private:
  static void isr_();

  int cmdSign_;
  float lastCmd_;
};
