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
