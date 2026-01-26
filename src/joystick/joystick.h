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
