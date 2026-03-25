#pragma once

#include <Arduino.h>

class ColorMazeUI {
public:
  void begin();
  void showIdle();
  void showRunning(const char* label, int speedPct);
  void showBackoff(float cm);
  void showTurn(float deg);
  void showDone();
  void showFailed(const char* msg);
};
