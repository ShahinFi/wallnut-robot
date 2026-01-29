#pragma once

#include <Arduino.h>

class WallAlign90UI {
public:
  enum class Phase : uint8_t { Idle, Check, Approach1, Turn, Approach2, Done, Failed };

  void begin();
  void showIdle();
  void showRunning(float distanceCm, Phase phase);
  void showFailed(const char* msg);
};
