#pragma once

#include <Arduino.h>

class WallSequenceUI {
public:
  enum class Phase : uint8_t {
    Idle,
    Approach30_1,
    TurnLeft_1,
    Approach15_2,
    TurnLeft_2,
    Approach30_3,
    TurnLeft_3,
    Approach15_4,
    Done,
    Failed
  };

  void begin();
  void showIdle();
  void showRunning(float distanceCm, float totalCm, Phase phase);
  void showFailed(const char* msg);
};
