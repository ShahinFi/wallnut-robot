#pragma once

#include <Arduino.h>

class SequenceExecutorUI {
public:
  enum class StepLabel : uint8_t { Move, Turn, End };

  void begin();
  void showIdle();
  void showRunning(float distanceCm, float totalCm, uint16_t stepIndex,
                   uint16_t totalSteps, StepLabel label);
  void showFailed(const char* msg);
};
