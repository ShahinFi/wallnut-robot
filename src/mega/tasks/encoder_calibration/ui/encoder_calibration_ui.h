#pragma once

#include <Arduino.h>

class EncoderCalibrationUI {
public:
  enum class Phase : uint8_t { Idle, CheckStart, Driving, Compute, Done, Failed };

  void begin();
  void showIdle();
  void showRunning(float distanceCm, float cmPerPulse, float pulses, Phase phase);
  void showFailed(const char* msg);
  void showSaved(float cmPerPulse);
};
