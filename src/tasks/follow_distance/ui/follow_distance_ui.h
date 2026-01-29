#pragma once

#include <Arduino.h>

// FollowDistanceUI: simple LCD output for follow-distance task.
class FollowDistanceUI {
public:
  enum class FollowStatus : uint8_t { Hold, Forward, Backward, Invalid };

  void begin();
  void showIdle(float targetCm);
  void showRunning(float targetCm, float currentCm, float errorCm,
                   float headingHoldDeg, float headingNowDeg, FollowStatus status);
};
