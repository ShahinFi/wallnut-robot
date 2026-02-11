#pragma once

#include <Arduino.h>

// RoomMeasureUI: task-local LCD output.
// Responsibility: show state/progress/results on LCD (no logic).
class RoomMeasureUI {
public:
  void begin();
  void showIdle();
  void showRunning(float sweepDeg, float lidarCm);
  void showSucceeded(const float wallCm[4], float widthCm, float lengthCm,
                     float areaM2, float volumeM3);
  void showFailed(const char* msg);
};
