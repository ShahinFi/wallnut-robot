#pragma once

#include <Arduino.h>

class Lidar {
public:
  Lidar();
  bool begin();
  float getDistance();
  bool update(float &distanceCm);

  // WHY: Split-phase measurement API enables deterministic scan timestamping.
  bool startRange();
  // CONTRACT: Returns true only when a completed measurement is available in distanceCm.
  bool pollRange(float& distanceCm);
  // CONTRACT: Cancels any in-flight measurement so scan ownership can be transferred safely.
  void abortRange();

private:
  bool measuring;
  uint32_t rangeStartMs;
  static const uint32_t kRangeTimeoutMs = 500;
};
