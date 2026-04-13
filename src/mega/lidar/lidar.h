#pragma once

#include <Arduino.h>

class Lidar {
public:
  Lidar();
  bool begin();
  float getDistance();
  bool update(float &distanceCm);

private:
  bool measuring;
  uint32_t rangeStartMs;
  static const uint32_t kRangeTimeoutMs = 500;
};
