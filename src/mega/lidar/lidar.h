#pragma once

#include <Arduino.h>

class Lidar {
public:
  Lidar();
  bool begin();
  float getDistance();
  bool update(float &distanceCm);

  // Split-phase API (thin + deterministic), useful for scan timestamping:
  // - startRange(): kicks off a measurement (non-blocking)
  // - pollRange(): returns true when the measurement is ready and fills distanceCm
  bool startRange();
  bool pollRange(float& distanceCm);
  void abortRange();  // drop any in-flight measurement (scan mode safety)

private:
  bool measuring;
  uint32_t rangeStartMs;
  static const uint32_t kRangeTimeoutMs = 500;
};
