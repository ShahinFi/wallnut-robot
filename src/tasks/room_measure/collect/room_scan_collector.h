#pragma once

#include <Arduino.h>

// RoomScanCollector: stores LiDAR samples during a sweep in angular bins.
// Responsibility: data collection only (no motor control, no geometry).
class RoomScanCollector {
public:
  enum class MinPickMode : uint8_t {
    Lowest,
    ClosestToCenter
  };

  struct Config {
    uint16_t binSizeDeg = 5;     // 1..90
    float    minValidCm = 5.0f;  // reject too small
    float    maxValidCm = 800.0f;// reject too big
    MinPickMode pickMode = MinPickMode::ClosestToCenter;
  };

  RoomScanCollector();

  void setConfig(const Config& cfg);
  const Config& config() const;

  void reset();

  // relSweepDeg: relative sweep angle in degrees (0..360+), monotonic.
  // lidarAvgCm: moving-averaged lidar distance (cm).
  void push(float relSweepDeg, float lidarAvgCm);

  // Returns false if any quadrant has no valid data.
  bool quadrantLocalMinima(float outWallCm[4]) const;

private:
  static float wrapDeg360(float deg);
  uint16_t wrapIndex(int idx) const;
  float smoothedAt(uint16_t idx) const;
  bool isLocalMin(uint16_t idx) const;

  Config cfg_;

  static const uint16_t kMaxBins = 360; // supports binSizeDeg >= 1
  uint16_t binCount_;
  float binMinCm_[kMaxBins];
};
