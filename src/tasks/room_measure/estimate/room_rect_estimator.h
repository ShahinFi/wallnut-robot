#pragma once

#include <Arduino.h>

struct RoomRectEstimateInput {
  float wallDistanceCm[4];       // LiDAR->wall minima per quadrant
  float lidarToCenterOffsetCm;   // LiDAR forward offset from rotation center
  float ceilingHeightCm;         // fixed (100 cm)
};

struct RoomRectEstimateOutput {
  float widthCm;
  float lengthCm;
  float areaM2;
  float volumeM3;
  bool  valid;
};

// RoomRectEstimator: pure geometry.
// Responsibility: compute width/length/area/volume from four wall distances.
class RoomRectEstimator {
public:
  static RoomRectEstimateOutput compute(const RoomRectEstimateInput& in);
};
