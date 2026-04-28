#pragma once

#include <Arduino.h>

#include "lidar/lidar.h"

// WHY: Thin wrapper around base LiDAR device for turret subsystem naming consistency.
class TurretLidar {
public:
  TurretLidar();

  bool begin();

  // WHY: Non-blocking update; returns true when a new distance is produced.
  bool update(float& distanceCm);

  // WHY: Direct blocking distance read-through.
  float getDistance();

private:
  Lidar lidar_;
};

