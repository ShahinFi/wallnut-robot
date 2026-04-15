#pragma once

#include <Arduino.h>

#include "lidar/lidar.h"

// TurretLidar: thin wrapper around the existing LiDAR device class.
// Exists only for naming consistency with other turret-mounted devices.
class TurretLidar {
public:
  TurretLidar();

  bool begin();

  // Non-blocking update; returns true when a new distance is produced.
  bool update(float& distanceCm);

  // Direct read (library call-through).
  float getDistance();

private:
  Lidar lidar_;
};

