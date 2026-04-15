#include "lidar/turret/turret_lidar.h"

TurretLidar::TurretLidar() : lidar_() {}

bool TurretLidar::begin() {
  return lidar_.begin();
}

bool TurretLidar::update(float& distanceCm) {
  return lidar_.update(distanceCm);
}

float TurretLidar::getDistance() {
  return lidar_.getDistance();
}
