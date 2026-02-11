#include "lidar.h"

#include <Wire.h>
#include "LIDARLite_v4LED.h"

static LIDARLite_v4LED myLIDAR;

Lidar::Lidar() : measuring(false) {}

bool Lidar::begin() {
  Wire.begin();
  measuring = false;
  return myLIDAR.begin();
}

float Lidar::getDistance() {
  return myLIDAR.getDistance();
}

bool Lidar::update(float &distanceCm) {
  if (!measuring) {
    myLIDAR.takeRange();
    measuring = true;
    return false;
  }

  if (myLIDAR.getBusyFlag()) {
    return false;
  }

  distanceCm = static_cast<float>(myLIDAR.readDistance());
  measuring = false;
  return true;
}
