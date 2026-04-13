#include "lidar.h"

#include <Wire.h>
#include "LIDARLite_v4LED.h"

static LIDARLite_v4LED myLIDAR;

Lidar::Lidar() : measuring(false), rangeStartMs(0) {}

bool Lidar::begin() {
  Wire.begin();
  measuring = false;
  rangeStartMs = 0;
  return myLIDAR.begin();
}

float Lidar::getDistance() {
  return myLIDAR.getDistance();
}

bool Lidar::update(float &distanceCm) {
  if (!measuring) {
    myLIDAR.takeRange();
    measuring = true;
    rangeStartMs = millis();
    return false;
  }

  if (myLIDAR.getBusyFlag()) {
    if (millis() - rangeStartMs > kRangeTimeoutMs) {
      // Timeout: drop this measurement and allow a fresh start next call.
      measuring = false;
    }
    return false;
  }

  distanceCm = static_cast<float>(myLIDAR.readDistance());
  measuring = false;
  return true;
}
