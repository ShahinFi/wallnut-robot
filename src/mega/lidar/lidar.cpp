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

bool Lidar::startRange() {
  if (measuring) return false;
  myLIDAR.takeRange();
  measuring = true;
  rangeStartMs = millis();
  return true;
}

bool Lidar::pollRange(float& distanceCm) {
  if (!measuring) return false;

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

void Lidar::abortRange() {
  measuring = false;
  rangeStartMs = 0;
}

bool Lidar::update(float &distanceCm) {
  if (!measuring) {
    startRange();
    return false;
  }
  return pollRange(distanceCm);
}
