#pragma once

#include <Arduino.h>

struct OdometryData {
  long  leftPulses;
  long  rightPulses;
  float leftCm;
  float rightCm;
  float avgCm;
  float totalAbsCm;
};

class Odometry {
public:
  Odometry(float pulsesPerMeter);

  void  setPulsesPerMeter(float pulsesPerMeter);
  float pulsesPerMeter() const;

  // Reads current encoder counts and returns distances in cm.
  OdometryData read() const;

  // Convenience conversions (stateless)
  float pulsesToCm(long pulses) const;
  long  cmToPulses(float cm) const;

private:
  float pulsesPerMeter_;
};

// Provide intended wheel directions when using single-channel encoders.
// Pass -1, 0, or +1 for each wheel (0 keeps last direction in motor layer).
void odometrySetWheelDirection(int leftSign, int rightSign);
