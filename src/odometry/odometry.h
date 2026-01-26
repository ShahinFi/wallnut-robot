#pragma once

#include <Arduino.h>

struct OdometryData {
  long  leftPulses;
  long  rightPulses;
  float leftCm;
  float rightCm;
  float avgCm;
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
