#pragma once

#include <Arduino.h>

struct OdometryData {
  // WHY: Signed values depend on wheel direction hints coming from the motor layer
  // WHY: (single-channel encoders cannot infer direction on their own).
  long  leftPulsesSigned;
  long  rightPulsesSigned;
  float leftCmSigned;
  float rightCmSigned;

  // WHY: Signed forward/backward travel (average of left/right).
  float avgCmSigned;

  // WHY: Absolute travel magnitude (average of abs(left), abs(right)), always >= 0.
  // WHY: despite the historic "total" naming, this is an average, not a sum.
  float avgCmAbs;
};

class Odometry {
public:
  Odometry(float pulsesPerMeter);

  void  setPulsesPerMeter(float pulsesPerMeter);
  float pulsesPerMeter() const;

  // WHY: Reads current encoder counts and returns distances in cm.
  OdometryData read() const;

  // WHY: Convenience conversions (stateless)
  float pulsesToCm(long pulses) const;
  long  cmToPulses(float cm) const;

private:
  float pulsesPerMeter_;
};

// WHY: Provide intended wheel directions when using single-channel encoders.
// WHY: Pass -1, 0, or +1 for each wheel (0 keeps last direction in motor layer).
void odometrySetWheelDirection(int leftSign, int rightSign);
