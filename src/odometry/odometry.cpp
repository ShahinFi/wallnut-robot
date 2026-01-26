#include "odometry/odometry.h"

#include <math.h>
#include "encoder/encoder.h"

Odometry::Odometry(float pulsesPerMeter)
: pulsesPerMeter_(pulsesPerMeter) {}

void Odometry::setPulsesPerMeter(float pulsesPerMeter) {
  pulsesPerMeter_ = pulsesPerMeter;
}

float Odometry::pulsesPerMeter() const {
  return pulsesPerMeter_;
}

float Odometry::pulsesToCm(long pulses) const {
  if (pulsesPerMeter_ <= 0.0f) return 0.0f;
  return (pulses * 100.0f) / pulsesPerMeter_;
}

long Odometry::cmToPulses(float cm) const {
  if (pulsesPerMeter_ <= 0.0f) return 0;
  const float p = (cm / 100.0f) * pulsesPerMeter_;
  return (long)(fabsf(p) + 0.5f);
}

OdometryData Odometry::read() const {
  const long l = encoderGetLeft();
  const long r = encoderGetRight();

  OdometryData d;
  d.leftPulses  = l;
  d.rightPulses = r;

  d.leftCm  = pulsesToCm(l);
  d.rightCm = pulsesToCm(r);
  d.avgCm   = 0.5f * (d.leftCm + d.rightCm);
  return d;
}
