#include "odometry/odometry.h"

#include <math.h>
#include <stdlib.h>
#include "encoder/encoder.h"

namespace {
int gLeftSign = 1;
int gRightSign = 1;

int clampSign(int v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}
}  // namespace

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
  if (p >= 0.0f) return (long)(p + 0.5f);
  return (long)(p - 0.5f);
}

OdometryData Odometry::read() const {
  const long l = encoderGetLeft();
  const long r = encoderGetRight();

  OdometryData d;
  const long ls = (long)gLeftSign * l;
  const long rs = (long)gRightSign * r;
  d.leftPulses  = ls;
  d.rightPulses = rs;

  d.leftCm  = pulsesToCm(ls);
  d.rightCm = pulsesToCm(rs);
  d.avgCm   = 0.5f * (d.leftCm + d.rightCm);
  const long avgPulsesAbs = (labs(l) + labs(r)) / 2;
  d.totalAbsCm = pulsesToCm(avgPulsesAbs);
  return d;
}

void odometrySetWheelDirection(int leftSign, int rightSign) {
  const int ls = clampSign(leftSign);
  const int rs = clampSign(rightSign);
  if (ls != 0) gLeftSign = ls;
  if (rs != 0) gRightSign = rs;
}
