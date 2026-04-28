#include "odometry/odometry.h"

#include <math.h>
#include <stdlib.h>
#include "encoder/encoder.h"

namespace {
int gLeftSign = 1;
int gRightSign = 1;

// WHY: Single-channel encoders only count upward. To provide signed distance we
// WHY: integrate signed deltas using the last commanded wheel direction sign.
bool gHasPrevRaw = false;
long gPrevLeftRaw = 0;
long gPrevRightRaw = 0;
long gLeftSignedAcc = 0;
long gRightSignedAcc = 0;

int clampSign(int v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}
}

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

  // WHY: Detect encoderReset() (raw counts drop) or first use, then re-base.
  if (!gHasPrevRaw || l < gPrevLeftRaw || r < gPrevRightRaw) {
    gHasPrevRaw = true;
    gPrevLeftRaw = l;
    gPrevRightRaw = r;
    gLeftSignedAcc = 0;
    gRightSignedAcc = 0;
  } else {
    const long dl = l - gPrevLeftRaw;
    const long dr = r - gPrevRightRaw;
    gPrevLeftRaw = l;
    gPrevRightRaw = r;

    gLeftSignedAcc += (long)gLeftSign * dl;
    gRightSignedAcc += (long)gRightSign * dr;
  }

  OdometryData d;
  const long ls = gLeftSignedAcc;
  const long rs = gRightSignedAcc;
  d.leftPulsesSigned  = ls;
  d.rightPulsesSigned = rs;

  d.leftCmSigned  = pulsesToCm(ls);
  d.rightCmSigned = pulsesToCm(rs);
  d.avgCmSigned   = 0.5f * (d.leftCmSigned + d.rightCmSigned);
  // WHY: Absolute travel since last reset (raw encoders only count up).
  const long avgPulsesAbs = (labs(l) + labs(r)) / 2;
  d.avgCmAbs = pulsesToCm(avgPulsesAbs);
  return d;
}

void odometrySetWheelDirection(int leftSign, int rightSign) {
  const int ls = clampSign(leftSign);
  const int rs = clampSign(rightSign);
  if (ls != 0) gLeftSign = ls;
  if (rs != 0) gRightSign = rs;
}
