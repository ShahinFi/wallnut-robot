#pragma once

#include <stdint.h>

// World-frame odometry integrated from:
// - signed forward travel (cm) from wheel odometry
// - compass heading (deg), where 0=N, 90=E
//
// Coordinates:
// - eastCm  (X): +East, -West
// - northCm (Y): +North, -South

struct WorldOdomData {
  float eastCm;
  float northCm;
};

struct WorldOdomState {
  bool  hasPrev;
  float prevAvgCmSigned;
  float prevHeadingDeg;  // degrees (any range), used for midpoint integration
  WorldOdomData pos;
};

// Clears position and internal baselines.
void worldOdomReset(WorldOdomState& s);

// Re-bases the integrator without changing the accumulated position.
// Use this after a hard encoder reset (encoder counters jump to zero) so the
// next update does not interpret the reset as motion.
void worldOdomRebase(WorldOdomState& s, float headingDeg, float avgCmSigned);

// Continuous integration step; call every loop with the latest readings.
//
// headingDeg: compass heading (deg), where 0=N, 90=E. Can be wrapped or continuous.
// avgCmSigned: signed forward/back travel (cm), typically OdometryData.avgCmSigned.
void worldOdomUpdate(WorldOdomState& s, float headingDeg, float avgCmSigned);
