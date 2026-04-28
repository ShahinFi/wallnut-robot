#pragma once

#include <stdint.h>

// WHY: World-frame odometry integrates signed travel with heading (0=N, 90=E).
// CONTRACT: Coordinates use eastCm (X, +East) and northCm (Y, +North).

struct WorldOdomData {
  float eastCm;
  float northCm;
};

struct WorldOdomState {
  bool  hasPrev;
  float prevAvgCmSigned;
  // WHY: Previous heading can be wrapped or continuous; update uses midpoint integration.
  float prevHeadingDeg;
  WorldOdomData pos;
};

// WHY: Clears position and internal baselines.
void worldOdomReset(WorldOdomState& s);

// WHY: Re-bases integrator state without changing accumulated world position.
// CONTRACT: Call after hard encoder resets to prevent false motion injection.
void worldOdomRebase(WorldOdomState& s, float headingDeg, float avgCmSigned);

// CONTRACT: Continuous integration step; call once per loop with latest heading and signed travel.
void worldOdomUpdate(WorldOdomState& s, float headingDeg, float avgCmSigned);
