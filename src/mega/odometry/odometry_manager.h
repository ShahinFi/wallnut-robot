#pragma once

#include <Arduino.h>

#include "odometry/odometry.h"
#include "odometry/world_odometry.h"

// SECTION: Odometry manager facade.
// WHY: Provides a consistent API for local signed travel and world-frame integration.
// CONTRACT: Use `odomHardResetKeepWorld` or `odomHardResetAll` for encoder hard resets.

// CONTRACT: Call once after configuring the Odometry instance (pulses per meter).
void odomManagerInit(Odometry* odom);

// CONTRACT: Call once per loop to refresh cached odometry and world integration.
void odomManagerUpdate(float headingDeg);

// WHY: Latest cached wheel odometry reading (updated by odomManagerUpdate).
OdometryData odomRaw();

// SECTION: Local (relative) odometry

// WHY: Re-bases the local "distance since reset" to 0 (does not affect encoders).
void  odomLocalReset();

// WHY: Signed forward/back travel in cm since last odomLocalReset().
float odomLocalReadCmSigned();

// SECTION: World (global) odometry

// WHY: Resets world position to (0,0) and re-bases integration.
void odomWorldReset(float headingDeg);

// WHY: Reads accumulated world position.
WorldOdomData odomWorldRead();

// WHY: Re-bases world integration without changing position. Use after encoder resets.
void odomWorldRebase(float headingDeg);

// WHY: Sets the accumulated world position (East/North, in cm) and re-bases integration.
// WHY: Use this to align world odometry to an external map frame after an initial localization.
void odomWorldSetPos(float eastCm, float northCm, float headingDeg);

// SECTION: Hard resets (rare)

// WHY: Hard-resets encoder counters but keeps world position continuous by rebasing.
// WHY: This is the safe replacement for calling encoderReset() directly.
void odomHardResetKeepWorld(float headingDeg);

// WHY: Hard-resets encoder counters AND resets world position to (0,0).
void odomHardResetAll(float headingDeg);
