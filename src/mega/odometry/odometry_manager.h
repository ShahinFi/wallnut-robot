#pragma once

#include <Arduino.h>

#include "odometry/odometry.h"
#include "odometry/world_odometry.h"

// Odometry manager: provides thin, consistent APIs for:
// - local (relative) signed travel since last local reset
// - global (world-frame) East/North integration
// - rare hard encoder resets with correct rebasing behavior
//
// Coordinate conventions (world frame):
// - eastCm  (X): +East, -West
// - northCm (Y): +North, -South
//
// Heading convention:
// - headingDeg: 0 = North, 90 = East (clockwise)
//
// IMPORTANT:
// - Prefer local baselines (odomLocalReset/odomLocalReadCmSigned) over calling
//   encoderReset() between actions.
// - If you must call encoderReset(), use odomHardResetKeepWorld() or
//   odomHardResetAll() so global integration doesn't see a fake jump.

// Must be called once after you configure the Odometry instance (pulses/meter).
void odomManagerInit(Odometry* odom);

// Must be called continuously (once per loop) to update cached readings and
// world-frame integration.
void odomManagerUpdate(float headingDeg);

// Latest cached wheel odometry reading (updated by odomManagerUpdate).
OdometryData odomRaw();

// -------- Local (relative) odometry --------

// Re-bases the local "distance since reset" to 0 (does not affect encoders).
void  odomLocalReset();

// Signed forward/back travel in cm since last odomLocalReset().
float odomLocalReadCmSigned();

// -------- World (global) odometry --------

// Resets world position to (0,0) and re-bases integration.
void odomWorldReset(float headingDeg);

// Reads accumulated world position.
WorldOdomData odomWorldRead();

// Re-bases world integration without changing position. Use after encoder resets.
void odomWorldRebase(float headingDeg);

// -------- Hard resets (rare) --------

// Hard-resets encoder counters but keeps world position continuous by rebasing.
// This is the safe replacement for calling encoderReset() directly.
void odomHardResetKeepWorld(float headingDeg);

// Hard-resets encoder counters AND resets world position to (0,0).
void odomHardResetAll(float headingDeg);
