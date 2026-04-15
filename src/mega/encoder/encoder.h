#pragma once
#include <Arduino.h>

void encoderInit();

long encoderGetLeft();
long encoderGetRight();
// Hard reset of the raw encoder counters (sets both back to 0).
//
// WARNING:
// - This will break any odometry/integration that assumes encoder counts are
//   continuous (e.g. world-frame East/North integration), unless you also
//   re-base the integrators.
//
// Prefer:
// - For "distance since start of an action": use a software baseline (or
//   `odomLocalReset()` / `odomLocalReadCmSigned()` in `odometry/odometry_manager.h`).
// - For safe hard reset: use `odomHardResetKeepWorld()` / `odomHardResetAll()`.
void encoderReset();
