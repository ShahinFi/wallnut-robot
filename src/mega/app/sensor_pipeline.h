#pragma once

#include "runtime.h"

bool updateHeading(RuntimeState& rt, CompassData& headingOut);
void updateTurretTracking(RuntimeState& rt);
bool handleTurretSweepEarlyReturn(RuntimeState& rt, uint32_t nowMs);
float updateLidarOdomTelemetry(RuntimeState& rt, const CompassData& heading);
void updateColorAndSpeedLatch(RuntimeState& rt, uint32_t nowMs);

