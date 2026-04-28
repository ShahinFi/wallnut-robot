#pragma once

#include "runtime.h"

void applyReflexSafety(RuntimeState& rt, float lidarFilteredCm, const CompassData& heading);
void resetReflexLatchesForNewCommand(RuntimeState& rt);

