#pragma once

#include "runtime.h"

void cancelActiveMotion(RuntimeState& rt);
void tickActiveControllers(RuntimeState& rt, const CompassData& heading, float lidarFilteredCm);
void emitCommandCompletionEdge(RuntimeState& rt);

