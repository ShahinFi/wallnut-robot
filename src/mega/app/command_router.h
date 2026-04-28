#pragma once

#include "runtime.h"

void sendTelemetrySnapshotToEsp(RuntimeState& rt);
bool handleEspCommand(RuntimeState& rt, const CompassData& heading, float lidarFilteredCm);

