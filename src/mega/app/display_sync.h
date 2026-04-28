#pragma once

#include "runtime.h"

void maybeRequestEspIp(RuntimeState& rt, uint32_t nowMs);
void updateMazeLcd(RuntimeState& rt, const CompassData& heading, float lidarFilteredCm);

