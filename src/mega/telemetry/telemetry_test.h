#pragma once

#include <Arduino.h>

// Automatic sweep of telemetry send rates.
void telemetryTestStart();
bool telemetryTestActive();
void telemetryTestUpdate(float lidarCm);
