#pragma once

#include <Arduino.h>

// Sends telemetry readings to ESP over Serial2.
void telemetryInit(uint32_t intervalMs = 1000);
void telemetryUpdate(float lidarCm, int headingDeg, const char* headingLabel);
void telemetryRgbUpdate(uint8_t r, uint8_t g, uint8_t b);
