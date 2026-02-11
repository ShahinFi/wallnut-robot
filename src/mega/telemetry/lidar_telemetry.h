#pragma once

#include <Arduino.h>

// Sends LIDAR readings to ESP over Serial3.
void lidarTelemetryInit(uint32_t intervalMs = 1000);
void lidarTelemetryUpdate(float lidarCm);
