#include "telemetry/lidar_telemetry.h"

static uint32_t gIntervalMs = 1000;
static uint32_t gLastSendMs = 0;

void lidarTelemetryInit(uint32_t intervalMs) {
  gIntervalMs = intervalMs;
  gLastSendMs = 0;
}

void lidarTelemetryUpdate(float lidarCm) {
  const uint32_t now = millis();
  if (now - gLastSendMs < gIntervalMs) return;
  gLastSendMs = now;

  Serial3.print("LIDAR:");
  Serial3.println(lidarCm, 1);
}
