#include "telemetry/telemetry.h"

static uint32_t gIntervalMs = 1000;
static uint32_t gLastSendMs = 0;
static uint32_t gRgbIntervalMs = 1000;
static uint32_t gRgbLastSendMs = 0;

void telemetryInit(uint32_t intervalMs) {
  gIntervalMs = intervalMs;
  gLastSendMs = 0;
  gRgbIntervalMs = intervalMs;
  gRgbLastSendMs = 0;
}

static bool shouldSendNow() {
  const uint32_t now = millis();
  if (now - gLastSendMs < gIntervalMs) return false;
  gLastSendMs = now;
  return true;
}

void telemetryUpdate(float lidarCm, int headingDeg, const char* headingLabel) {
  if (!shouldSendNow()) return;
  Serial3.print("LIDAR:");
  Serial3.println(lidarCm, 1);
  Serial3.print("COMPASS:");
  Serial3.print(headingDeg);
  Serial3.print(",");
  Serial3.println(headingLabel ? headingLabel : "");
}

static bool shouldSendRgbNow() {
  const uint32_t now = millis();
  if (now - gRgbLastSendMs < gRgbIntervalMs) return false;
  gRgbLastSendMs = now;
  return true;
}

void telemetryRgbUpdate(uint8_t r, uint8_t g, uint8_t b) {
  if (!shouldSendRgbNow()) return;
  Serial3.print("RGB:");
  Serial3.print(r);
  Serial3.print(",");
  Serial3.print(g);
  Serial3.print(",");
  Serial3.println(b);
}
