#include "telemetry/telemetry_test.h"

static const uint16_t kRatesHz[] = {1, 2, 5, 10, 20, 30, 40, 50};
static const uint8_t kRateCount = sizeof(kRatesHz) / sizeof(kRatesHz[0]);
static const uint32_t kWindowMs = 10000;

static bool gActive = false;
static uint8_t gRateIdx = 0;
static uint32_t gWindowStartMs = 0;
static uint32_t gLastSendMs = 0;
static uint32_t gSeq = 0;
static uint32_t gLastReportMs = 0;
static uint32_t gSentInWindow = 0;

void telemetryTestStart() {
  gActive = true;
  gRateIdx = 0;
  gWindowStartMs = millis();
  gLastSendMs = 0;
  gSeq = 0;
  gLastReportMs = gWindowStartMs;
  gSentInWindow = 0;
  Serial.println("Telemetry test started");
}

bool telemetryTestActive() { return gActive; }

void telemetryTestUpdate(float lidarCm) {
  if (!gActive) return;

  const uint32_t now = millis();
  const uint16_t hz = kRatesHz[gRateIdx];
  const uint32_t intervalMs = (hz > 0) ? (1000UL / hz) : 1000UL;

  if (now - gLastSendMs >= intervalMs) {
    gLastSendMs = now;
    Serial2.print("LIDAR:");
    Serial2.print(lidarCm, 1);
    Serial2.print(",SEQ:");
    Serial2.print(gSeq++);
    Serial2.print(",T:");
    Serial2.println(now);
    gSentInWindow++;
  }

  if (now - gWindowStartMs >= kWindowMs) {
    const float elapsed = (now - gWindowStartMs) / 1000.0f;
    const float actualHz = (elapsed > 0.0f) ? (gSentInWindow / elapsed) : 0.0f;
    Serial.print("Rate ");
    Serial.print(kRatesHz[gRateIdx]);
    Serial.print(" Hz -> sent ");
    Serial.print(gSentInWindow);
    Serial.print(" packets in ");
    Serial.print(elapsed, 1);
    Serial.print("s (");
    Serial.print(actualHz, 1);
    Serial.println(" Hz)");

    gRateIdx++;
    gWindowStartMs = now;
    gLastSendMs = 0;
    gSentInWindow = 0;
    if (gRateIdx >= kRateCount) {
      gActive = false;
      Serial.println("Telemetry test complete");
    }
  }
}
