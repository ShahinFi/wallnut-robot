#pragma once
// (Refactor) Transport module: UART ingestion.

#include <Arduino.h>

#include "../core/esp_state.h"
#include "esp_scan_events.h"
#include "esp_alert_ring.h"

namespace esp_uart {

// Non-blocking UART poller: reads Mega->ESP lines and updates cached state.
void poll(EspState& state, esp_scan_events::ScanEvents& scan, esp_alert_ring::AlertRing& alerts);

}  // namespace esp_uart
