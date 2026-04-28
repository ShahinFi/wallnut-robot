#pragma once
// SECTION: Mega->ESP UART transport.

#include <Arduino.h>

#include "../core/esp_state.h"
#include "esp_scan_events.h"
#include "esp_alert_ring.h"

namespace esp_uart {

// CONTRACT: Non-blocking UART poller; updates shared ESP state from Mega lines.
void poll(EspState& state, esp_scan_events::ScanEvents& scan, esp_alert_ring::AlertRing& alerts);

}
