#pragma once
// (Refactor) HTTP module: endpoint registration.

#include <ESP8266WebServer.h>

#include "../transport/esp_alert_ring.h"
#include "../transport/esp_scan_events.h"
#include "../core/esp_state.h"

namespace esp_routes {

void registerRoutes(ESP8266WebServer& server,
                    EspState& state,
                    esp_scan_events::ScanEvents& scan,
                    esp_alert_ring::AlertRing& alerts);

}  // namespace esp_routes
