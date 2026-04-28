#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "core/esp_state.h"
#include "http/esp_static_files.h"
#include "transport/esp_scan_events.h"
#include "transport/esp_alert_ring.h"
#include "transport/esp_uart.h"
#include "http/esp_routes.h"

// SECTION: Optional local Wi-Fi secrets override (gitignored).
#if defined(__has_include)
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#endif

// SECTION: Wi-Fi credentials (compile-time defaults).
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

// CONTRACT: Fallback AP is enabled when STA is unavailable.
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "WallNut"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD ""
#endif

static const char* kWifiSsid = WIFI_SSID;
static const char* kWifiPassword = WIFI_PASSWORD;
static const char* kWifiApSsid = WIFI_AP_SSID;
static const char* kWifiApPassword = WIFI_AP_PASSWORD;

static EspState gState;

ESP8266WebServer server(80);

static esp_scan_events::ScanEvents gScanEvents;
static esp_alert_ring::AlertRing gAlertRing;

void listAllFiles();

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting...");

  if (!LittleFS.begin()) {
    Serial.println("❌ Error while mounting LittleFS");
    return;
  }
  Serial.println("✅ LittleFS mounted successfully");

  listAllFiles();

  // WHY: Keep large transport buffers off static .bss on ESP8266.
  (void)gScanEvents.allocOnce();
  gAlertRing.reset();

    bool started = false;
  if (kWifiSsid && kWifiSsid[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.begin(kWifiSsid, kWifiPassword);
    Serial.print("Connecting to WiFi (STA): ");
    Serial.println(kWifiSsid);

    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      attempt++;
      // WHY: Fall back to AP after ~15 seconds of STA retry attempts.
      if (attempt >= 30) break;
    }

    if (WiFi.status() == WL_CONNECTED) {
      started = true;
      Serial.println("\nWiFi connected (STA).");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      // CONTRACT: ESP reports active IP to Mega for LCD display.
      Serial.print("ESPIP:");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nWiFi STA connect failed. Falling back to AP.");
      WiFi.disconnect();
    }
  }

  if (!started) {
    WiFi.mode(WIFI_AP);
    if (kWifiApPassword && kWifiApPassword[0] != '\0') {
      WiFi.softAP(kWifiApSsid, kWifiApPassword);
    } else {
      WiFi.softAP(kWifiApSsid);
    }
    Serial.println("WiFi started (AP).");
    Serial.print("AP SSID: ");
    Serial.println(kWifiApSsid);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    // CONTRACT: ESP reports active AP IP to Mega for LCD display.
    Serial.print("ESPIP:");
    Serial.println(WiFi.softAPIP());
  }

  esp_routes::registerRoutes(server, gState, gScanEvents, gAlertRing);

  server.begin();
  Serial.println("✅ Web server started!");
}

void loop() {
  server.handleClient();
  esp_uart::poll(gState, gScanEvents, gAlertRing);
}

void listAllFiles() {
  Serial.println("\nListing files in LittleFS:");
  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    Serial.print("  FILE: ");
    Serial.print(dir.fileName());
    File f = dir.openFile("r");
    Serial.print("\tSIZE: ");
    Serial.println(f.size());
    f.close();
  }
  Dir dirEsp = LittleFS.openDir("/esp");
  while (dirEsp.next()) {
    Serial.print("  FILE: /esp/");
    Serial.print(dirEsp.fileName());
    File f = dirEsp.openFile("r");
    Serial.print("\tSIZE: ");
    Serial.println(f.size());
    f.close();
  }
  Serial.println("----- End of list -----");
}


