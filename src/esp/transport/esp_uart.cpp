#include "esp_uart.h"
// (Refactor) Transport module implementation.

#include <ESP8266WiFi.h>
#include <math.h>
#include <string.h>

namespace esp_uart {
namespace {

static constexpr size_t kMaxSerialLineLen = 128;
static char gLine[kMaxSerialLineLen + 1];
static uint8_t gLineLen = 0;

static bool parseDecFloatSpan_(const char* s, const char* e, float& out) {
  while (s < e && (*s == ' ' || *s == '\t')) s++;
  if (s >= e) return false;

  bool neg = false;
  if (*s == '-') {
    neg = true;
    s++;
  } else if (*s == '+') {
    s++;
  }

  bool any = false;
  uint32_t ip = 0;
  while (s < e && *s >= '0' && *s <= '9') {
    any = true;
    ip = ip * 10u + (uint32_t)(*s - '0');
    s++;
  }

  uint32_t fp = 0;
  uint32_t scale = 1;
  if (s < e && *s == '.') {
    s++;
    while (s < e && *s >= '0' && *s <= '9') {
      any = true;
      if (scale < 1000000u) {
        fp = fp * 10u + (uint32_t)(*s - '0');
        scale *= 10u;
      }
      s++;
    }
  }

  if (!any) return false;
  float v = (float)ip;
  if (scale > 1u) v += (float)fp / (float)scale;
  if (neg) v = -v;
  out = v;
  return isfinite(out);
}

static void processLine_(EspState& state,
                         esp_scan_events::ScanEvents& scan,
                         esp_alert_ring::AlertRing& alerts,
                         const String& data) {
  state.noteMegaRx();

  // Deterministic IP reporting for Mega LCD:
  // Request:  IPREQ
  // Response: ESPIP:<ip>
  if (data.equalsIgnoreCase("IPREQ")) {
    IPAddress ip;
    if (WiFi.status() == WL_CONNECTED) ip = WiFi.localIP();
    if ((uint32_t)ip != 0) {
      Serial.print("ESPIP:");
      Serial.println(ip);
    } else {
      Serial.print("ESPIP:");
      Serial.println(WiFi.softAPIP());
    }
    return;
  }

  if (data.startsWith("AUTH:")) {
    const String s = data.substring(5);
    if (s.startsWith("OK")) {
      state.authState = EspState::AuthState::Armed;
      state.triesLeft = 3;
      state.authPending = false;
    } else if (s.startsWith("OFF")) {
      state.authState = EspState::AuthState::Disarmed;
      state.authPending = false;
    } else if (s.startsWith("LOCKED")) {
      state.authState = EspState::AuthState::Locked;
      state.authPending = false;
    } else if (s.startsWith("FAIL:")) {
      const int tries = s.substring(5).toInt();
      if (tries >= 0 && tries <= 255) state.triesLeft = static_cast<uint8_t>(tries);
      state.authState = EspState::AuthState::Disarmed;
      state.authPending = false;
    }
    return;
  }

  if (data.startsWith("LIDAR:")) {
    state.lidarPacket = data;
    String payload = state.lidarPacket;
    int comma = payload.indexOf(',');
    if (comma >= 0) payload = payload.substring(0, comma);
    state.lidarData = payload;
  } else if (data.startsWith("COMPASS:")) {
    state.compassData = data.substring(8);
  } else if (data.startsWith("RGB:")) {
    state.rgbPacket = data;
  } else if (data.startsWith("RGBCLS:")) {
    const int v = data.substring(7).toInt();
    if (v >= 0 && v <= 4) state.rgbClassPacket = String("CLASS:") + String(v);
  } else if (data.startsWith("RGBREF:")) {
    state.rgbRefsPacket = String("REFS:") + data.substring(7);
  } else if (data.startsWith("ENC_CAL:")) {
    state.encCalPacket = data;
  } else if (data.startsWith("TURCAL:")) {
    state.turretCalPacket = data;
  } else if (data.startsWith("ODOM:")) {
    state.odomPacket = data;
  }

  if (data.startsWith("TSCAN:")) {
    const char* line = data.c_str();
    const int len = data.length();
    if (len <= 6) return;

    const char* payload = line + 6;
    const char* end = line + len;

    if ((end - payload) >= 6 && strncmp(payload, "BEGIN,", 6) == 0) {
      const char* p = payload + 6;
      const int dir = (memchr(p, '-', (size_t)(end - p)) != nullptr) ? -1 : 1;
      scan.pushBegin(dir);
      return;
    }
    if ((end - payload) >= 4 && strncmp(payload, "DONE", 4) == 0) {
      scan.pushDone();
      return;
    }
    if ((end - payload) >= 6 && strncmp(payload, "CANCEL", 6) == 0) {
      scan.pushCancel();
      return;
    }

    const void* cpos = memchr(payload, ',', (size_t)(end - payload));
    if (!cpos) return;
    const char* comma = (const char*)cpos;
    float a = NAN;
    float d = NAN;
    if (!parseDecFloatSpan_(payload, comma, a)) return;
    if (!parseDecFloatSpan_(comma + 1, end, d)) return;
    scan.pushSample(a, d);
  }

  if (data.startsWith("EVT:")) alerts.pushFromLine(data);
}

}  // namespace

void poll(EspState& state, esp_scan_events::ScanEvents& scan, esp_alert_ring::AlertRing& alerts) {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (gLineLen > 0) {
        gLine[gLineLen] = '\0';
        processLine_(state, scan, alerts, String(gLine));
        gLineLen = 0;
      }
      continue;
    }
    if (gLineLen >= kMaxSerialLineLen) {
      gLineLen = 0;
      continue;
    }
    gLine[gLineLen++] = c;
  }
}

}  // namespace esp_uart
