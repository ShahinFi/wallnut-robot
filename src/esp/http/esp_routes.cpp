#include "esp_routes.h"
// SECTION: HTTP endpoint implementations.

#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <math.h>

#include "esp_static_files.h"

namespace esp_routes {
namespace {

struct Ctx {
  ESP8266WebServer* server;
  EspState* state;
  esp_scan_events::ScanEvents* scan;
  esp_alert_ring::AlertRing* alerts;
};

static Ctx g;

// CONTRACT: All arm-protected routes must gate through this auth check.
static bool isArmed_() { return g.state && g.state->isArmed(); }
static void rejectNotArmed_() { g.server->send(403, "text/plain", "NOT_ARMED"); }

static const char* authStateText_(const EspState& s) {
  if (s.authPending) return "PENDING";
  if (s.authState == EspState::AuthState::Armed) return "ARMED";
  if (s.authState == EspState::AuthState::Locked) return "LOCKED";
  return "DISARMED";
}

static void sendJsonStringContent_(ESP8266WebServer& server, const char* s) {
  server.sendContent("\"");
  if (!s) {
    server.sendContent("\"");
    return;
  }
  String chunk;
  chunk.reserve(64);
  for (const char* p = s; *p; p++) {
    const char c = *p;
    if (c == '\"' || c == '\\') {
      chunk += '\\';
      chunk += c;
    } else if (c == '\n') {
      chunk += "\\n";
    } else if (c == '\r') {
      chunk += "\\r";
    } else if (c == '\t') {
      chunk += "\\t";
    } else if ((uint8_t)c < 0x20) {
      // CONTRACT: Control characters outside JSON escapes are discarded.
    } else {
      chunk += c;
    }
    if (chunk.length() >= 96) {
      server.sendContent(chunk);
      chunk = "";
    }
  }
  if (chunk.length()) server.sendContent(chunk);
  server.sendContent("\"");
}

static void sendJsonStringContent_(ESP8266WebServer& server, const String& s) {
  sendJsonStringContent_(server, s.c_str());
}

static void handleNotFound_() {
  const String uri = g.server->uri();
  if (esp_static_files::tryServeStaticUri(*g.server, uri)) return;

  String message = "404: Not Found\n\n";
  message += "URI: ";
  message += uri;
  message += "\n";
  g.server->send(404, "text/plain", message);
  Serial.println("404 for: " + uri);
}

static void handleMove_(int distance) {
  if (!isArmed_()) return rejectNotArmed_();
  Serial.println("Move: " + String(distance));
  g.server->send(200, "text/plain", "Move OK");
}

static void handleCompass_() {
  if (!isArmed_()) return rejectNotArmed_();
  if (g.server->hasArg("value")) {
    String valueString = g.server->arg("value");
    Serial.println("Turn: " + valueString);
  } else {
    Serial.println("Turn: (no value)");
  }
  g.server->send(200, "text/plain", "Compass OK");
}

static void handleEncCal_() {
  if (g.server->method() == HTTP_POST) {
    if (!isArmed_()) return rejectNotArmed_();
    Serial.println("EncCal");
    g.server->send(200, "text/plain", "STARTED");
    return;
  }

  if (!isArmed_()) return rejectNotArmed_();
  g.server->send(200, "text/plain", g.state->encCalPacket);
}

static void handleTurretZero_() {
  if (!isArmed_()) return rejectNotArmed_();
  Serial.println("TurretZero");
  g.server->send(200, "text/plain", "OK");
}

static void handleTurretTpr_() {
  if (!isArmed_()) return rejectNotArmed_();
  if (!g.server->hasArg("pos") || !g.server->hasArg("neg")) {
    g.server->send(400, "text/plain", "MISSING_POSNEG");
    return;
  }
  const uint32_t pos = (uint32_t)g.server->arg("pos").toInt();
  const uint32_t neg = (uint32_t)g.server->arg("neg").toInt();
  if (pos < 10u || pos > 200000u || neg < 10u || neg > 200000u) {
    g.server->send(400, "text/plain", "BAD_RANGE");
    return;
  }
  Serial.print("TurretTpr:");
  Serial.print((unsigned long)pos);
  Serial.print(",");
  Serial.println((unsigned long)neg);
  g.server->send(200, "text/plain", "OK");
}

static void handleTurretScanPlus_() {
  if (!isArmed_()) return rejectNotArmed_();
  (void)g.scan->allocOnce();
  g.scan->reset();
  g.scan->pushBegin(+1);
  Serial.println("TurretScanPlus");
  g.server->send(200, "text/plain", "OK");
}

static void handleTurretScanMinus_() {
  if (!isArmed_()) return rejectNotArmed_();
  (void)g.scan->allocOnce();
  g.scan->reset();
  g.scan->pushBegin(-1);
  Serial.println("TurretScanMinus");
  g.server->send(200, "text/plain", "OK");
}

static void handleTurretScanCancel_() {
  if (!isArmed_()) return rejectNotArmed_();
  g.scan->pushCancel();
  Serial.println("TurretScanCancel");
  g.server->send(200, "text/plain", "OK");
}

// WHY: /maze owns mission control; route set here provides transport primitives only.

static void handleArm_() {
  if (g.state->authState == EspState::AuthState::Locked) {
    g.server->send(423, "text/plain", "LOCKED");
    return;
  }

  String code;
  if (g.server->hasArg("code")) code = g.server->arg("code");
  code.trim();
  if (code.length() == 0) {
    g.server->send(400, "text/plain", "MISSING_CODE");
    return;
  }

  g.state->authPending = true;

  Serial.print("Passcode:");
  Serial.println(code);
  g.server->send(202, "text/plain", "PENDING");
}

static void handleDisarm_() {
  g.state->authPending = true;
  Serial.println("Disarm");
  g.server->send(202, "text/plain", "PENDING");
}

static void handleAuthState_() {
  // SECTION: Auth and link state endpoint.
  const uint32_t now = millis();
  const uint32_t ageMs = g.state->hasMegaRx ? (now - g.state->lastMegaRxMs) : 0xFFFFFFFFUL;
  const char* auth = authStateText_(*g.state);

  g.server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  g.server->sendHeader("Cache-Control", "no-store");
  g.server->send(200, "application/json", "");

  g.server->sendContent("{\"auth\":");
  sendJsonStringContent_(*g.server, auth);
  g.server->sendContent(",\"tries_left\":");
  g.server->sendContent(String((unsigned)g.state->triesLeft));
  g.server->sendContent(",\"mega_age_ms\":");
  g.server->sendContent(String((unsigned long)ageMs));
  g.server->sendContent("}");
}

static void handleTelemetry_() {
  if (!isArmed_()) return rejectNotArmed_();
  // CONTRACT: Snapshot endpoint serves cached ESP state only (no blocking round-trip).
  const uint32_t now = millis();
  const uint32_t ageMs = g.state->hasMegaRx ? (now - g.state->lastMegaRxMs) : 0xFFFFFFFFUL;

  const char* auth = authStateText_(*g.state);

  g.server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  g.server->sendHeader("Cache-Control", "no-store");
  g.server->send(200, "application/json", "");

  g.server->sendContent("{\"auth\":");
  sendJsonStringContent_(*g.server, auth);
  g.server->sendContent(",\"tries_left\":");
  g.server->sendContent(String((unsigned)g.state->triesLeft));
  g.server->sendContent(",\"mega_age_ms\":");
  g.server->sendContent(String((unsigned long)ageMs));

  g.server->sendContent(",\"lidar\":");
  sendJsonStringContent_(*g.server, g.state->lidarPacket);
  g.server->sendContent(",\"compass\":");
  sendJsonStringContent_(*g.server, g.state->compassData);
  g.server->sendContent(",\"odom\":");
  sendJsonStringContent_(*g.server, g.state->odomPacket);
  g.server->sendContent(",\"rgb\":");
  sendJsonStringContent_(*g.server, g.state->rgbPacket);
  g.server->sendContent(",\"rgb_class\":");
  sendJsonStringContent_(*g.server, g.state->rgbClassPacket);
  g.server->sendContent(",\"rgb_refs\":");
  sendJsonStringContent_(*g.server, g.state->rgbRefsPacket);
  g.server->sendContent(",\"enc_cal\":");
  sendJsonStringContent_(*g.server, g.state->encCalPacket);
  g.server->sendContent(",\"turret_cal\":");
  sendJsonStringContent_(*g.server, g.state->turretCalPacket);
  g.server->sendContent("}");
}

static void handleEvents_() {
  if (!isArmed_()) return rejectNotArmed_();
  uint32_t from = 0;
  if (g.server->hasArg("from")) from = (uint32_t)g.server->arg("from").toInt();
  g.scan->serveHttp(*g.server, from);
}

static void handleAlerts_() {
  if (!isArmed_()) return rejectNotArmed_();
  uint32_t from = 0;
  if (g.server->hasArg("from")) from = (uint32_t)g.server->arg("from").toInt();
  g.alerts->serveHttp(*g.server, from);
}

static void handleAlertsTail_() {
  if (!isArmed_()) return rejectNotArmed_();
  const uint32_t tail = g.alerts ? g.alerts->newestSeq() : 0;
  g.server->sendHeader("Cache-Control", "no-store");
  g.server->send(200, "text/plain", String((unsigned long)tail));
}

static void handleMoveCm_() {
  // SECTION: Motion command routes.
  if (!isArmed_()) return rejectNotArmed_();
  if (!g.server->hasArg("cm")) {
    g.server->send(400, "text/plain", "MISSING_CM");
    return;
  }
  const int cm = g.server->arg("cm").toInt();
  Serial.print("Move:");
  Serial.println(cm);
  g.server->send(200, "text/plain", "OK");
}

static void handleTurnDeg_() {
  if (!isArmed_()) return rejectNotArmed_();
  if (!g.server->hasArg("deg")) {
    g.server->send(400, "text/plain", "MISSING_DEG");
    return;
  }
  const int deg = g.server->arg("deg").toInt();
  Serial.print("Turn:");
  Serial.println(deg);
  g.server->send(200, "text/plain", "OK");
}

static void handleTurnDegShortest_() {
  if (!isArmed_()) return rejectNotArmed_();
  if (!g.server->hasArg("deg")) {
    g.server->send(400, "text/plain", "MISSING_DEG");
    return;
  }
  int deg = g.server->arg("deg").toInt();
  // CONTRACT: turn_short always issues shortest-arc turns.
  deg %= 360;
  if (deg > 180) deg -= 360;
  if (deg < -180) deg += 360;
  Serial.print("TurnShort:");
  Serial.println(deg);
  g.server->send(200, "text/plain", "OK");
}

static void handleSetPose_() {
  // CONTRACT: `/set_pose` updates Mega map alignment; payload must be finite.
  if (!isArmed_()) return rejectNotArmed_();

  if (!g.server->hasArg("x") || !g.server->hasArg("y")) {
    g.server->send(400, "text/plain", "MISSING_XY");
    return;
  }

  const String xs = g.server->arg("x");
  const String ys = g.server->arg("y");
  const float x = xs.toFloat();
  const float y = ys.toFloat();
  if (!(isfinite(x) && isfinite(y))) {
    g.server->send(400, "text/plain", "BAD_XY");
    return;
  }

  bool hasH = false;
  float h = 0.0f;
  if (g.server->hasArg("h")) {
    const String hs = g.server->arg("h");
    h = hs.toFloat();
    if (isfinite(h)) hasH = true;
  }

  Serial.print("MapPose:");
  Serial.print(x, 2);
  Serial.print(",");
  Serial.print(y, 2);
  if (hasH) {
    Serial.print(",");
    Serial.print(h, 2);
  }
  Serial.println();

  g.server->send(200, "text/plain", "OK");
}

}

void registerRoutes(ESP8266WebServer& server,
                    EspState& state,
                    esp_scan_events::ScanEvents& scan,
                    esp_alert_ring::AlertRing& alerts) {
  // SECTION: Route wiring table.
  g.server = &server;
  g.state = &state;
  g.scan = &scan;
  g.alerts = &alerts;

  server.on("/", []() { esp_static_files::serveFile(*g.server, "/esp/index.html", "text/html"); });
  server.on("/index.html", []() { esp_static_files::serveFile(*g.server, "/esp/index.html", "text/html"); });
  server.on("/style.css", []() { esp_static_files::serveFile(*g.server, "/esp/style.css", "text/css"); });
  server.on("/favicon.ico", []() { esp_static_files::serveFile(*g.server, "/esp/favicon.png", "image/png"); });

  server.on("/forwards5", []() { handleMove_(5); });
  server.on("/forwards20", []() { handleMove_(20); });
  server.on("/backwards5", []() { handleMove_(-5); });
  server.on("/backwards20", []() { handleMove_(-20); });
  server.on("/compass", handleCompass_);
  server.on("/maze", HTTP_GET, []() { esp_static_files::serveFile(*g.server, "/esp/maze.html", "text/html"); });
  server.on("/arm", HTTP_POST, handleArm_);
  server.on("/disarm", HTTP_POST, handleDisarm_);
  server.on("/auth_state", HTTP_GET, handleAuthState_);
  server.on("/enc_cal", handleEncCal_);
  server.on("/turret_zero", HTTP_POST, handleTurretZero_);
  server.on("/turret_tpr", HTTP_POST, handleTurretTpr_);
  server.on("/turret_tpr", HTTP_GET, handleTurretTpr_);
  server.on("/scan_plus", HTTP_POST, handleTurretScanPlus_);
  server.on("/scan_minus", HTTP_POST, handleTurretScanMinus_);
  server.on("/scan_cancel", HTTP_POST, handleTurretScanCancel_);
  server.on("/events", HTTP_GET, handleEvents_);
  server.on("/alerts", HTTP_GET, handleAlerts_);
  server.on("/alerts_tail", HTTP_GET, handleAlertsTail_);
  server.on("/set_pose", HTTP_POST, handleSetPose_);
  server.on("/move", HTTP_POST, handleMoveCm_);
  server.on("/turn", HTTP_POST, handleTurnDeg_);
  server.on("/turn_short", HTTP_POST, handleTurnDegShortest_);
  server.on("/telemetry", HTTP_GET, handleTelemetry_);

  server.onNotFound(handleNotFound_);
}

}
