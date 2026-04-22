#include "esp_routes.h"
// (Refactor) HTTP module implementation.

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

static bool isArmed_() { return g.state && g.state->isArmed(); }
static void rejectNotArmed_() { g.server->send(403, "text/plain", "NOT_ARMED"); }

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

static void handleNorth_() {
  if (!isArmed_()) return rejectNotArmed_();
  Serial.println("North");
  g.server->send(200, "text/plain", "North OK");
}

static void handleSetNorth_() {
  if (!isArmed_()) return rejectNotArmed_();
  Serial.println("SetNorth");
  g.server->send(200, "text/plain", "SetNorth OK");
}

static void handleLidar_() {
  if (!isArmed_()) {
    g.server->send(200, "text/plain", "LIDAR:--");
    return;
  }
  g.server->send(200, "text/plain", g.state->lidarPacket);
}

static void handleCompassData_() {
  if (!isArmed_()) {
    g.server->send(200, "text/plain", "--");
    return;
  }
  g.server->send(200, "text/plain", g.state->compassData);
}

static void handleRgb_() {
  if (!isArmed_()) {
    g.server->send(200, "text/plain", "RGB:--,--,--");
    return;
  }
  g.server->send(200, "text/plain", g.state->rgbPacket);
}

static void handleRgbClass_() {
  if (!isArmed_()) {
    g.server->send(200, "text/plain", "CLASS:0");
    return;
  }
  g.server->send(200, "text/plain", g.state->rgbClassPacket);
}

static void handleRgbRefs_() {
  if (!isArmed_()) {
    g.server->send(200, "text/plain", "REFS:--");
    return;
  }
  g.server->send(200, "text/plain", g.state->rgbRefsPacket);
}

static void handleEncCal_() {
  if (g.server->method() == HTTP_POST) {
    if (!isArmed_()) return rejectNotArmed_();
    Serial.println("EncCal");
    g.server->send(200, "text/plain", "STARTED");
    return;
  }

  if (!isArmed_()) {
    g.server->send(200, "text/plain", "--");
    return;
  }
  g.server->send(200, "text/plain", g.state->encCalPacket);
}

static void handleTurretCal_() {
  if (!isArmed_()) {
    g.server->send(200, "text/plain", "--");
    return;
  }
  g.server->send(200, "text/plain", g.state->turretCalPacket);
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

static void handleMaze_() {
  if (!isArmed_()) return rejectNotArmed_();
  Serial.println("Maze");
  g.server->send(200, "text/plain", "Maze OK");
}

static void handleMazeCommand_() { handleMaze_(); }

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

static void handleAuth_() {
  if (g.state->authPending) {
    g.server->send(200, "text/plain", "PENDING");
    return;
  }
  if (g.state->authState == EspState::AuthState::Armed) {
    g.server->send(200, "text/plain", "ARMED");
    return;
  }
  if (g.state->authState == EspState::AuthState::Locked) {
    g.server->send(200, "text/plain", "LOCKED");
    return;
  }
  g.server->send(200, "text/plain", String("DISARMED:") + String(g.state->triesLeft));
}

static void handleStatus_() {
  const uint32_t now = millis();
  const uint32_t ageMs = g.state->hasMegaRx ? (now - g.state->lastMegaRxMs) : 0xFFFFFFFFUL;

  const char* auth = "DISARMED";
  if (g.state->authPending) auth = "PENDING";
  else if (g.state->authState == EspState::AuthState::Armed) auth = "ARMED";
  else if (g.state->authState == EspState::AuthState::Locked) auth = "LOCKED";

  const IPAddress ip = (WiFi.getMode() & WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();

  String out;
  out.reserve(180);
  out += "{";
  out += "\"auth\":\"";
  out += auth;
  out += "\",";
  out += "\"tries_left\":";
  out += String((unsigned)g.state->triesLeft);
  out += ",";
  out += "\"mega_age_ms\":";
  out += String((unsigned long)ageMs);
  out += ",";
  out += "\"ip\":\"";
  out += ip.toString();
  out += "\",";
  out += "\"mode\":\"";
  out += (WiFi.getMode() & WIFI_AP) ? "ap" : "sta";
  out += "\"";
  out += "}";

  g.server->send(200, "application/json", out);
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

static void handleOdom_() {
  if (!isArmed_()) {
    g.server->send(200, "text/plain", "ODOM:--");
    return;
  }
  g.server->send(200, "text/plain", g.state->odomPacket);
}

static void handleMoveCm_() {
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

static void handleSetPose_() {
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

}  // namespace

void registerRoutes(ESP8266WebServer& server,
                    EspState& state,
                    esp_scan_events::ScanEvents& scan,
                    esp_alert_ring::AlertRing& alerts) {
  g.server = &server;
  g.state = &state;
  g.scan = &scan;
  g.alerts = &alerts;

  server.on("/", []() { esp_static_files::serveFile(*g.server, "/esp/index.html", "text/html"); });
  server.on("/index.html", []() { esp_static_files::serveFile(*g.server, "/esp/index.html", "text/html"); });
  server.on("/style.css", []() { esp_static_files::serveFile(*g.server, "/esp/style.css", "text/css"); });
  server.on("/script.js", []() { esp_static_files::serveFile(*g.server, "/esp/script.js", "application/javascript"); });
  server.on("/maze.js", []() { esp_static_files::serveFile(*g.server, "/esp/maze.js", "application/javascript"); });
  server.on("/favicon.ico", []() { esp_static_files::serveFile(*g.server, "/esp/favicon.png", "image/png"); });

  server.on("/forwards5", []() { handleMove_(5); });
  server.on("/forwards20", []() { handleMove_(20); });
  server.on("/backwards5", []() { handleMove_(-5); });
  server.on("/backwards20", []() { handleMove_(-20); });
  server.on("/compass", handleCompass_);
  server.on("/north", []() { handleNorth_(); });
  server.on("/setnorth", []() { handleSetNorth_(); });
  server.on("/maze", HTTP_GET, []() { esp_static_files::serveFile(*g.server, "/esp/maze.html", "text/html"); });
  server.on("/maze", HTTP_POST, handleMazeCommand_);
  server.on("/lidar", handleLidar_);
  server.on("/compassdata", handleCompassData_);
  server.on("/rgb", handleRgb_);
  server.on("/rgb_class", handleRgbClass_);
  server.on("/rgb_refs", handleRgbRefs_);
  server.on("/arm", HTTP_POST, handleArm_);
  server.on("/disarm", HTTP_POST, handleDisarm_);
  server.on("/auth", handleAuth_);
  server.on("/enc_cal", handleEncCal_);
  server.on("/turret_cal", handleTurretCal_);
  server.on("/turret_zero", HTTP_POST, handleTurretZero_);
  server.on("/turret_tpr", HTTP_POST, handleTurretTpr_);
  server.on("/turret_tpr", HTTP_GET, handleTurretTpr_);
  server.on("/scan_plus", HTTP_POST, handleTurretScanPlus_);
  server.on("/scan_minus", HTTP_POST, handleTurretScanMinus_);
  server.on("/scan_cancel", HTTP_POST, handleTurretScanCancel_);
  server.on("/events", HTTP_GET, handleEvents_);
  server.on("/alerts", HTTP_GET, handleAlerts_);
  server.on("/odom", HTTP_GET, handleOdom_);
  server.on("/set_pose", HTTP_POST, handleSetPose_);
  server.on("/move", HTTP_POST, handleMoveCm_);
  server.on("/turn", HTTP_POST, handleTurnDeg_);
  server.on("/status", handleStatus_);

  server.onNotFound(handleNotFound_);
}

}  // namespace esp_routes
