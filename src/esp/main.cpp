#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <math.h>
#include <string.h>

// Optional local secrets file (gitignored): src/esp/wifi_secrets.h
// Use this to avoid command-line quoting issues with spaces/& in passwords.
#if defined(__has_include)
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#endif

// ====== Wi-Fi credentials (compile-time, override locally) ======
// Keep secrets out of source control via `platformio_override.ini`:
// -DWIFI_SSID="Your2p4GHzSSID"
// -DWIFI_PASSWORD="YourPassword"
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

// Fallback AP when STA credentials are not provided or STA connect fails.
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "mobile-robot"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD ""
#endif

static const char* kWifiSsid = WIFI_SSID;
static const char* kWifiPassword = WIFI_PASSWORD;
static const char* kWifiApSsid = WIFI_AP_SSID;
static const char* kWifiApPassword = WIFI_AP_PASSWORD;

String lidarData = "0 cm";
String lidarPacket = "LIDAR:0,SEQ:0,T:0";
String compassData = "0,N";
String rgbPacket = "RGB:0,0,0";
String encCalPacket = "ENC_CAL:--";
String turretCalPacket = "TURCAL:--";
String odomPacket = "ODOM:0.0,0.0";

ESP8266WebServer server(80);
static String serialLineBuf;
static const size_t kMaxSerialLineLen = 128;

// ====== Option 1: Passcode arming (UART) ======
enum class AuthState : uint8_t { Disarmed, Armed, Locked };
static AuthState gAuthState = AuthState::Disarmed;
static uint8_t gTriesLeft = 3;
static bool gAuthPending = false;
static const uint8_t kMaxTries = 3;
static uint32_t gLastMegaRxMs = 0;
static bool gHasMegaRx = false;

// ====== Function Declarations ======
void handleNotFound();
void handleMove(int distance);
void handleNorth();
void handleSetNorth();
void handleCompass();
void handleLidar();
void handleCompassData();
void handleRgb();
void handleMaze();
void handleMazeCommand();
void handleArm();
void handleDisarm();
void handleAuth();
void handleEncCal();
void handleTurretCal();
void handleTurretCalStart();
void handleTurretCalDone();
void handleTurretZero();
void handleTurretScanPlus();
void handleTurretScanMinus();
void handleTurretScanCancel();
void handleStatus();
void handleEvents();
void handleOdom();
void listAllFiles();
void processSerialLine(const String& data);
void pollSerialNonBlocking();
void serveFile(const char* path, const char* contentType);
bool tryServeStaticUri(const String& uri);
const char* contentTypeForPath(const String& path);

// ====== Event polling fallback (robust) ======
// NOTE:
// We need to preserve *all* turret scan points for browser-side matching.
// A small ring buffer can overwrite points during a sweep, producing unstable
// localization. Instead, we keep a dedicated per-scan event buffer that holds
// the full scan transaction (BEGIN + samples + DONE/CANCEL).
//
// This is still served via /events?from=<seq> with the same "seq|LINE" format,
// so the browser polling logic doesn't change.
static const uint16_t kScanMaxEvents = 4096;
enum class ScanEvtKind : uint8_t { None = 0, Begin = 1, Sample = 2, Done = 3, Cancel = 4, Overflow = 5 };
static uint32_t gScanNextSeq = 1;
static uint16_t gScanCount = 0;         // number of recorded events
static bool gScanHasData = false;       // true after first BEGIN is stored
static bool gScanOverflow = false;      // set if we exceed kScanMaxEvents (should never happen in normal configs)
static ScanEvtKind gScanKind[kScanMaxEvents];
static int8_t gScanDir[kScanMaxEvents];        // +1 / -1 for BEGIN, else 0
static uint16_t gScanAngleCdeg[kScanMaxEvents]; // centi-degrees [0..36000]
static uint16_t gScanDistMm[kScanMaxEvents];    // millimeters

static void resetScanBuffer_() {
  gScanNextSeq = 1;
  gScanCount = 0;
  gScanHasData = false;
  gScanOverflow = false;
  // No need to clear arrays; gScanCount gates reads.
}

static void pushScanBegin_(int dirSign) {
  if (gScanCount >= kScanMaxEvents) { gScanOverflow = true; return; }
  gScanKind[gScanCount] = ScanEvtKind::Begin;
  gScanDir[gScanCount] = (dirSign < 0) ? -1 : 1;
  gScanAngleCdeg[gScanCount] = 0;
  gScanDistMm[gScanCount] = 0;
  gScanCount++;
  gScanNextSeq++;
  gScanHasData = true;
}

static void pushScanDone_() {
  if (gScanCount >= kScanMaxEvents) { gScanOverflow = true; return; }
  gScanKind[gScanCount] = ScanEvtKind::Done;
  gScanDir[gScanCount] = 0;
  gScanAngleCdeg[gScanCount] = 0;
  gScanDistMm[gScanCount] = 0;
  gScanCount++;
  gScanNextSeq++;
  gScanHasData = true;
}

static void pushScanCancel_() {
  if (gScanCount >= kScanMaxEvents) { gScanOverflow = true; return; }
  gScanKind[gScanCount] = ScanEvtKind::Cancel;
  gScanDir[gScanCount] = 0;
  gScanAngleCdeg[gScanCount] = 0;
  gScanDistMm[gScanCount] = 0;
  gScanCount++;
  gScanNextSeq++;
  gScanHasData = true;
}

static void pushScanSample_(float angleDeg, float distCm) {
  if (gScanCount >= kScanMaxEvents) { gScanOverflow = true; return; }
  if (!(isfinite(angleDeg) && isfinite(distCm))) return;
  // Normalize/quantize to keep memory small and deterministic.
  // Angle: centi-deg in [0..36000].
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  int a = (int)lroundf(angleDeg * 100.0f);
  if (a < 0) a = 0;
  if (a > 36000) a = 36000;
  // Dist: mm (supports up to ~65m, plenty).
  int dmm = (int)lroundf(distCm * 10.0f);
  if (dmm < 0) dmm = 0;
  if (dmm > 65535) dmm = 65535;

  gScanKind[gScanCount] = ScanEvtKind::Sample;
  gScanDir[gScanCount] = 0;
  gScanAngleCdeg[gScanCount] = (uint16_t)a;
  gScanDistMm[gScanCount] = (uint16_t)dmm;
  gScanCount++;
  gScanNextSeq++;
  gScanHasData = true;
}

static bool isArmed() { return gAuthState == AuthState::Armed; }
static void rejectNotArmed() { server.send(403, "text/plain", "NOT_ARMED"); }

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
      if (attempt >= 30) break;  // ~15s then fall back to AP
    }

    if (WiFi.status() == WL_CONNECTED) {
      started = true;
      Serial.println("\nWiFi connected (STA).");
      Serial.print("IP address: ");
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
  }

  server.on("/", []() { serveFile("/esp/index.html", "text/html"); });
  server.on("/index.html", []() { serveFile("/esp/index.html", "text/html"); });
  server.on("/style.css", []() { serveFile("/esp/style.css", "text/css"); });
  server.on("/script.js", []() { serveFile("/esp/script.js", "application/javascript"); });
  server.on("/maze.js", []() { serveFile("/esp/maze.js", "application/javascript"); });
  server.on("/favicon.ico", []() { serveFile("/esp/favicon.png", "image/png"); });

  server.on("/forwards5", []() { handleMove(5); });
  server.on("/forwards20", []() { handleMove(20); });
  server.on("/backwards5", []() { handleMove(-5); });
  server.on("/backwards20", []() { handleMove(-20); });
  server.on("/compass", handleCompass);
  server.on("/north", []() { handleNorth(); });
  server.on("/setnorth", []() { handleSetNorth(); });
  server.on("/maze", HTTP_GET, []() { serveFile("/esp/maze.html", "text/html"); });
  server.on("/maze", HTTP_POST, handleMazeCommand);
  server.on("/lidar", handleLidar);
  server.on("/compassdata", handleCompassData);
  server.on("/rgb", handleRgb);
  server.on("/arm", HTTP_POST, handleArm);
  server.on("/disarm", HTTP_POST, handleDisarm);
  server.on("/auth", handleAuth);
  server.on("/enc_cal", handleEncCal);
  server.on("/turret_cal", handleTurretCal);
  server.on("/turret_cal_start", HTTP_POST, handleTurretCalStart);
  server.on("/turret_cal_done", HTTP_POST, handleTurretCalDone);
  server.on("/turret_zero", HTTP_POST, handleTurretZero);
  server.on("/scan_plus", HTTP_POST, handleTurretScanPlus);
  server.on("/scan_minus", HTTP_POST, handleTurretScanMinus);
  server.on("/scan_cancel", HTTP_POST, handleTurretScanCancel);
  server.on("/events", HTTP_GET, handleEvents);
  server.on("/odom", HTTP_GET, handleOdom);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("✅ Web server started!");
}

void loop() {
  server.handleClient();
  pollSerialNonBlocking();
}

void processSerialLine(const String& data) {
  // Any valid line from the Mega means the UART link is alive.
  gLastMegaRxMs = millis();
  gHasMegaRx = true;

  if (data.startsWith("AUTH:")) {
    // Expected formats:
    // - AUTH:OK
    // - AUTH:OFF
    // - AUTH:LOCKED
    // - AUTH:FAIL:<triesLeft>
    const String s = data.substring(5);
    if (s.startsWith("OK")) {
      gAuthState = AuthState::Armed;
      gTriesLeft = kMaxTries;
      gAuthPending = false;
    } else if (s.startsWith("OFF")) {
      gAuthState = AuthState::Disarmed;
      gAuthPending = false;
    } else if (s.startsWith("LOCKED")) {
      gAuthState = AuthState::Locked;
      gAuthPending = false;
    } else if (s.startsWith("FAIL:")) {
      const int tries = s.substring(5).toInt();
      if (tries >= 0 && tries <= 255) gTriesLeft = static_cast<uint8_t>(tries);
      gAuthState = AuthState::Disarmed;
      gAuthPending = false;
    }
    return;
  }

  if (data.startsWith("LIDAR:")) {
    lidarPacket = data;
    String payload = lidarPacket;
    int comma = payload.indexOf(',');
    if (comma >= 0) payload = payload.substring(0, comma);
    lidarData = payload;
  } else if (data.startsWith("COMPASS:")) {
    compassData = data.substring(8);
  } else if (data.startsWith("RGB:")) {
    rgbPacket = data;
  } else if (data.startsWith("ENC_CAL:")) {
    encCalPacket = data;
  } else if (data.startsWith("TURCAL:")) {
    turretCalPacket = data;
  } else if (data.startsWith("ODOM:")) {
    odomPacket = data;
  }

  // Push only scan lines into the events ring. This keeps /events dedicated to
  // scan transactions (and avoids ring overflow from background telemetry).
  if (data.startsWith("TSCAN:")) {
    // Expected formats (from Mega):
    // - TSCAN:BEGIN,+
    // - TSCAN:BEGIN,-
    // - TSCAN:DONE
    // - TSCAN:CANCEL
    // - TSCAN:<angleDeg>,<distCm>
    const String payload = data.substring(6); // after "TSCAN:"
    if (payload.startsWith("BEGIN,")) {
      const int dir = payload.indexOf('-') >= 0 ? -1 : 1;
      pushScanBegin_(dir);
    } else if (payload.startsWith("DONE")) {
      pushScanDone_();
    } else if (payload.startsWith("CANCEL")) {
      pushScanCancel_();
    } else {
      const int comma = payload.indexOf(',');
      if (comma > 0) {
        const float a = payload.substring(0, comma).toFloat();
        const float d = payload.substring(comma + 1).toFloat();
        pushScanSample_(a, d);
      }
    }
  }
}

void pollSerialNonBlocking() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (serialLineBuf.length() > 0) {
        processSerialLine(serialLineBuf);
        serialLineBuf = "";
      }
      continue;
    }

    if (serialLineBuf.length() >= kMaxSerialLineLen) {
      serialLineBuf = "";
      continue;
    }
    serialLineBuf += c;
  }
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

void handleNotFound() {
  const String uri = server.uri();
  if (tryServeStaticUri(uri)) return;

  String message = "404: Not Found\n\n";
  message += "URI: ";
  message += uri;
  message += "\n";
  server.send(404, "text/plain", message);
  Serial.println("404 for: " + uri);
}

void handleMove(int distance) {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("Move: " + String(distance));
  server.send(200, "text/plain", "Move OK");
}

void handleCompass() {
  if (!isArmed()) return rejectNotArmed();
  if (server.hasArg("value")) {
    String valueString = server.arg("value");
    Serial.println("Turn: " + valueString);
  } else {
    Serial.println("Turn: (no value)");
  }
  server.send(200, "text/plain", "Compass OK");
}

void handleNorth() {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("North");
  server.send(200, "text/plain", "North OK");
}

void handleSetNorth() {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("SetNorth");
  server.send(200, "text/plain", "SetNorth OK");
}

void handleLidar() {
  if (!isArmed()) {
    server.send(200, "text/plain", "LIDAR:--");
    return;
  }
  server.send(200, "text/plain", lidarPacket);
}

void handleCompassData() {
  if (!isArmed()) {
    server.send(200, "text/plain", "--");
    return;
  }
  server.send(200, "text/plain", compassData);
}

void handleRgb() {
  if (!isArmed()) {
    server.send(200, "text/plain", "RGB:--,--,--");
    return;
  }
  server.send(200, "text/plain", rgbPacket);
}

void handleEncCal() {
  if (server.method() == HTTP_POST) {
    if (!isArmed()) return rejectNotArmed();
    Serial.println("EncCal");
    server.send(200, "text/plain", "STARTED");
    return;
  }

  // GET: show last calibrated value (mm/pulse) cached from Mega.
  if (!isArmed()) {
    server.send(200, "text/plain", "--");
    return;
  }
  server.send(200, "text/plain", encCalPacket);
}

void handleTurretCal() {
  // GET: show last calibrated ticks/rev cached from Mega.
  if (!isArmed()) {
    server.send(200, "text/plain", "--");
    return;
  }
  server.send(200, "text/plain", turretCalPacket);
}

void handleTurretCalStart() {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("TurretCalStart");
  server.send(200, "text/plain", "STARTED");
}

void handleTurretCalDone() {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("TurretCalDone");
  server.send(200, "text/plain", "DONE");
}

void handleTurretZero() {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("TurretZero");
  server.send(200, "text/plain", "OK");
}

void handleTurretScanPlus() {
  if (!isArmed()) return rejectNotArmed();
  resetScanBuffer_();
  pushScanBegin_(+1);  // synthetic BEGIN so browser sees it immediately
  Serial.println("TurretScanPlus");
  server.send(200, "text/plain", "OK");
}

void handleTurretScanMinus() {
  if (!isArmed()) return rejectNotArmed();
  resetScanBuffer_();
  pushScanBegin_(-1);  // synthetic BEGIN so browser sees it immediately
  Serial.println("TurretScanMinus");
  server.send(200, "text/plain", "OK");
}

void handleTurretScanCancel() {
  if (!isArmed()) return rejectNotArmed();
  pushScanCancel_(); // synthetic
  Serial.println("TurretScanCancel");
  server.send(200, "text/plain", "OK");
}

void handleMaze() {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("Maze");
  server.send(200, "text/plain", "Maze OK");
}

void handleMazeCommand() {
  // Preserve the existing /maze behavior as a command endpoint.
  handleMaze();
}

void handleArm() {
  if (gAuthState == AuthState::Locked) {
    server.send(423, "text/plain", "LOCKED");
    return;
  }

  String code;
  if (server.hasArg("code")) code = server.arg("code");
  code.trim();
  if (code.length() == 0) {
    server.send(400, "text/plain", "MISSING_CODE");
    return;
  }

  gAuthPending = true;

  Serial.print("Passcode:");
  Serial.println(code);

  // Non-blocking: UI should poll /auth for the final state.
  server.send(202, "text/plain", "PENDING");
}

void handleDisarm() {
  gAuthPending = true;
  Serial.println("Disarm");

  // Non-blocking: UI should poll /auth for the final state.
  server.send(202, "text/plain", "PENDING");
}

void handleAuth() {
  if (gAuthPending) {
    server.send(200, "text/plain", "PENDING");
    return;
  }
  if (gAuthState == AuthState::Armed) {
    server.send(200, "text/plain", "ARMED");
    return;
  }
  if (gAuthState == AuthState::Locked) {
    server.send(200, "text/plain", "LOCKED");
    return;
  }
  server.send(200, "text/plain", String("DISARMED:") + String(gTriesLeft));
}

void handleStatus() {
  const uint32_t now = millis();
  const uint32_t ageMs = gHasMegaRx ? (now - gLastMegaRxMs) : 0xFFFFFFFFUL;

  const char* auth = "DISARMED";
  if (gAuthPending) auth = "PENDING";
  else if (gAuthState == AuthState::Armed) auth = "ARMED";
  else if (gAuthState == AuthState::Locked) auth = "LOCKED";

  const IPAddress ip = (WiFi.getMode() & WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();

  String out;
  out.reserve(180);
  out += "{";
  out += "\"auth\":\"";
  out += auth;
  out += "\",";
  out += "\"tries_left\":";
  out += String((unsigned)gTriesLeft);
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

  server.send(200, "application/json", out);
}

void serveFile(const char* path, const char* contentType) {
  File file = LittleFS.open(path, "r");
  // Support both filesystem layouts:
  // - older: files stored under /esp/*
  // - newer/clean uploadfs: files stored at the filesystem root
  if (!file && path && strncmp(path, "/esp/", 5) == 0) {
    file = LittleFS.open(path + 4, "r");  // strip "/esp"
  }
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  // Avoid stale UI code during development; ESP caching behavior varies by client.
  // (If you want caching later, make it explicit with versioned asset URLs.)
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(file, contentType);
  file.close();
}

void handleEvents() {
  if (!isArmed()) return rejectNotArmed();

  uint32_t from = 0;
  if (server.hasArg("from")) from = (uint32_t)server.arg("from").toInt();

  // Return lines in chronological order with "seq|LINE" per row.
  // seq numbering is 1-based and monotonic within the current scan buffer.
  //
  // This buffer is per-scan and large enough to hold all samples, ensuring
  // the browser gets the full sweep even if it polls slowly.
  String out;
  out.reserve(2400);

  if (gScanOverflow) {
    // Warn the client that data is incomplete.
    // Keep format consistent (seq|LINE).
    out += String(from + 1);
    out += "|TSCAN:OVERFLOW\n";
    server.send(200, "text/plain", out);
    return;
  }

  if (!gScanHasData || gScanCount == 0) {
    server.send(200, "text/plain", "");
    return;
  }

  // seq == index+1
  for (uint16_t i = 0; i < gScanCount; ++i) {
    const uint32_t seq = (uint32_t)i + 1u;
    if (seq <= from) continue;

    out += String(seq);
    out += "|";

    const ScanEvtKind k = gScanKind[i];
    if (k == ScanEvtKind::Begin) {
      out += "TSCAN:BEGIN,";
      out += (gScanDir[i] < 0) ? "-" : "+";
      out += "\n";
    } else if (k == ScanEvtKind::Done) {
      out += "TSCAN:DONE\n";
    } else if (k == ScanEvtKind::Cancel) {
      out += "TSCAN:CANCEL\n";
    } else if (k == ScanEvtKind::Sample) {
      const float a = ((float)gScanAngleCdeg[i]) / 100.0f;
      const float d = ((float)gScanDistMm[i]) / 10.0f;
      out += "TSCAN:";
      out += String(a, 2);
      out += ",";
      out += String(d, 1);
      out += "\n";
    } else {
      // ignore None/Overflow markers
      out += "\n";
    }

    // Avoid huge responses; client will keep polling with updated "from".
    if (out.length() >= 1800) break;
  }

  server.send(200, "text/plain", out);
}

void handleOdom() {
  if (!isArmed()) {
    server.send(200, "text/plain", "ODOM:--");
    return;
  }
  server.send(200, "text/plain", odomPacket);
}

const char* contentTypeForPath(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css")) return "text/css";
  if (path.endsWith(".js")) return "application/javascript";
  if (path.endsWith(".png")) return "image/png";
  if (path.endsWith(".svg")) return "image/svg+xml";
  if (path.endsWith(".ico")) return "image/x-icon";
  if (path.endsWith(".json")) return "application/json";
  return "text/plain";
}

bool tryServeStaticUri(const String& uri) {
  if (uri.length() == 0) return false;
  if (uri == "/") return false;
  if (uri == "/events") return false;
  if (uri.indexOf("..") >= 0) return false;

  // If client asks "/foo", treat as "/foo" and also allow "/esp/foo" style.
  const char* ct = contentTypeForPath(uri);
  String pathToServe = uri;
  File f = LittleFS.open(pathToServe, "r");
  if (!f && uri.startsWith("/")) {
    // also try legacy /esp prefix
    pathToServe = String("/esp") + uri;
    f = LittleFS.open(pathToServe, "r");
  }
  if (!f) return false;
  f.close();

  serveFile(pathToServe.c_str(), ct);
  return true;
}
