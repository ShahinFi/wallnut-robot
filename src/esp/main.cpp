#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <math.h>
#include <stdlib.h>
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
String rgbClassPacket = "CLASS:0";  // 0=NONE, 1..4 = stored color index
String rgbRefsPacket = "REFS:--";   // REFS:r,g,b;... (4 refs) or "--"
String encCalPacket = "ENC_CAL:--";
String turretCalPacket = "TURCAL:--";
String odomPacket = "ODOM:0.0,0.0";

ESP8266WebServer server(80);
static const size_t kMaxSerialLineLen = 128;
static char gSerialLine[kMaxSerialLineLen + 1];
static uint8_t gSerialLineLen = 0;

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
void handleRgbClass();
void handleRgbRefs();
void handleMaze();
void handleMazeCommand();
void handleArm();
void handleDisarm();
void handleAuth();
void handleEncCal();
void handleTurretCal();
void handleTurretZero();
void handleTurretTpr();
void handleTurretScanPlus();
void handleTurretScanMinus();
void handleTurretScanCancel();
void handleStatus();
void handleEvents();
void handleOdom();
void handleSetPose();
void handleAlerts();
void handleMoveCm();
void handleTurnDeg();
void listAllFiles();
void processSerialLine(const String& data);
void pollSerialNonBlocking();
void serveFile(const char* path, const char* contentType);
bool tryServeStaticUri(const String& uri);
const char* contentTypeForPath(const String& path);

static bool parseDecFloatSpan_(const char* s, const char* e, float& out) {
  // Parse a decimal float in [s,e) without allocating or requiring NUL termination.
  // Supports optional +/- sign and a fractional part after '.'.
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
      // Cap fractional precision to keep math bounded.
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

// Large fixed global arrays can overflow ESP8266 DRAM `.bss` at link time.
// Keep the same scan buffering behavior, but allocate storage once on heap.
static ScanEvtKind* gScanKind = nullptr;
static int8_t* gScanDir = nullptr;            // +1 / -1 for BEGIN, else 0
static uint16_t* gScanAngleCdeg = nullptr;    // centi-degrees [0..36000]
static uint16_t* gScanDistMm = nullptr;       // millimeters
static uint16_t gScanCap = 0;

static void freeScanBuf_() {
  if (gScanKind) { free(gScanKind); gScanKind = nullptr; }
  if (gScanDir) { free(gScanDir); gScanDir = nullptr; }
  if (gScanAngleCdeg) { free(gScanAngleCdeg); gScanAngleCdeg = nullptr; }
  if (gScanDistMm) { free(gScanDistMm); gScanDistMm = nullptr; }
  gScanCap = 0;
}

static bool allocScanBufOnce_() {
  if (gScanCap > 0 && gScanKind && gScanDir && gScanAngleCdeg && gScanDistMm) return true;

  freeScanBuf_();
  gScanKind = (ScanEvtKind*)malloc((size_t)kScanMaxEvents * sizeof(ScanEvtKind));
  gScanDir = (int8_t*)malloc((size_t)kScanMaxEvents * sizeof(int8_t));
  gScanAngleCdeg = (uint16_t*)malloc((size_t)kScanMaxEvents * sizeof(uint16_t));
  gScanDistMm = (uint16_t*)malloc((size_t)kScanMaxEvents * sizeof(uint16_t));
  if (!(gScanKind && gScanDir && gScanAngleCdeg && gScanDistMm)) {
    freeScanBuf_();
    return false;
  }
  gScanCap = kScanMaxEvents;
  return true;
}

static void resetScanBuffer_() {
  gScanNextSeq = 1;
  gScanCount = 0;
  gScanHasData = false;
  gScanOverflow = false;
  // No need to clear arrays; gScanCount gates reads.
}

static void pushScanBegin_(int dirSign) {
  if (!gScanCap || !gScanKind || !gScanDir || !gScanAngleCdeg || !gScanDistMm) { gScanOverflow = true; return; }
  if (gScanCount >= gScanCap) { gScanOverflow = true; return; }
  gScanKind[gScanCount] = ScanEvtKind::Begin;
  gScanDir[gScanCount] = (dirSign < 0) ? -1 : 1;
  gScanAngleCdeg[gScanCount] = 0;
  gScanDistMm[gScanCount] = 0;
  gScanCount++;
  gScanNextSeq++;
  gScanHasData = true;
}

static void pushScanDone_() {
  if (!gScanCap || !gScanKind || !gScanDir || !gScanAngleCdeg || !gScanDistMm) { gScanOverflow = true; return; }
  if (gScanCount >= gScanCap) { gScanOverflow = true; return; }
  gScanKind[gScanCount] = ScanEvtKind::Done;
  gScanDir[gScanCount] = 0;
  gScanAngleCdeg[gScanCount] = 0;
  gScanDistMm[gScanCount] = 0;
  gScanCount++;
  gScanNextSeq++;
  gScanHasData = true;
}

static void pushScanCancel_() {
  if (!gScanCap || !gScanKind || !gScanDir || !gScanAngleCdeg || !gScanDistMm) { gScanOverflow = true; return; }
  if (gScanCount >= gScanCap) { gScanOverflow = true; return; }
  gScanKind[gScanCount] = ScanEvtKind::Cancel;
  gScanDir[gScanCount] = 0;
  gScanAngleCdeg[gScanCount] = 0;
  gScanDistMm[gScanCount] = 0;
  gScanCount++;
  gScanNextSeq++;
  gScanHasData = true;
}

static void pushScanSample_(float angleDeg, float distCm) {
  if (!gScanCap || !gScanKind || !gScanDir || !gScanAngleCdeg || !gScanDistMm) { gScanOverflow = true; return; }
  if (gScanCount >= gScanCap) { gScanOverflow = true; return; }
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

// ====== Alert events (reflex stops, cmd done, etc.) ======
// Separate from scan transport to keep scan bandwidth deterministic.
//
// IMPORTANT (robustness on ESP8266):
// Do NOT heap-allocate a large "string ring" here. We already allocate a big
// per-scan buffer to guarantee "keep all scan points"; adding another large
// heap allocation can fail/fragment and result in `/alerts` being permanently
// empty, which makes browser motion commands time out.
//
// Instead, store a compact parsed representation in `.bss` and reconstruct the
// wire format on demand. External behavior stays identical:
//   `/alerts?from=N` returns lines of `seq|EVT:...`.
static const uint16_t kAlertRingSize = 256;
static uint32_t gAlertNextSeq = 1;
static uint16_t gAlertHead = 0;   // next write index
static uint16_t gAlertCount = 0;  // number of valid entries (<= kAlertRingSize)

struct AlertEvt {
  uint32_t seq;           // monotonic seq
  char tag[12];           // e.g. "CMDOK", "RED" (NUL terminated)
  float x_cm;             // east (cm)
  float y_cm;             // north (cm)
  float heading_deg;      // wrapped heading (deg)
  float extra;            // optional (e.g. dist)
  bool hasExtra;
};

static AlertEvt gAlerts[kAlertRingSize];

static void resetAlertRing_() {
  gAlertNextSeq = 1;
  gAlertHead = 0;
  gAlertCount = 0;
  // No need to clear the ring; gAlertCount gates reads.
}

static void pushAlertLine_(const String& line) {
  // Expected: "EVT:TAG,x,y,h[,extra]"
  const String s = line;
  if (!s.startsWith("EVT:")) return;

  const int p0 = 4;  // after "EVT:"
  const int c1 = s.indexOf(',', p0);
  if (c1 < 0) return;
  const int c2 = s.indexOf(',', c1 + 1);
  const int c3 = (c2 >= 0) ? s.indexOf(',', c2 + 1) : -1;
  const int c4 = (c3 >= 0) ? s.indexOf(',', c3 + 1) : -1;

  // Must have at least TAG,x,y,h (3 commas after tag)
  if (c2 < 0 || c3 < 0) return;

  AlertEvt& e = gAlerts[gAlertHead];
  e.seq = gAlertNextSeq++;

  // Tag
  const String tag = s.substring(p0, c1);
  tag.toCharArray(e.tag, sizeof(e.tag));
  e.tag[sizeof(e.tag) - 1] = '\0';

  // Numbers (best-effort; keep NaNs if malformed)
  e.x_cm = s.substring(c1 + 1, c2).toFloat();
  e.y_cm = s.substring(c2 + 1, c3).toFloat();
  if (c4 < 0) {
    e.heading_deg = s.substring(c3 + 1).toFloat();
    e.hasExtra = false;
    e.extra = NAN;
  } else {
    e.heading_deg = s.substring(c3 + 1, c4).toFloat();
    e.extra = s.substring(c4 + 1).toFloat();
    e.hasExtra = true;
  }

  gAlertHead = (uint16_t)((gAlertHead + 1) % kAlertRingSize);
  if (gAlertCount < kAlertRingSize) gAlertCount++;
}

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

  // Allocate large scan/alert buffers on heap to avoid ESP8266 `.bss` overflow.
  (void)allocScanBufOnce_();
  resetAlertRing_();

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
      // Report to Mega (UART protocol) for LCD display (best-effort).
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
    // Report to Mega (UART protocol) for LCD display (best-effort).
    Serial.print("ESPIP:");
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
  server.on("/rgb_class", handleRgbClass);
  server.on("/rgb_refs", handleRgbRefs);
  server.on("/arm", HTTP_POST, handleArm);
  server.on("/disarm", HTTP_POST, handleDisarm);
  server.on("/auth", handleAuth);
  server.on("/enc_cal", handleEncCal);
  server.on("/turret_cal", handleTurretCal);
  server.on("/turret_zero", HTTP_POST, handleTurretZero);
  // Accept both POST and GET for robustness (UI uses POST; GET is handy for quick manual testing).
  server.on("/turret_tpr", HTTP_POST, handleTurretTpr);
  server.on("/turret_tpr", HTTP_GET, handleTurretTpr);
  server.on("/scan_plus", HTTP_POST, handleTurretScanPlus);
  server.on("/scan_minus", HTTP_POST, handleTurretScanMinus);
  server.on("/scan_cancel", HTTP_POST, handleTurretScanCancel);
  server.on("/events", HTTP_GET, handleEvents);
  server.on("/alerts", HTTP_GET, handleAlerts);
  server.on("/odom", HTTP_GET, handleOdom);
  server.on("/set_pose", HTTP_POST, handleSetPose);
  server.on("/move", HTTP_POST, handleMoveCm);
  server.on("/turn", HTTP_POST, handleTurnDeg);
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

  // Deterministic IP reporting for Mega LCD:
  // Mega can miss the ESP's one-shot `ESPIP:` during boot, so Mega may request it.
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
  } else if (data.startsWith("RGBCLS:")) {
    // Expected: RGBCLS:<idx> where idx is 0..4 (0=NONE).
    const int v = data.substring(7).toInt();
    if (v >= 0 && v <= 4) rgbClassPacket = String("CLASS:") + String(v);
  } else if (data.startsWith("RGBREF:")) {
    // Expected: RGBREF:r,g,b;r,g,b;r,g,b;r,g,b
    rgbRefsPacket = String("REFS:") + data.substring(7);
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
    const char* line = data.c_str();
    const int len = data.length();
    if (len <= 6) return;

    const char* payload = line + 6;         // after "TSCAN:"
    const char* end = line + len;           // one past end

    // BEGIN/DONE/CANCEL are short and should not allocate.
    if ((end - payload) >= 6 && strncmp(payload, "BEGIN,", 6) == 0) {
      const char* p = payload + 6;
      const int dir = (memchr(p, '-', (size_t)(end - p)) != nullptr) ? -1 : 1;
      pushScanBegin_(dir);
      return;
    }
    if ((end - payload) >= 4 && strncmp(payload, "DONE", 4) == 0) {
      pushScanDone_();
      return;
    }
    if ((end - payload) >= 6 && strncmp(payload, "CANCEL", 6) == 0) {
      pushScanCancel_();
      return;
    }

    // Sample: "<angleDeg>,<distCm>"
    const void* cpos = memchr(payload, ',', (size_t)(end - payload));
    if (!cpos) return;
    const char* comma = (const char*)cpos;
    float a = NAN;
    float d = NAN;
    if (!parseDecFloatSpan_(payload, comma, a)) return;
    if (!parseDecFloatSpan_(comma + 1, end, d)) return;
    pushScanSample_(a, d);
  }

  // Push reflex / command events into the alerts ring.
  if (data.startsWith("EVT:")) pushAlertLine_(data);
}

void pollSerialNonBlocking() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (gSerialLineLen > 0) {
        gSerialLine[gSerialLineLen] = '\0';
        processSerialLine(String(gSerialLine));
        gSerialLineLen = 0;
      }
      continue;
    }

    if (gSerialLineLen >= kMaxSerialLineLen) {
      gSerialLineLen = 0;
      continue;
    }
    gSerialLine[gSerialLineLen++] = c;
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

void handleRgbClass() {
  if (!isArmed()) {
    server.send(200, "text/plain", "CLASS:0");
    return;
  }
  server.send(200, "text/plain", rgbClassPacket);
}

void handleRgbRefs() {
  if (!isArmed()) {
    server.send(200, "text/plain", "REFS:--");
    return;
  }
  server.send(200, "text/plain", rgbRefsPacket);
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

void handleTurretZero() {
  if (!isArmed()) return rejectNotArmed();
  Serial.println("TurretZero");
  server.send(200, "text/plain", "OK");
}

void handleTurretTpr() {
  if (!isArmed()) return rejectNotArmed();
  if (!server.hasArg("pos") || !server.hasArg("neg")) {
    server.send(400, "text/plain", "MISSING_POSNEG");
    return;
  }
  const uint32_t pos = (uint32_t)server.arg("pos").toInt();
  const uint32_t neg = (uint32_t)server.arg("neg").toInt();
  // Keep range aligned with Mega-side sanity checks.
  if (pos < 10u || pos > 200000u || neg < 10u || neg > 200000u) {
    server.send(400, "text/plain", "BAD_RANGE");
    return;
  }
  // Forward to Mega as a single line command.
  Serial.print("TurretTpr:");
  Serial.print((unsigned long)pos);
  Serial.print(",");
  Serial.println((unsigned long)neg);
  server.send(200, "text/plain", "OK");
}

void handleTurretScanPlus() {
  if (!isArmed()) return rejectNotArmed();
  (void)allocScanBufOnce_();
  resetScanBuffer_();
  pushScanBegin_(+1);  // synthetic BEGIN so browser sees it immediately
  Serial.println("TurretScanPlus");
  server.send(200, "text/plain", "OK");
}

void handleTurretScanMinus() {
  if (!isArmed()) return rejectNotArmed();
  (void)allocScanBufOnce_();
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

  // seq == index+1, so we can start directly from `from` without scanning old entries.
  // (This avoids long CPU loops that can starve WiFi / trigger WDT resets on ESP8266.)
  if (from >= (uint32_t)gScanCount) {
    server.send(200, "text/plain", "");
    return;
  }

  const uint16_t startIdx = (from > 0) ? (uint16_t)from : 0u; // from=N => start at seq N+1 => index N
  for (uint16_t i = startIdx; i < gScanCount; ++i) {
    const uint32_t seq = (uint32_t)i + 1u;

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

    // Keep ESP8266 WiFi/OS happy during large scans.
    if ((i & 0x3Fu) == 0) yield();
  }

  server.send(200, "text/plain", out);
}

void handleAlerts() {
  if (!isArmed()) return rejectNotArmed();
  uint32_t from = 0;
  if (server.hasArg("from")) from = (uint32_t)server.arg("from").toInt();

  String out;
  out.reserve(1400);

  if (gAlertCount == 0) {
    server.send(200, "text/plain", "");
    return;
  }

  const uint16_t oldestIdx = (uint16_t)((gAlertHead + kAlertRingSize - gAlertCount) % kAlertRingSize);
  const uint16_t newestIdx = (uint16_t)((gAlertHead + kAlertRingSize - 1) % kAlertRingSize);
  const uint32_t newestSeq = gAlerts[newestIdx].seq;

  if (from >= newestSeq) {
    server.send(200, "text/plain", "");
    return;
  }

  // Walk the ring in chronological order and emit entries with seq > from.
  // (Bounded to 256 entries, and we yield periodically.)
  uint16_t emitted = 0;
  for (uint16_t i = 0; i < gAlertCount; ++i) {
    const uint16_t idx = (uint16_t)((oldestIdx + i) % kAlertRingSize);
    const AlertEvt& e = gAlerts[idx];
    if (e.seq == 0) continue;
    if (e.seq <= from) continue;

    out += String(e.seq);
    out += "|";
    out += "EVT:";
    out += e.tag;
    out += ",";
    out += String(e.x_cm, 2);
    out += ",";
    out += String(e.y_cm, 2);
    out += ",";
    out += String(e.heading_deg, 1);
    if (e.hasExtra && isfinite(e.extra)) {
      out += ",";
      out += String(e.extra, 1);
    }
    out += "\n";

    emitted++;
    if (out.length() >= 1200) break;
    if ((emitted & 0x1Fu) == 0) yield();
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

void handleMoveCm() {
  if (!isArmed()) return rejectNotArmed();
  if (!server.hasArg("cm")) {
    server.send(400, "text/plain", "MISSING_CM");
    return;
  }
  const int cm = server.arg("cm").toInt();
  Serial.print("Move:");
  Serial.println(cm);
  server.send(200, "text/plain", "OK");
}

void handleTurnDeg() {
  if (!isArmed()) return rejectNotArmed();
  if (!server.hasArg("deg")) {
    server.send(400, "text/plain", "MISSING_DEG");
    return;
  }
  const int deg = server.arg("deg").toInt();
  Serial.print("Turn:");
  Serial.println(deg);
  server.send(200, "text/plain", "OK");
}

void handleSetPose() {
  // Browser provides initial map pose (x=east_cm, y=north_cm) after first scan match,
  // so the Mega's world odometry can be aligned to the map frame for later steps.
  if (!isArmed()) return rejectNotArmed();

  if (!server.hasArg("x") || !server.hasArg("y")) {
    server.send(400, "text/plain", "MISSING_XY");
    return;
  }

  const String xs = server.arg("x");
  const String ys = server.arg("y");
  const float x = xs.toFloat();
  const float y = ys.toFloat();
  if (!(isfinite(x) && isfinite(y))) {
    server.send(400, "text/plain", "BAD_XY");
    return;
  }

  // Optional matched heading (deg, 0=N, 90=E). If present, Mega can use it to
  // calibrate compass heading offset over time.
  bool hasH = false;
  float h = 0.0f;
  if (server.hasArg("h")) {
    const String hs = server.arg("h");
    h = hs.toFloat();
    if (isfinite(h)) hasH = true;
  }

  // Forward to Mega via UART in a simple line-based command.
  // Mega parser accepts:
  // - "MapPose:<east_cm>,<north_cm>"
  // - "MapPose:<east_cm>,<north_cm>,<heading_deg>"
  Serial.print("MapPose:");
  Serial.print(x, 2);
  Serial.print(",");
  Serial.print(y, 2);
  if (hasH) {
    Serial.print(",");
    Serial.print(h, 2);
  }
  Serial.println();

  server.send(200, "text/plain", "OK");
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
