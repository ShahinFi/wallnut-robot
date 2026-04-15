#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <string.h>

// ====== Wi-Fi credentials ======
const char* ssid = "Titenet-IoT";
const char* password = "7kDtaphg";

String lidarData = "0 cm";
String lidarPacket = "LIDAR:0,SEQ:0,T:0";
String compassData = "0,N";
String rgbPacket = "RGB:0,0,0";
String encCalPacket = "ENC_CAL:--";
String turretCalPacket = "TURCAL:--";

ESP8266WebServer server(80);
static String serialLineBuf;
static const size_t kMaxSerialLineLen = 128;

// ====== Option 1: Passcode arming (UART) ======
enum class AuthState : uint8_t { Disarmed, Armed, Locked };
static AuthState gAuthState = AuthState::Disarmed;
static uint8_t gTriesLeft = 3;
static bool gAuthPending = false;
static const uint8_t kMaxTries = 3;

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
void listAllFiles();
void processSerialLine(const String& data);
void pollSerialNonBlocking();
void serveFile(const char* path, const char* contentType);

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

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempt++;
    if (attempt > 60) {
      Serial.println("\n⚠️ WiFi connection failed. Restarting...");
      ESP.restart();
    }
  }
  Serial.println("\n✅ WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

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
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("✅ Web server started!");
}

void loop() {
  server.handleClient();
  pollSerialNonBlocking();
}

void processSerialLine(const String& data) {
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
  String message = "404: Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\n";
  server.send(404, "text/plain", message);
  Serial.println("404 for: " + server.uri());
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

  const uint32_t start = millis();
  while (millis() - start < 700) {
    pollSerialNonBlocking();
    if (!gAuthPending) break;
    delay(5);
  }

  if (gAuthPending) {
    // No reply from Mega yet.
    server.send(504, "text/plain", "NO_REPLY");
    return;
  }

  if (gAuthState == AuthState::Armed) {
    server.send(200, "text/plain", "OK");
    return;
  }
  if (gAuthState == AuthState::Locked) {
    server.send(423, "text/plain", "LOCKED");
    return;
  }

  server.send(401, "text/plain", String("FAIL:") + String(gTriesLeft));
}

void handleDisarm() {
  // Forward disarm to the Mega and wait briefly for an AUTH reply.
  // This avoids showing "DISARMED:3" when the Mega is actually LOCKED.
  gAuthPending = true;
  Serial.println("Disarm");

  const uint32_t start = millis();
  while (millis() - start < 700) {
    pollSerialNonBlocking();
    if (!gAuthPending) break;
    delay(5);
  }

  if (gAuthPending) {
    server.send(504, "text/plain", "NO_REPLY");
    return;
  }

  if (gAuthState == AuthState::Locked) {
    server.send(423, "text/plain", "LOCKED");
    return;
  }

  server.send(200, "text/plain", "OFF");
}

void handleAuth() {
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
  server.streamFile(file, contentType);
  file.close();
}
