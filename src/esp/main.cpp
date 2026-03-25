#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>

// ====== Wi-Fi credentials ======
const char* ssid = "Titenet-IoT";
const char* password = "7kDtaphg";

// ====== HTTP Basic Auth ======
const char* kAuthUser = "robot";
const char* kAuthPass = "control";
const char* kAuthRealm = "robot";
String lidarData = "0 cm";
String lidarPacket = "LIDAR:0,SEQ:0,T:0";
String compassData = "0,N";
String rgbPacket = "RGB:0,0,0";

ESP8266WebServer server(80);
static String serialLineBuf;
static const size_t kMaxSerialLineLen = 128;

// ====== Function Declarations ======
void handleNotFound();
void handleMove(int distance);
void handleNorth();
void handleCompass();
void handleLidar();
void handleCompassData();
void handleRgb();
void handleLogout();
void handleMaze();
void listAllFiles();
void processSerialLine(const String& data);
void pollSerialNonBlocking();
bool requireAuth();
void serveFileAuthed(const char* path, const char* contentType);

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

  server.on("/", []() { serveFileAuthed("/esp/index.html", "text/html"); });
  server.on("/index.html", []() { serveFileAuthed("/esp/index.html", "text/html"); });
  server.on("/style.css", []() { serveFileAuthed("/esp/style.css", "text/css"); });
  server.on("/script.js", []() { serveFileAuthed("/esp/script.js", "application/javascript"); });
  server.on("/favicon.ico", []() { serveFileAuthed("/esp/favicon.png", "image/png"); });

  server.on("/forwards5", []() { handleMove(5); });
  server.on("/forwards20", []() { handleMove(20); });
  server.on("/backwards5", []() { handleMove(-5); });
  server.on("/backwards20", []() { handleMove(-20); });
  server.on("/compass", handleCompass);
  server.on("/north", []() { handleNorth(); });
  server.on("/maze", handleMaze);
  server.on("/lidar", handleLidar);
  server.on("/compassdata", handleCompassData);
  server.on("/rgb", handleRgb);
  server.on("/logout", handleLogout);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("✅ Web server started!");
}

void loop() {
  server.handleClient();
  pollSerialNonBlocking();
}

void processSerialLine(const String& data) {
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
  if (!requireAuth()) return;
  String message = "404: Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\n";
  server.send(404, "text/plain", message);
  Serial.println("404 for: " + server.uri());
}

void handleMove(int distance) {
  if (!requireAuth()) return;
  Serial.println("Move: " + String(distance));
  server.send(200, "text/plain", "Move OK");
}

void handleCompass() {
  if (!requireAuth()) return;
  if (server.hasArg("value")) {
    String valueString = server.arg("value");
    Serial.println("Turn: " + valueString);
  } else {
    Serial.println("Turn: (no value)");
  }
  server.send(200, "text/plain", "Compass OK");
}

void handleNorth() {
  if (!requireAuth()) return;
  Serial.println("North");
  server.send(200, "text/plain", "North OK");
}

void handleLidar() {
  if (!requireAuth()) return;
  server.send(200, "text/plain", lidarPacket);
}

void handleCompassData() {
  if (!requireAuth()) return;
  server.send(200, "text/plain", compassData);
}

void handleRgb() {
  if (!requireAuth()) return;
  server.send(200, "text/plain", rgbPacket);
}

void handleMaze() {
  if (!requireAuth()) return;
  Serial.println("Maze");
  server.send(200, "text/plain", "Maze OK");
}

void handleLogout() {
  if (!requireAuth()) return;
  server.sendHeader("WWW-Authenticate", String("Basic realm=\"") + kAuthRealm + "\"");
  server.send(401, "text/plain", "Logged out");
}

bool requireAuth() {
  if (server.authenticate(kAuthUser, kAuthPass)) return true;
  server.requestAuthentication(BASIC_AUTH, kAuthRealm);
  return false;
}

void serveFileAuthed(const char* path, const char* contentType) {
  if (!requireAuth()) return;
  File file = LittleFS.open(path, "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, contentType);
  file.close();
}
