#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>

// ====== Wi-Fi credentials ======
const char* ssid = "Titenet-IoT";
const char* password = "7kDtaphg";
String lidarData = "0 cm";
String lidarPacket = "LIDAR:0,SEQ:0,T:0";
String compassData = "0,N";

ESP8266WebServer server(80);

// ====== Function Declarations ======
void handleNotFound();
void handleMove(int distance);
void handleNorth();
void handleCompass();
void handleLidar();
void handleCompassData();
void listAllFiles();

void setup() {
  Serial.begin(9600);
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

  server.on("/", []() {
    File file = LittleFS.open("/esp/index.html", "r");
    if (!file) {
      server.send(404, "text/plain", "index.html not found");
      return;
    }
    server.streamFile(file, "text/html");
    file.close();
  });

  server.serveStatic("/index.html", LittleFS, "/esp/index.html");
  server.serveStatic("/style.css", LittleFS, "/esp/style.css");
  server.serveStatic("/script.js", LittleFS, "/esp/script.js");
  server.serveStatic("/favicon.ico", LittleFS, "/esp/favicon.png");

  server.on("/forwards5", []() { handleMove(5); });
  server.on("/forwards20", []() { handleMove(20); });
  server.on("/backwards5", []() { handleMove(-5); });
  server.on("/backwards20", []() { handleMove(-20); });
  server.on("/compass", handleCompass);
  server.on("/north", []() { handleNorth(); });
  server.on("/lidar", handleLidar);
  server.on("/compassdata", handleCompassData);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("✅ Web server started!");
}

void loop() {
  server.handleClient();

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.startsWith("LIDAR:")) {
      lidarPacket = data;
      String payload = lidarPacket;
      int comma = payload.indexOf(',');
      if (comma >= 0) payload = payload.substring(0, comma);
      lidarData = payload;
    } else if (data.startsWith("COMPASS:")) {
      compassData = data.substring(8);
    }
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
  Serial.println("Move: " + String(distance));
  server.send(200, "text/plain", "Move OK");
}

void handleCompass() {
  if (server.hasArg("value")) {
    String valueString = server.arg("value");
    Serial.println("Turn: " + valueString);
  } else {
    Serial.println("Turn: (no value)");
  }
  server.send(200, "text/plain", "Compass OK");
}

void handleNorth() {
  Serial.println("North");
  server.send(200, "text/plain", "North OK");
}

void handleLidar() {
  server.send(200, "text/plain", lidarPacket);
}

void handleCompassData() {
  server.send(200, "text/plain", compassData);
}
