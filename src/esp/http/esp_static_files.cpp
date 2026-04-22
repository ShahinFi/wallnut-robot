#include "esp_static_files.h"
// (Refactor) HTTP module implementation.

#include <LittleFS.h>
#include <string.h>

namespace esp_static_files {

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

void serveFile(ESP8266WebServer& server, const char* path, const char* contentType) {
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

bool tryServeStaticUri(ESP8266WebServer& server, const String& uri) {
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

  serveFile(server, pathToServe.c_str(), ct);
  return true;
}

}  // namespace esp_static_files
