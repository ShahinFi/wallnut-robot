#include "esp_static_files.h"
// SECTION: Static file serving helpers.

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
  // WHY: Keep backward compatibility with both filesystem layouts.
  if (!file && path && strncmp(path, "/esp/", 5) == 0) {
    // WHY: Strip "/esp" to support legacy root-level uploaded files.
    file = LittleFS.open(path + 4, "r");
  }
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  // CONTRACT: Disable cache to avoid stale UI assets across clients.
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(file, contentType);
  file.close();
}

bool tryServeStaticUri(ESP8266WebServer& server, const String& uri) {
  if (uri.length() == 0) return false;
  if (uri == "/") return false;
  if (uri == "/events") return false;
  if (uri.indexOf("..") >= 0) return false;

  // WHY: Accept both root and legacy /esp-prefixed asset paths.
  const char* ct = contentTypeForPath(uri);
  String pathToServe = uri;
  File f = LittleFS.open(pathToServe, "r");
  if (!f && uri.startsWith("/")) {
    // WHY: Legacy fallback for older uploaded filesystems.
    pathToServe = String("/esp") + uri;
    f = LittleFS.open(pathToServe, "r");
  }
  if (!f) return false;
  f.close();

  serveFile(server, pathToServe.c_str(), ct);
  return true;
}

}
