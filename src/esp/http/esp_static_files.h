#pragma once
// (Refactor) HTTP module: static file serving.

#include <Arduino.h>
#include <ESP8266WebServer.h>

namespace esp_static_files {

const char* contentTypeForPath(const String& path);
void serveFile(ESP8266WebServer& server, const char* path, const char* contentType);
bool tryServeStaticUri(ESP8266WebServer& server, const String& uri);

}  // namespace esp_static_files
