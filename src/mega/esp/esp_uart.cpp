#include "esp/esp_uart.h"

static String espInput;

void espSetup() {
  Serial2.begin(115200);
}

static bool parseLine(const String& line, EspCommand& out) {
  String s = line;
  s.trim();
  if (s.length() == 0) return false;

  if (s.startsWith("Move:")) {
    out.type = EspCommand::Type::Move;
    out.value = s.substring(5).toInt();
    return true;
  }
  if (s.startsWith("Turn:")) {
    out.type = EspCommand::Type::Turn;
    out.value = s.substring(5).toInt();
    return true;
  }
  if (s.equalsIgnoreCase("North")) {
    out.type = EspCommand::Type::North;
    out.value = 0;
    return true;
  }
  if (s.equalsIgnoreCase("Maze")) {
    out.type = EspCommand::Type::Maze;
    out.value = 0;
    return true;
  }
  return false;
}

bool espPoll(EspCommand& out) {
  out.type = EspCommand::Type::None;
  out.value = 0;

  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
    if (c == '\n' || c == '\r') {
      if (espInput.length()) {
        const String line = espInput;
        espInput = "";
        return parseLine(line, out);
      }
    } else {
      espInput += c;
    }
  }
  return false;
}
