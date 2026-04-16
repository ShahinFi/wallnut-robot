#include "esp/esp_uart.h"

static String espInput;
static const size_t kMaxEspLineLen = 128;

void espSetup() {
  Serial2.begin(115200);
}

static bool parseLine(const String& line, EspCommand& out) {
  String s = line;
  s.trim();
  if (s.length() == 0) return false;

  if (s.startsWith("Passcode:")) {
    out.type = EspCommand::Type::Passcode;
    out.value = 0;
    out.text = s.substring(9);
    out.text.trim();
    return true;
  }
  if (s.equalsIgnoreCase("Disarm")) {
    out.type = EspCommand::Type::Disarm;
    out.value = 0;
    out.text = "";
    return true;
  }

  if (s.startsWith("Move:")) {
    out.type = EspCommand::Type::Move;
    out.value = s.substring(5).toInt();
    out.text = "";
    return true;
  }
  if (s.startsWith("Turn:")) {
    out.type = EspCommand::Type::Turn;
    out.value = s.substring(5).toInt();
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("North")) {
    out.type = EspCommand::Type::North;
    out.value = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("SetNorth")) {
    out.type = EspCommand::Type::SetNorth;
    out.value = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("Maze")) {
    out.type = EspCommand::Type::Maze;
    out.value = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("EncCal")) {
    out.type = EspCommand::Type::EncCal;
    out.value = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("TurretCalStart")) {
    out.type = EspCommand::Type::TurretCalStart;
    out.value = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("TurretCalDone")) {
    out.type = EspCommand::Type::TurretCalDone;
    out.value = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("TurretZero")) {
    out.type = EspCommand::Type::TurretZero;
    out.value = 0;
    out.text = "";
    return true;
  }
  return false;
}

bool espPoll(EspCommand& out) {
  out.type = EspCommand::Type::None;
  out.value = 0;
  out.text = "";

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
      if (espInput.length() >= kMaxEspLineLen) {
        // Drop overly long/noisy lines to avoid unbounded heap growth.
        espInput = "";
        continue;
      }
      espInput += c;
    }
  }
  return false;
}
