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
    // WHY: Robust: parse digits without allocating substrings, keep only the last 4 digits.
    // WHY: This avoids fragile String heap behavior and tolerates occasional UART garbage.
    int acc = 0;
    int digits = 0;
    for (int i = 9; i < s.length(); i++) {
      const char c = s.charAt(i);
      if (c >= '0' && c <= '9') {
        digits++;
        acc = (acc * 10 + (c - '0')) % 10000;
      }
    }
    out.value = (digits > 0) ? acc : -1;
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.startsWith("ESPIP:")) {
    out.type = EspCommand::Type::EspIp;
    out.value = 0;
    out.value2 = 0;
    out.text = s.substring(6);
    out.text.trim();
    return true;
  }
  if (s.equalsIgnoreCase("Disarm")) {
    out.type = EspCommand::Type::Disarm;
    out.value = 0;
    out.value2 = 0;
    out.text = "";
    return true;
  }

  if (s.startsWith("Move:")) {
    out.type = EspCommand::Type::Move;
    out.value = s.substring(5).toInt();
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.startsWith("MapPose:")) {
    out.type = EspCommand::Type::MapPose;
    out.value = 0;
    out.value2 = 0;
    out.text = s.substring(8);
    out.text.trim();
    return true;
  }
  if (s.startsWith("Turn:")) {
    out.type = EspCommand::Type::Turn;
    out.value = s.substring(5).toInt();
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.startsWith("TurnShort:")) {
    out.type = EspCommand::Type::TurnShortest;
    out.value = s.substring(10).toInt();
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.startsWith("TurnAbs:")) {
    out.type = EspCommand::Type::TurnAbs;
    out.value = s.substring(8).toInt();
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("EncCal")) {
    out.type = EspCommand::Type::EncCal;
    out.value = 0;
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("TurretZero")) {
    out.type = EspCommand::Type::TurretZero;
    out.value = 0;
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.startsWith("TurretTpr:")) {
    out.type = EspCommand::Type::TurretTpr;
    // CONTRACT: Parses "TurretTpr:<pos>,<neg>" with digits-only extraction.
    // WHY: In-place parse avoids substring allocations and reduces heap churn on AVR.
    long pos = 0;
    long neg = 0;
    bool hasPos = false;
    bool hasNeg = false;
    bool parsingNeg = false;
    for (int i = 10; i < s.length(); ++i) {
      const char c = s.charAt(i);
      if (c == ',') {
        parsingNeg = true;
        continue;
      }
      if (c < '0' || c > '9') continue;
      if (!parsingNeg) {
        hasPos = true;
        pos = pos * 10 + (c - '0');
        if (pos > 2147483647L) pos = 2147483647L;
      } else {
        hasNeg = true;
        neg = neg * 10 + (c - '0');
        if (neg > 2147483647L) neg = 2147483647L;
      }
    }
    out.value = hasPos ? (int)pos : 0;
    out.value2 = hasNeg ? (int)neg : 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("TurretScanPlus")) {
    out.type = EspCommand::Type::TurretScanPlus;
    out.value = 0;
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("TurretScanMinus")) {
    out.type = EspCommand::Type::TurretScanMinus;
    out.value = 0;
    out.value2 = 0;
    out.text = "";
    return true;
  }
  if (s.equalsIgnoreCase("TurretScanCancel")) {
    out.type = EspCommand::Type::TurretScanCancel;
    out.value = 0;
    out.value2 = 0;
    out.text = "";
    return true;
  }
  return false;
}

bool espPoll(EspCommand& out) {
  out.type = EspCommand::Type::None;
  out.value = 0;
  out.value2 = 0;
  out.text = "";

  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n' || c == '\r') {
      if (espInput.length()) {
        const String line = espInput;
        espInput = "";
        return parseLine(line, out);
      }
    } else {
      if (espInput.length() >= kMaxEspLineLen) {
        // WHY: Drop overly long/noisy lines to avoid unbounded heap growth.
        espInput = "";
        continue;
      }
      espInput += c;
    }
  }
  return false;
}
