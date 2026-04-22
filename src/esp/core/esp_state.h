#pragma once
// (Refactor) Core module: cached state.

#include <Arduino.h>

// Cached ESP-side state shared across handlers and UART parsing.
struct EspState {
  // Cached telemetry packets (as sent over HTTP).
  String lidarData = "0 cm";
  String lidarPacket = "LIDAR:0,SEQ:0,T:0";
  String compassData = "0,N";
  String rgbPacket = "RGB:0,0,0";
  String rgbClassPacket = "CLASS:0";  // 0=NONE, 1..4 = stored color index
  String rgbRefsPacket = "REFS:--";   // REFS:r,g,b;... (4 refs) or "--"
  String encCalPacket = "ENC_CAL:--";
  String turretCalPacket = "TURCAL:--";
  String odomPacket = "ODOM:0.0,0.0";

  // Passcode arming.
  enum class AuthState : uint8_t { Disarmed, Armed, Locked };
  AuthState authState = AuthState::Disarmed;
  uint8_t triesLeft = 3;
  bool authPending = false;

  // UART link health.
  uint32_t lastMegaRxMs = 0;
  bool hasMegaRx = false;

  bool isArmed() const { return authState == AuthState::Armed; }
  void noteMegaRx() {
    lastMegaRxMs = millis();
    hasMegaRx = true;
  }
};
