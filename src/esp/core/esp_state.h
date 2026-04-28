#pragma once
// SECTION: Shared ESP runtime state.

#include <Arduino.h>

// CONTRACT: Shared mutable state for UART parser and HTTP handlers.
struct EspState {
  // SECTION: Cached telemetry packets.
  String lidarData = "0 cm";
  String lidarPacket = "LIDAR:0,SEQ:0,T:0";
  String compassData = "0,N";
  String rgbPacket = "RGB:0,0,0";
  // CONTRACT: `rgbClassPacket` is `CLASS:0` for none, otherwise `CLASS:1..4`.
  String rgbClassPacket = "CLASS:0";
  // CONTRACT: `rgbRefsPacket` is `REFS:r,g,b;...` or `REFS:--` when unavailable.
  String rgbRefsPacket = "REFS:--";
  String encCalPacket = "ENC_CAL:--";
  String turretCalPacket = "TURCAL:--";
  String odomPacket = "ODOM:0.0,0.0";

  // SECTION: Passcode arming state.
  enum class AuthState : uint8_t { Disarmed, Armed, Locked };
  AuthState authState = AuthState::Disarmed;
  uint8_t triesLeft = 3;
  bool authPending = false;

  // SECTION: UART link health.
  uint32_t lastMegaRxMs = 0;
  bool hasMegaRx = false;

  bool isArmed() const { return authState == AuthState::Armed; }
  void noteMegaRx() {
    lastMegaRxMs = millis();
    hasMegaRx = true;
  }
};
