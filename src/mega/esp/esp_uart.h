#pragma once

#include <Arduino.h>

struct EspCommand {
  enum class Type : uint8_t {
    None,
    Move,
    Turn,
    TurnShortest,
    TurnAbs,
    North,
    SetNorth,
    EncCal,
    MapPose,
    TurretZero,
    TurretTpr,
    TurretScanPlus,
    TurretScanMinus,
    TurretScanCancel,
    EspIp,
    Passcode,
    Disarm
  } type;
  int value;
  int value2;  // optional second integer payload (used by TurretTpr)
  String text;
};

void espSetup();
bool espPoll(EspCommand& out);
