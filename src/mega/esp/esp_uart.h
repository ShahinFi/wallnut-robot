#pragma once

#include <Arduino.h>

struct EspCommand {
  enum class Type : uint8_t {
    None,
    Move,
    Turn,
    TurnShortest,
    TurnAbs,
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
  // WHY: Secondary integer payload used by commands that carry two numbers.
  int value2;
  String text;
};

void espSetup();
bool espPoll(EspCommand& out);
