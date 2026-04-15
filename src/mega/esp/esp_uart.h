#pragma once

#include <Arduino.h>

struct EspCommand {
  enum class Type : uint8_t {
    None,
    Move,
    Turn,
    North,
    SetNorth,
    Maze,
    EncCal,
    TurretCalStart,
    TurretCalDone,
    TurretZero,
    Passcode,
    Disarm
  } type;
  int value;
  String text;
};

void espSetup();
bool espPoll(EspCommand& out);
