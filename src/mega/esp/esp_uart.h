#pragma once

#include <Arduino.h>

struct EspCommand {
  enum class Type : uint8_t { None, Move, Turn, North, Maze } type;
  int value;
};

void espSetup();
bool espPoll(EspCommand& out);
