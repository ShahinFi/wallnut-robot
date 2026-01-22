#pragma once

#include <stdint.h>

struct DisplayData {
  float averageCm;
};

class DisplayUI {
public:
  DisplayUI();
  void begin();
  void setAverageHz(uint8_t hz);
  void update(const DisplayData &data);

private:
  uint32_t lastAverageMs;
  uint32_t averageIntervalMs;
};
