#pragma once

#include <stdint.h>

struct DisplayData {
  float averageCm;
};

enum class DisplayField : uint8_t {
  Average = 0,
  FieldCount
};

class DisplayUI {
public:
  DisplayUI();
  void begin();
  void setFieldHz(DisplayField field, uint8_t hz);
  void update(const DisplayData &data);

private:
};
