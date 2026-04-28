#pragma once

#include <stdint.h>

struct ColorRgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

class ColorSensor {
public:
  ColorSensor();

  bool begin();

  // CONTRACT: Output channels are normalized to [0,255]; returns false when normalization is invalid.
  bool read(ColorRgb& out);

private:
  bool normalizeRgb_(uint16_t r, uint16_t g, uint16_t b, uint16_t c, ColorRgb& out);
};
