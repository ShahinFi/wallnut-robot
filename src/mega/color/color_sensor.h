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

  // Reads and returns normalized RGB (0..255). Returns false on failure.
  bool read(ColorRgb& out);

private:
  bool normalizeRgb_(uint16_t r, uint16_t g, uint16_t b, uint16_t c, ColorRgb& out);
};
