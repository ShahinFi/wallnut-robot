#include "color/color_sensor.h"

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

namespace {
// WHY: Fixed defaults keep sensor behavior stable across boots.
const auto kIntegrationTime = TCS34725_INTEGRATIONTIME_50MS;
const auto kGain = TCS34725_GAIN_4X;
}

static Adafruit_TCS34725 gTcs(kIntegrationTime, kGain);

ColorSensor::ColorSensor() {}

bool ColorSensor::begin() {
  return gTcs.begin();
}

bool ColorSensor::read(ColorRgb& out) {
  uint16_t r = 0, g = 0, b = 0, c = 0;
  gTcs.getRawData(&r, &g, &b, &c);
  return normalizeRgb_(r, g, b, c, out);
}

bool ColorSensor::normalizeRgb_(uint16_t r, uint16_t g, uint16_t b, uint16_t c,
                                ColorRgb& out) {
  if (c == 0) {
    out = {0, 0, 0};
    return false;
  }

  uint32_t rN = (static_cast<uint32_t>(r) * 255U) / c;
  uint32_t gN = (static_cast<uint32_t>(g) * 255U) / c;
  uint32_t bN = (static_cast<uint32_t>(b) * 255U) / c;

  if (rN > 255U) rN = 255U;
  if (gN > 255U) gN = 255U;
  if (bN > 255U) bN = 255U;

  out = {static_cast<uint8_t>(rN),
         static_cast<uint8_t>(gN),
         static_cast<uint8_t>(bN)};
  return true;
}
