#pragma once

#include <Arduino.h>
#include "color/color_sensor.h"

class ColorCalibrationUI {
public:
  void begin();

  void showIdle();
  void showPrompt(uint8_t index, const ColorRgb* live, bool liveValid);
  void showSaved(uint8_t index, const ColorRgb& saved);
  void showDone();
  void showSensorInvalid(uint8_t index);
};
