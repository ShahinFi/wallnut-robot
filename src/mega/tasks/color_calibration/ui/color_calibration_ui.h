#pragma once

#include <Arduino.h>
#include "color/color_sensor.h"

class ColorCalibrationUI {
public:
  void begin();

  void showIdle();
  void showPrompt(uint8_t index, uint8_t total, const ColorRgb* live, bool liveValid);
  void showSaved(uint8_t index, uint8_t total, const ColorRgb& saved);
  void showDone(uint8_t total);
  void showSensorInvalid(uint8_t index, uint8_t total);
  void showFailed(const char* reason);
};
