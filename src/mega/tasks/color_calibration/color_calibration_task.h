#pragma once

#include <Arduino.h>
#include "color/color_sensor.h"
#include "tasks/color_calibration/ui/color_calibration_ui.h"

class ColorCalibrationTask {
public:
  enum class State : uint8_t { Idle, Prompt1, Prompt2, Prompt3, Done };

  ColorCalibrationTask();

  void begin();
  void update(const ColorRgb* live, bool liveValid);
  void onButtonPress(const ColorRgb* live, bool liveValid);
  bool loadFromEeprom();
  bool hasCalibration() const;
  const ColorRgb* refs() const;

  bool active() const;
  State state() const;

private:
  void setState_(State s);
  void capture_(uint8_t index, const ColorRgb& rgb);
  bool saveToEeprom_();

  State state_;
  ColorRgb refs_[3];
  bool hasCalibration_;
  uint32_t lastUiMs_;
  uint32_t doneStartMs_;
  ColorCalibrationUI ui_;
};
