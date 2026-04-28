#pragma once

#include <Arduino.h>
#include "color/color_sensor.h"
#include "tasks/color_calibration/ui/color_calibration_ui.h"

class ColorCalibrationTask {
public:
  static constexpr uint8_t kColorCount = 4;
  enum class State : uint8_t { Idle, Prompt1, Prompt2, Prompt3, Prompt4, Done };

  ColorCalibrationTask();

  void begin();
  // CONTRACT: Immediately exits calibration flow for serial control and safety paths.
  void cancel();
  void update(const ColorRgb* live, bool liveValid);
  void onButtonPress(const ColorRgb* live, bool liveValid);
  bool loadFromEeprom();
  bool hasCalibration() const;
  const ColorRgb* refs() const;
  // CONTRACT: Returns true exactly once after a successful EEPROM save.
  bool consumeJustSaved();

  bool active() const;
  State state() const;

private:
  void setState_(State s);
  void capture_(uint8_t index, const ColorRgb& rgb);
  bool saveToEeprom_();

  State state_;
  ColorRgb refs_[kColorCount];
  bool hasCalibration_;
  bool justSaved_;
  uint32_t doneStartMs_;
  ColorCalibrationUI ui_;
};
