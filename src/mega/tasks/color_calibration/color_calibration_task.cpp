#include "tasks/color_calibration/color_calibration_task.h"

#include <string.h>
#include <EEPROM.h>

namespace {
const uint32_t kUiIntervalMs = 200;
const uint32_t kDoneHoldMs = 2000;
const uint16_t kEepromMagic = 0xC0A1;
const int kEepromAddr = 16;
}

ColorCalibrationTask::ColorCalibrationTask()
: state_(State::Idle),
  refs_{},
  hasCalibration_(false),
  lastUiMs_(0),
  doneStartMs_(0),
  ui_() {}

void ColorCalibrationTask::begin() {
  ui_.begin();
  ui_.showPrompt(1, nullptr, false);
  lastUiMs_ = millis();
  doneStartMs_ = 0;
  setState_(State::Prompt1);
}

void ColorCalibrationTask::update(const ColorRgb* live, bool liveValid) {
  if (state_ == State::Idle) return;

  const uint32_t now = millis();
  if (state_ == State::Done) {
    if (now - doneStartMs_ >= kDoneHoldMs) {
      ui_.showIdle();
      setState_(State::Idle);
    }
    return;
  }

  if (now - lastUiMs_ < kUiIntervalMs) return;
  lastUiMs_ = now;

  if (state_ == State::Prompt1) ui_.showPrompt(1, live, liveValid);
  if (state_ == State::Prompt2) ui_.showPrompt(2, live, liveValid);
  if (state_ == State::Prompt3) ui_.showPrompt(3, live, liveValid);
}

void ColorCalibrationTask::onButtonPress(const ColorRgb* live, bool liveValid) {
  if (state_ == State::Idle) {
    begin();
    return;
  }

  if (!liveValid || live == nullptr) {
    const uint8_t idx = (state_ == State::Prompt1) ? 1 : (state_ == State::Prompt2) ? 2 : 3;
    ui_.showSensorInvalid(idx);
    return;
  }

  if (state_ == State::Prompt1) {
    capture_(1, *live);
    ui_.showSaved(1, refs_[0]);
    setState_(State::Prompt2);
    return;
  }
  if (state_ == State::Prompt2) {
    capture_(2, *live);
    ui_.showSaved(2, refs_[1]);
    setState_(State::Prompt3);
    return;
  }
  if (state_ == State::Prompt3) {
    capture_(3, *live);
    ui_.showSaved(3, refs_[2]);
    saveToEeprom_();
    ui_.showDone();
    doneStartMs_ = millis();
    setState_(State::Done);
    return;
  }
}

bool ColorCalibrationTask::active() const { return state_ != State::Idle; }
ColorCalibrationTask::State ColorCalibrationTask::state() const { return state_; }
bool ColorCalibrationTask::hasCalibration() const { return hasCalibration_; }
const ColorRgb* ColorCalibrationTask::refs() const { return refs_; }

void ColorCalibrationTask::setState_(State s) { state_ = s; }

void ColorCalibrationTask::capture_(uint8_t index, const ColorRgb& rgb) {
  if (index < 1 || index > 3) return;
  refs_[index - 1] = rgb;
}

bool ColorCalibrationTask::saveToEeprom_() {
  EEPROM.put(kEepromAddr, kEepromMagic);
  EEPROM.put(kEepromAddr + sizeof(kEepromMagic), refs_);
  hasCalibration_ = true;
  return true;
}

bool ColorCalibrationTask::loadFromEeprom() {
  uint16_t magic = 0;
  EEPROM.get(kEepromAddr, magic);
  if (magic != kEepromMagic) {
    hasCalibration_ = false;
    return false;
  }
  EEPROM.get(kEepromAddr + sizeof(kEepromMagic), refs_);
  hasCalibration_ = true;
  return true;
}
