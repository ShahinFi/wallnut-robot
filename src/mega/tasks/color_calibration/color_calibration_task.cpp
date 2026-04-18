#include "tasks/color_calibration/color_calibration_task.h"

#include <string.h>
#include <EEPROM.h>

namespace {
const uint32_t kDoneHoldMs = 2000;
// NOTE: bumping magic because calibration payload size changed (3 -> 4 colors).
// Old EEPROM data will be ignored and needs recalibration.
const uint16_t kEepromMagic = 0xC0A2;
const int kEepromAddr = 16;
}

ColorCalibrationTask::ColorCalibrationTask()
: state_(State::Idle),
  refs_{},
  hasCalibration_(false),
  doneStartMs_(0),
  ui_() {}

void ColorCalibrationTask::begin() {
  ui_.begin();
  ui_.showPrompt(1, kColorCount, nullptr, false);
  doneStartMs_ = 0;
  setState_(State::Prompt1);
}

void ColorCalibrationTask::cancel() {
  if (state_ == State::Idle) return;
  ui_.showIdle();
  setState_(State::Idle);
  doneStartMs_ = 0;
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

  if (state_ == State::Prompt1) ui_.showPrompt(1, kColorCount, live, liveValid);
  if (state_ == State::Prompt2) ui_.showPrompt(2, kColorCount, live, liveValid);
  if (state_ == State::Prompt3) ui_.showPrompt(3, kColorCount, live, liveValid);
  if (state_ == State::Prompt4) ui_.showPrompt(4, kColorCount, live, liveValid);
}

void ColorCalibrationTask::onButtonPress(const ColorRgb* live, bool liveValid) {
  if (state_ == State::Idle) {
    begin();
    return;
  }

  if (!liveValid || live == nullptr) {
    uint8_t idx = 1;
    if (state_ == State::Prompt2) idx = 2;
    else if (state_ == State::Prompt3) idx = 3;
    else if (state_ == State::Prompt4) idx = 4;
    ui_.showSensorInvalid(idx, kColorCount);
    return;
  }

  if (state_ == State::Prompt1) {
    capture_(1, *live);
    ui_.showSaved(1, kColorCount, refs_[0]);
    setState_(State::Prompt2);
    return;
  }
  if (state_ == State::Prompt2) {
    capture_(2, *live);
    ui_.showSaved(2, kColorCount, refs_[1]);
    setState_(State::Prompt3);
    return;
  }
  if (state_ == State::Prompt3) {
    capture_(3, *live);
    ui_.showSaved(3, kColorCount, refs_[2]);
    setState_(State::Prompt4);
    return;
  }
  if (state_ == State::Prompt4) {
    capture_(4, *live);
    saveToEeprom_();
    ui_.showDone(kColorCount);
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
  if (index < 1 || index > kColorCount) return;
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
