#include "tasks/color_calibration/ui/color_calibration_ui.h"

#include <stdio.h>
#include "display/lcd.h"

static void formatRgb(char* out, size_t outSize, const ColorRgb* rgb, bool valid) {
  if (!valid || rgb == nullptr) {
    snprintf(out, outSize, "R:--- G:--- B:---");
    return;
  }
  snprintf(out, outSize, "R:%3u G:%3u B:%3u", rgb->r, rgb->g, rgb->b);
}

void ColorCalibrationUI::begin() {
  lcdInit();
}

void ColorCalibrationUI::showIdle() {
  lcdClear();
  lcdWrite(0, 0, "Color Calib");
  lcdWrite(1, 0, "Press btn start");
  lcdWrite(2, 0, "ESP mode active");
}

void ColorCalibrationUI::showPrompt(uint8_t index, uint8_t total, const ColorRgb* live, bool liveValid) {
  char line2[21];
  char line3[21];
  snprintf(line2, sizeof(line2), "Show Color %u/%u", index, total);
  formatRgb(line3, sizeof(line3), live, liveValid);

  lcdWrite(0, 0, "Color Calib");
  lcdWrite(1, 0, line2);
  lcdWrite(2, 0, "Press to save");
  lcdWrite(3, 0, line3);
}

void ColorCalibrationUI::showSaved(uint8_t index, uint8_t total, const ColorRgb& saved) {
  char line2[21];
  char line3[21];
  snprintf(line2, sizeof(line2), "Saved Color %u/%u", index, total);
  formatRgb(line3, sizeof(line3), &saved, true);

  lcdWrite(0, 0, "Color Calib");
  lcdWrite(1, 0, line2);
  lcdWrite(2, 0, "Move to next");
  lcdWrite(3, 0, line3);
}

void ColorCalibrationUI::showDone(uint8_t total) {
  char line2[21];
  snprintf(line2, sizeof(line2), "Saved %u colors", total);
  lcdWrite(0, 0, "Color Calib");
  lcdWrite(1, 0, line2);
  lcdWrite(2, 0, "Returning...");
  lcdWrite(3, 0, "ESP mode active");
}

void ColorCalibrationUI::showSensorInvalid(uint8_t index, uint8_t total) {
  char line2[21];
  snprintf(line2, sizeof(line2), "Show Color %u/%u", index, total);
  lcdWrite(0, 0, "Color Calib");
  lcdWrite(1, 0, line2);
  lcdWrite(2, 0, "Sensor invalid");
  lcdWrite(3, 0, "Try again");
}
