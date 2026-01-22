#include "display_ui.h"

#include <Arduino.h>

#include "display/lcd.h"

DisplayUI::DisplayUI() : lastAverageMs(0), averageIntervalMs(500) {}

void DisplayUI::begin() {
  lcdInit();
  lcdWrite(0, 0, "Avg (cm):");
}

void DisplayUI::setAverageHz(uint8_t hz) {
  if (hz == 0) return;
  averageIntervalMs = 1000UL / hz;
}

void DisplayUI::update(const DisplayData &data) {
  const uint32_t now = millis();

  if (now - lastAverageMs >= averageIntervalMs) {
    lastAverageMs = now;
    const long avgRounded = lroundf(data.averageCm);
    lcdWriteInt(1, 0, avgRounded);
  }
}
