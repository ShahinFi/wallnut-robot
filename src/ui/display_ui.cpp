#include "display_ui.h"

#include <Arduino.h>

#include "display/lcd.h"

DisplayUI::DisplayUI() {
  timers[static_cast<uint8_t>(DisplayField::Average)] = {0, 500};
}

void DisplayUI::begin() {
  lcdInit();
  lcdWrite(0, 0, "Avg (cm):");
}

void DisplayUI::setFieldHz(DisplayField field, uint8_t hz) {
  if (hz == 0) return;
  const uint8_t idx = static_cast<uint8_t>(field);
  timers[idx].intervalMs = 1000UL / hz;
}

void DisplayUI::update(const DisplayData &data) {
  const uint32_t now = millis();

  FieldTimer &avgTimer = timers[static_cast<uint8_t>(DisplayField::Average)];
  if (now - avgTimer.lastMs >= avgTimer.intervalMs) {
    avgTimer.lastMs = now;
    const long avgRounded = lroundf(data.averageCm);
    lcdWriteInt(1, 0, avgRounded);
  }
}
