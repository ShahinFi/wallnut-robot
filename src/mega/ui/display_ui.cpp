#include "display_ui.h"

#include <Arduino.h>

#include "display/lcd.h"

DisplayUI::DisplayUI() {
}

void DisplayUI::begin() {
  lcdInit();
  lcdWrite(0, 0, "Avg (cm):");
}

void DisplayUI::setFieldHz(DisplayField field, uint8_t hz) {
  (void)field;
  (void)hz;
}

void DisplayUI::update(const DisplayData &data) {
  const long avgRounded = lroundf(data.averageCm);
  lcdWriteInt(1, 0, avgRounded);
}
