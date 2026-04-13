#include "lcd.h"

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <string.h>

// ---- LCD PINS / SIZE ----
static const int LCD_RS = 22;
static const int LCD_EN = 24;
static const int LCD_D4 = 26;
static const int LCD_D5 = 28;
static const int LCD_D6 = 30;
static const int LCD_D7 = 32;

static const uint8_t LCD_COLS = 20;
static const uint8_t LCD_ROWS = 4;

static LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Global LCD update throttle to reduce loop load.
static const uint32_t kLcdThrottleMs = 300;
static const uint32_t kLcdBurstWindowMs = 10;
static uint32_t gLastBurstMs = 0;
static uint32_t gBurstUntilMs = 0;

static bool lcdCanWrite() {
  const uint32_t now = millis();
  if (now <= gBurstUntilMs) return true;
  if (now - gLastBurstMs >= kLcdThrottleMs) {
    gLastBurstMs = now;
    gBurstUntilMs = now + kLcdBurstWindowMs;
    return true;
  }
  return false;
}

void lcdInit() {
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  const uint32_t now = millis();
  gLastBurstMs = now;
  gBurstUntilMs = now + kLcdBurstWindowMs;
}

void lcdClear() {
  if (!lcdCanWrite()) return;
  lcd.clear();
}

void lcdWrite(uint8_t row, uint8_t col, const char *text, bool clearToEOL) {
  if (row >= LCD_ROWS || col >= LCD_COLS || text == nullptr) return;
  if (!lcdCanWrite()) return;

  lcd.setCursor(col, row);
  lcd.print(text);

  if (clearToEOL) {
    size_t len = strlen(text);
    for (size_t i = len; i < (LCD_COLS - col); ++i) lcd.print(' ');
  }
}

void lcdWriteInt(uint8_t row, uint8_t col, long value, bool clearToEOL) {
  char buf[21];
  snprintf(buf, sizeof(buf), "%ld", value);
  lcdWrite(row, col, buf, clearToEOL);
}

void lcdWriteFloat(uint8_t row, uint8_t col, float value, uint8_t decimals,
                   bool clearToEOL) {
  char buf[21];
  dtostrf(value, 0, decimals, buf);
  lcdWrite(row, col, buf, clearToEOL);
}
