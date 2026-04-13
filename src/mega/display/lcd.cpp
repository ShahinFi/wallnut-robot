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

void lcdInit() {
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
}

void lcdClear() {
  lcd.clear();
}

void lcdWrite(uint8_t row, uint8_t col, const char *text, bool clearToEOL) {
  if (row >= LCD_ROWS || col >= LCD_COLS || text == nullptr) return;

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
