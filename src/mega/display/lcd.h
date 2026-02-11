#pragma once

#include <stdint.h>

void lcdInit();
void lcdClear();
void lcdWrite(uint8_t row, uint8_t col, const char *text, bool clearToEOL = true);
void lcdWriteInt(uint8_t row, uint8_t col, long value, bool clearToEOL = true);
void lcdWriteFloat(uint8_t row, uint8_t col, float value, uint8_t decimals = 2,
                   bool clearToEOL = true);
