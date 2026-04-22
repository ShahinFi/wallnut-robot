#include "missions/maze/maze_lcd.h"

#include <Arduino.h>

#include "display/lcd.h"

namespace maze_lcd {
namespace {

static char gIp[16] = "0.0.0.0";

static uint32_t gLastUpdateMs = 0;
static const uint32_t kUpdatePeriodMs = 200;

enum class StopLatch : uint8_t { None = 0, Red = 1, Front = 2 };
static StopLatch gStopLatch = StopLatch::None;
static uint32_t gStopLatchUntilMs = 0;
static const uint32_t kStopLatchMs = 2500;

static void writeLine_(uint8_t row, const char* text) {
  // IMPORTANT:
  // `lcdWrite()` is globally throttled. It can drop writes if called outside the
  // allowed burst window. The maze dashboard must never "think it wrote" when
  // it didn't, otherwise lines can get stuck at the boot placeholder.
  //
  // So we do not cache "last line" here; we simply attempt the writes every
  // update tick and rely on the LCD throttle to limit load.
  lcdWrite(row, 0, text ? text : "", true);
}

static void clearLine_(char out[21]) {
  for (uint8_t i = 0; i < 20; ++i) out[i] = ' ';
  out[20] = '\0';
}

static uint8_t appendStr_(char out[21], uint8_t pos, const char* s) {
  if (!s) return pos;
  while (*s && pos < 20) out[pos++] = *s++;
  return pos;
}

static uint8_t appendChar_(char out[21], uint8_t pos, char c) {
  if (pos < 20) out[pos++] = c;
  return pos;
}

static uint8_t appendU3_(char out[21], uint8_t pos, uint16_t v) {
  if (v > 999) v = 999;
  const uint16_t a = (uint16_t)(v / 100);
  const uint16_t b = (uint16_t)((v / 10) % 10);
  const uint16_t c = (uint16_t)(v % 10);
  pos = appendChar_(out, pos, (char)('0' + a));
  pos = appendChar_(out, pos, (char)('0' + b));
  pos = appendChar_(out, pos, (char)('0' + c));
  return pos;
}

static uint8_t appendI3_(char out[21], uint8_t pos, int16_t v) {
  if (v < 0) {
    pos = appendChar_(out, pos, '-');
    const int16_t a = (v == INT16_MIN) ? (int16_t)32767 : (int16_t)(-v);
    return appendU3_(out, pos, (uint16_t)a);
  }
  return appendU3_(out, pos, (uint16_t)v);
}

static const char* authLabel_(AuthState a) {
  switch (a) {
    case AuthState::Disarmed: return "DIS";
    case AuthState::Pending: return "PEND";
    case AuthState::Armed: return "ARM";
    case AuthState::Locked: return "LOCK";
    default: return "DIS";
  }
}

static const char* autoLabel_(AutoState s) {
  switch (s) {
    case AutoState::Idle: return "IDLE";
    case AutoState::Scan: return "SCAN";
    case AutoState::Move: return "MOVE";
    case AutoState::Turn: return "TURN";
    case AutoState::Run: return "RUN";
    default: return "IDLE";
  }
}

static void formatIpLine_(char out[21]) {
  clearLine_(out);
  uint8_t pos = 0;
  pos = appendStr_(out, pos, "IP:");
  pos = appendStr_(out, pos, gIp);
  (void)pos;
}

}  // namespace

void init() {
  lcdInit();
  lcdWrite(0, 0, "Booting...", true);
  lcdWrite(1, 0, "Init...", true);
  lcdWrite(2, 0, "", true);
  lcdWrite(3, 0, "", true);
  gLastUpdateMs = 0;
  gStopLatch = StopLatch::None;
  gStopLatchUntilMs = 0;
}

void setIp(const char* ip) {
  if (!ip || !ip[0]) return;
  strncpy(gIp, ip, sizeof(gIp) - 1);
  gIp[sizeof(gIp) - 1] = '\0';
}

void notifyStopRed() {
  gStopLatch = StopLatch::Red;
  gStopLatchUntilMs = millis() + kStopLatchMs;
}

void notifyStopFront() {
  gStopLatch = StopLatch::Front;
  gStopLatchUntilMs = millis() + kStopLatchMs;
}

void update(AuthState auth, AutoState autoState, int16_t x_cm, int16_t y_cm, uint16_t headingDeg, uint16_t aheadCm,
            uint8_t colorClass) {
  const uint32_t now = millis();
  if (now - gLastUpdateMs < kUpdatePeriodMs) return;
  gLastUpdateMs = now;

  // Keep stack usage near-zero: reuse a single static scratch line.
  static char line[21] = {0};

  formatIpLine_(line);
  writeLine_(0, line);

  clearLine_(line);
  uint8_t pos = 0;
  pos = appendStr_(line, pos, "AUTH:");
  pos = appendStr_(line, pos, authLabel_(auth));
  pos = appendStr_(line, pos, "  AUTO:");
  pos = appendStr_(line, pos, autoLabel_(autoState));
  (void)pos;
  writeLine_(1, line);

  clearLine_(line);
  pos = 0;
  pos = appendStr_(line, pos, "X:");
  pos = appendI3_(line, pos, x_cm);
  pos = appendStr_(line, pos, " Y:");
  pos = appendI3_(line, pos, y_cm);
  pos = appendStr_(line, pos, " H:");
  pos = appendU3_(line, pos, (uint16_t)(headingDeg % 360u));
  (void)pos;
  writeLine_(2, line);

  clearLine_(line);
  if (gStopLatch != StopLatch::None && (int32_t)(now - gStopLatchUntilMs) < 0) {
    if (gStopLatch == StopLatch::Red) {
      appendStr_(line, 0, "STOP:RED MARKED");
    } else {
      appendStr_(line, 0, "STOP:FRONT <10cm");
    }
  } else {
    gStopLatch = StopLatch::None;
    gStopLatchUntilMs = 0;
    pos = 0;
    pos = appendStr_(line, pos, "AHEAD:");
    pos = appendU3_(line, pos, aheadCm);
    pos = appendStr_(line, pos, "cm CLR:");
    if (colorClass == 0) {
      pos = appendChar_(line, pos, 'N');
    } else {
      pos = appendChar_(line, pos, '#');
      // Expect 1..4; print a single digit, clamp for safety.
      uint8_t c = colorClass;
      if (c > 9) c = 9;
      pos = appendChar_(line, pos, (char)('0' + c));
    }
    (void)pos;
  }
  writeLine_(3, line);
}

void showFatal(const char* line2, const char* line3) {
  static char line[21] = {0};
  formatIpLine_(line);
  writeLine_(0, line);
  writeLine_(1, "FATAL");

  clearLine_(line);
  appendStr_(line, 0, line2 ? line2 : "");
  writeLine_(2, line);

  clearLine_(line);
  appendStr_(line, 0, line3 ? line3 : "");
  writeLine_(3, line);
}

}  // namespace maze_lcd
