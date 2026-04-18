#pragma once

#include <Arduino.h>

namespace mapping {

// ============================================================
// NOTE (Project Direction):
// This scan buffer exists for on-board debug experiments only.
// The primary pipeline streams `TSCAN` to the browser/PC where mapping/SLAM
// is performed in JS.
// ============================================================

// Thin fixed-capacity scan buffer for one sweep.
// Stores scan endpoints in BODY cartesian coords (x forward, y right).
class ScanBuffer {
public:
  // NOTE: Keep this small to avoid SRAM exhaustion on Mega.
  // With our typical scan sampling (~2 deg), 128 points is usually enough.
  // Increase only if you have confirmed sufficient free SRAM at runtime.
  static const uint16_t kMaxPoints = 128;

  // Compact representation to save SRAM on Mega:
  // - x,y in mm in body frame (0,0 means invalid / missing sample)
  struct Point {
    int16_t xb_mm = 0;  // +forward
    int16_t yb_mm = 0;  // +right

    float xb_cm() const { return (float)xb_mm * 0.1f; }
    float yb_cm() const { return (float)yb_mm * 0.1f; }
  };

  ScanBuffer();

  void reset();
  bool push(float angleDeg, float distCm);

  uint16_t size() const;
  const Point& at(uint16_t i) const;

private:
  uint16_t n_;
  Point pts_[kMaxPoints];
};

}  // namespace mapping
