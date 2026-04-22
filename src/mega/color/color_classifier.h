#pragma once

#include <stdint.h>

#include "color/color_sensor.h"

namespace color {

struct ClassifyConfig {
  // Absolute closeness gate in normalized RGB space (Euclidean distance).
  // A sample must be within this distance of the nearest reference to classify.
  float absMaxDist = 0.18f;

  // Relative confidence gate: nearest must be at least this much closer than
  // runner-up. Expressed as a ratio on (linear) distance:
  //   bestDist / secondDist <= bestOverSecondMax
  float bestOverSecondMax = 0.80f;
};

struct ClassifyResult {
  int8_t idx = -1;        // 0..(refCount-1), or -1 for NONE/unknown
  float bestD2 = 0.0f;    // squared normalized distance to best
  float secondD2 = 0.0f;  // squared normalized distance to runner-up
};

// Classify a live RGB sample against prototype references.
// Returns idx in [0..refCount-1] only if:
// - nearest ref is absolutely close enough, AND
// - nearest ref is confidently closer than runner-up (ratio test).
// Otherwise returns idx = -1.
ClassifyResult classifyNearest(const ColorRgb& live, const ColorRgb* refs, uint8_t refCount,
                              const ClassifyConfig& cfg);

}  // namespace color

