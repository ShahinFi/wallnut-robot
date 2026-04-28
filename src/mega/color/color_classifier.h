#pragma once

#include <stdint.h>

#include "color/color_sensor.h"

namespace color {

struct ClassifyConfig {
  // WHY: Absolute gate in normalized RGB distance space.
  float absMaxDist = 0.18f;

  // WHY: Relative gate on nearest-vs-runner-up distance ratio.
  // CONTRACT: classify only when bestDist / secondDist <= bestOverSecondMax.
  float bestOverSecondMax = 0.80f;
};

struct ClassifyResult {
  int8_t idx = -1;
  float bestD2 = 0.0f;
  float secondD2 = 0.0f;
};

// CONTRACT: Returns idx in [0..refCount-1] only when absolute and relative gates pass.
// WHY: On failure or ambiguity, idx remains -1.
ClassifyResult classifyNearest(const ColorRgb& live, const ColorRgb* refs, uint8_t refCount,
                              const ClassifyConfig& cfg);

}

