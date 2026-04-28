#include "color/color_classifier.h"

#include <math.h>

namespace color {
namespace {
// SECTION: Normalized RGB distance helpers

static float distSqNormalizedRgb_(const ColorRgb& a, const ColorRgb& b) {
  float ar = (float)a.r, ag = (float)a.g, ab = (float)a.b;
  float br = (float)b.r, bg = (float)b.g, bb = (float)b.b;
  const float sa = ar + ag + ab;
  const float sb = br + bg + bb;
  if (!(sa > 0.0f) || !(sb > 0.0f)) return INFINITY;
  ar /= sa;
  ag /= sa;
  ab /= sa;
  br /= sb;
  bg /= sb;
  bb /= sb;
  const float dr = ar - br;
  const float dg = ag - bg;
  const float db = ab - bb;
  return dr * dr + dg * dg + db * db;
}

static float clampNonNeg_(float v) {
  if (!isfinite(v)) return 0.0f;
  if (v < 0.0f) return 0.0f;
  return v;
}

}

ClassifyResult classifyNearest(const ColorRgb& live, const ColorRgb* refs, uint8_t refCount,
                              const ClassifyConfig& cfg) {
  ClassifyResult out;
  out.idx = -1;
  out.bestD2 = INFINITY;
  out.secondD2 = INFINITY;

  if (!refs || refCount == 0) return out;

  // WHY: Keep both nearest and runner-up distances for ambiguity gating.
  float best = INFINITY;
  float second = INFINITY;
  int bestIdx = -1;

  for (uint8_t i = 0; i < refCount; i++) {
    const float d2 = distSqNormalizedRgb_(live, refs[i]);
    if (!isfinite(d2)) continue;
    if (d2 < best) {
      second = best;
      best = d2;
      bestIdx = (int)i;
    } else if (d2 < second) {
      second = d2;
    }
  }

  out.bestD2 = best;
  out.secondD2 = second;

  if (bestIdx < 0) return out;
  if (!(isfinite(best) && best >= 0.0f)) return out;
  // CONTRACT: Ratio confidence check requires a valid non-zero runner-up distance.
  if (!(isfinite(second) && second > 0.0f)) return out;

  const float absMaxDist = clampNonNeg_(cfg.absMaxDist);
  const float ratioMax = clampNonNeg_(cfg.bestOverSecondMax);
  const float absMaxD2 = absMaxDist * absMaxDist;
  const float ratioMaxSq = ratioMax * ratioMax;

  const bool absOk = best <= absMaxD2;
  const bool ratioOk = best <= second * ratioMaxSq;
  if (!(absOk && ratioOk)) return out;

  out.idx = (int8_t)bestIdx;
  return out;
}

}

