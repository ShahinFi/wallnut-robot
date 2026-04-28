#include "odometry/world_odometry.h"

#include <math.h>

namespace {
static inline float degToRad(float deg) { return deg * (3.14159265358979323846f / 180.0f); }

static float wrapDeg360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static float wrapDeg180(float deg) {
  while (deg <= -180.0f) deg += 360.0f;
  while (deg > 180.0f) deg -= 360.0f;
  return deg;
}

static bool finite2(float a, float b) { return isfinite(a) && isfinite(b); }
}

void worldOdomReset(WorldOdomState& s) {
  s.hasPrev = false;
  s.prevAvgCmSigned = 0.0f;
  s.prevHeadingDeg = 0.0f;
  s.pos.eastCm = 0.0f;
  s.pos.northCm = 0.0f;
}

void worldOdomRebase(WorldOdomState& s, float headingDeg, float avgCmSigned) {
  if (!finite2(headingDeg, avgCmSigned)) return;
  s.hasPrev = true;
  s.prevHeadingDeg = headingDeg;
  s.prevAvgCmSigned = avgCmSigned;
}

void worldOdomUpdate(WorldOdomState& s, float headingDeg, float avgCmSigned) {
  if (!finite2(headingDeg, avgCmSigned)) return;

  if (!s.hasPrev) {
    worldOdomRebase(s, headingDeg, avgCmSigned);
    return;
  }

  // WHY: Delta is signed forward motion since the previous integration sample.
  const float d = avgCmSigned - s.prevAvgCmSigned;

  // WHY: Midpoint heading using shortest-arc delta (works for wrapped or continuous headings).
  const float dh = wrapDeg180(headingDeg - s.prevHeadingDeg);
  const float hMid = s.prevHeadingDeg + 0.5f * dh;

  // WHY: Keep trig argument wrapped for numeric stability.
  const float rad = degToRad(wrapDeg360(hMid));

  // WHY: World projection (0=N, 90=E):
  s.pos.eastCm  += d * sinf(rad);
  s.pos.northCm += d * cosf(rad);

  s.prevHeadingDeg = headingDeg;
  s.prevAvgCmSigned = avgCmSigned;
}
