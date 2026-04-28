#include "protocol_helpers.h"

float wrapDeg360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float wrapDegDiff180(float targetDeg, float currentDeg) {
  float d = targetDeg - currentDeg;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

float clampf_(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool parseCommaFloats2(const String& s, float& aOut, float& bOut) {
  const int comma = s.indexOf(',');
  if (comma <= 0) return false;
  const float a = s.substring(0, comma).toFloat();
  const float b = s.substring(comma + 1).toFloat();
  if (!(isfinite(a) && isfinite(b))) return false;
  aOut = a;
  bOut = b;
  return true;
}

bool parseCommaFloats3Opt(const String& s, float& aOut, float& bOut, float& cOut, bool& hasCOut) {
  hasCOut = false;
  const int c1 = s.indexOf(',');
  if (c1 <= 0) return false;
  const int c2 = s.indexOf(',', c1 + 1);
  if (c2 < 0) {
    float a = 0.0f, b = 0.0f;
    if (!parseCommaFloats2(s, a, b)) return false;
    aOut = a;
    bOut = b;
    return true;
  }

  const float a = s.substring(0, c1).toFloat();
  const float b = s.substring(c1 + 1, c2).toFloat();
  const float c = s.substring(c2 + 1).toFloat();
  if (!(isfinite(a) && isfinite(b) && isfinite(c))) return false;
  aOut = a;
  bOut = b;
  cOut = c;
  hasCOut = true;
  return true;
}

