#pragma once

#include <Arduino.h>

float wrapDeg360(float deg);
float wrapDegDiff180(float targetDeg, float currentDeg);
float clampf_(float v, float lo, float hi);
bool parseCommaFloats2(const String& s, float& aOut, float& bOut);
bool parseCommaFloats3Opt(const String& s, float& aOut, float& bOut, float& cOut, bool& hasCOut);

