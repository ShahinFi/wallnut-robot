#include "joystick.h"

#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct VelocityRatioMap3 {
  float start_angle;
  float mid_angle;
  float end_angle;
  float start_ratio;
  float mid_ratio;
  float end_ratio;
};

static JoystickConfig gCfg;

static inline float linmap(float x, float x0, float x1, float y0, float y1) {
  if (x1 == x0) return 0.5f * (y0 + y1);
  float t = (x - x0) / (x1 - x0);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return y0 + t * (y1 - y0);
}

static float map3PointAngle(float angle, const VelocityRatioMap3 &m) {
  if (angle <= m.mid_angle)
    return linmap(angle, m.start_angle, m.mid_angle, m.start_ratio, m.mid_ratio);
  return linmap(angle, m.mid_angle, m.end_angle, m.mid_ratio, m.end_ratio);
}

static float joystickAngle(const JoystickData &d) {
  const float x = static_cast<float>(d.rawX) - 512.0f;
  const float y = static_cast<float>(d.rawY) - 512.0f;
  float angle = atan2f(y, x);
  if (angle < 0.0f) angle += 2.0f * static_cast<float>(M_PI);
  return angle;
}

static float joystickMagnitude(const JoystickData &d) {
  const float x = static_cast<float>(d.rawX) - 512.0f;
  const float y = static_cast<float>(d.rawY) - 512.0f;
  float mag = sqrtf(x * x + y * y) / 512.0f;
  if (mag > 1.0f) mag = 1.0f;
  return mag;
}

void joystickInit(const JoystickConfig &cfg) {
  gCfg = cfg;
}

JoystickData joystickRead() {
  JoystickData d = {};
  d.rawX = analogRead(gCfg.pinX);
  d.rawY = analogRead(gCfg.pinY);
  return d;
}

JoystickCommand joystickDrive() {
  JoystickCommand cmd = {0.0f, 0.0f};
  const JoystickData d = joystickRead();
  const float magnitude = joystickMagnitude(d);
  if (magnitude < gCfg.activeThreshold) return cmd;

  const float angle = joystickAngle(d);
  const float midAngle = static_cast<float>(M_PI) / 4.0f;
  VelocityRatioMap3 m = {};

  if (angle >= 0.0f && angle < static_cast<float>(M_PI) / 2.0f) {
    cmd.left = magnitude;
    m = {0.0f, midAngle, static_cast<float>(M_PI) / 2.0f, -1.0f, 0.0f, 1.0f};
    cmd.right = cmd.left * map3PointAngle(angle, m);
  } else if (angle >= static_cast<float>(M_PI) / 2.0f &&
             angle < static_cast<float>(M_PI)) {
    cmd.right = magnitude;
    m = {static_cast<float>(M_PI) / 2.0f,
         static_cast<float>(M_PI) - midAngle,
         static_cast<float>(M_PI), 1.0f, 0.0f, -1.0f};
    cmd.left = cmd.right * map3PointAngle(angle, m);
  } else if (angle >= static_cast<float>(M_PI) &&
             angle < 3.0f * static_cast<float>(M_PI) / 2.0f) {
    cmd.left = -magnitude;
    m = {static_cast<float>(M_PI),
         static_cast<float>(M_PI) + midAngle,
         3.0f * static_cast<float>(M_PI) / 2.0f, -1.0f, 0.0f, 1.0f};
    cmd.right = cmd.left * map3PointAngle(angle, m);
  } else {
    cmd.right = -magnitude;
    m = {3.0f * static_cast<float>(M_PI) / 2.0f,
         2.0f * static_cast<float>(M_PI) - midAngle,
         2.0f * static_cast<float>(M_PI), 1.0f, 0.0f, -1.0f};
    cmd.left = cmd.right * map3PointAngle(angle, m);
  }

  return cmd;
}
