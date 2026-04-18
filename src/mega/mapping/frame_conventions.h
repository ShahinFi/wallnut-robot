#pragma once

#include <Arduino.h>
#include <math.h>

// ============================================================
// NOTE (Project Direction):
// Mapping / SLAM is now intended to run on the browser/PC side.
//
// The code under `src/mega/mapping/` and `src/mega/tasks/mapping/` is kept
// primarily for on-board debugging experiments (ASCII dumps, quick tests),
// not for the main maze-solving pipeline.
//
// The Mega's "production" responsibility is to:
// - execute turret sweeps and stream `TSCAN` samples,
// - provide odometry + heading telemetry,
// - execute motion actions safely (timeouts, stops, etc.).
//
// If you are building the real mapping logic: do it in the browser.
// ============================================================

// Mapping / scan matching frame conventions (single source of truth).
//
// Map/world frame (2D) for drawing:
// - X: +East  (to the right on screen)
// - Y: +North (up on screen)
//
// Heading convention (matches our compass/odometry docs):
// - headingDeg: 0 = North, 90 = East (clockwise increasing)
//
// Robot/body frame (2D):
// - x: +Forward
// - y: +Right
//
// Turret scan angles:
// - turretAngleDeg: 0 means turret aligned with robot +Forward (when zeroed)
// - positive angles rotate toward robot +Right (clockwise looking down)
namespace mapping {

struct Pose2D {
  float x_cm = 0.0f;       // map X (+East)
  float y_cm = 0.0f;       // map Y (+North)
  float headingDeg = 0.0f; // 0=N, 90=E, clockwise
};

struct LidarOffsetBodyCm {
  float x_cm = 0.0f;  // forward
  float y_cm = 0.0f;  // right
};

static inline float degToRad(float deg) { return deg * (3.14159265358979323846f / 180.0f); }

// Rotate a body-frame point (x forward, y right) into map frame (X east, Y north)
// using clockwise headingDeg where 0=N and 90=E.
static inline void bodyToMap(float headingDeg, float xb, float yb, float& xw, float& yw) {
  const float h = degToRad(headingDeg);
  const float c = cosf(h);
  const float s = sinf(h);
  // World basis in (North,East):
  // - forward -> (N=c, E=s)
  // - right   -> (N=-s, E=c)
  //
  // Convert to map drawing axes (X=East, Y=North):
  xw = s * xb + c * yb;   // East
  yw = c * xb - s * yb;   // North
}

// Polar (turret-to-body) endpoint in body frame.
static inline void polarToBody(float turretAngleDeg, float r_cm, float& xb, float& yb) {
  const float a = degToRad(turretAngleDeg);
  xb = r_cm * cosf(a);
  yb = r_cm * sinf(a);
}

}  // namespace mapping
