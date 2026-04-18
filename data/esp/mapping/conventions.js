// Single source of truth for mapping conventions (matches Mega code).
//
// Map/world frame for drawing:
// - X: +East  (right on screen)
// - Y: +North (up on screen)
//
// Heading:
// - 0 deg = North
// - 90 deg = East
// - clockwise positive
//
// Body frame:
// - x: +Forward
// - y: +Right
//
// Turret scan:
// - angleDeg: 0 means forward
// - positive rotates toward body +Right

export function degToRad(deg) {
  return (deg * Math.PI) / 180.0;
}

export function wrap360(deg) {
  let d = deg;
  while (d < 0) d += 360;
  while (d >= 360) d -= 360;
  return d;
}

// Converts a body-frame point (xb forward, yb right) into map-frame delta (x East, y North)
// using headingDeg where 0=N, 90=E and clockwise increasing.
export function bodyToMap(headingDeg, xb, yb) {
  const h = degToRad(headingDeg);
  const c = Math.cos(h);
  const s = Math.sin(h);
  // Same as Mega's mapping/frame_conventions.h:
  // East  = s*xb + c*yb
  // North = c*xb - s*yb
  return { x: s * xb + c * yb, y: c * xb - s * yb };
}

export function polarToBody(angleDeg, r_cm) {
  const a = degToRad(angleDeg);
  return { xb: r_cm * Math.cos(a), yb: r_cm * Math.sin(a) };
}

