// SECTION: Mapping conventions (matches Mega code).
// WHY: Map frame is +X east, +Y north, with heading 0=N and clockwise positive.
// WHY: Body frame is +x forward, +y right; turret scan angle is body-relative.

export function degToRad(deg) {
  return (deg * Math.PI) / 180.0;
}

export function wrap360(deg) {
  let d = deg;
  while (d < 0) d += 360;
  while (d >= 360) d -= 360;
  return d;
}

// WHY: Convert body-frame delta (forward/right) into map-frame delta (east/north).
export function bodyToMap(headingDeg, xb, yb) {
  const h = degToRad(headingDeg);
  const c = Math.cos(h);
  const s = Math.sin(h);
  // CONTRACT: Transform must match Mega `mapping/frame_conventions.h`.
  return { x: s * xb + c * yb, y: c * xb - s * yb };
}

export function polarToBody(angleDeg, r_cm) {
  const a = degToRad(angleDeg);
  return { xb: r_cm * Math.cos(a), yb: r_cm * Math.sin(a) };
}


