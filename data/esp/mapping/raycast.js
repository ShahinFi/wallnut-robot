// Robust grid ray traversal in a uniform 2D grid.
//
// We use a standard Amanatides & Woo voxel traversal variant. This is the
// "single critical primitive" for FREE-space marking: enumerate exactly which
// grid cells a line segment passes through (no ad-hoc rounding).

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function inBoundsCell(grid, cx, cy) {
  return cx >= 0 && cy >= 0 && cx < grid.w && cy < grid.h;
}

// Iterate all grid cells intersected by the segment from (x0,y0) to (x1,y1),
// in *world cm* coordinates. Stops if it leaves the map bounds.
//
// Calls `fn(cx, cy)` for each visited cell, including the start cell, and
// including the end cell if it is inside the map.
//
// If `fn` returns `false`, iteration stops early.
export function forEachCellOnSegmentWorld(grid, x0_cm, y0_cm, x1_cm, y1_cm, fn) {
  const cell = grid.cell_cm;
  if (!Number.isFinite(x0_cm) || !Number.isFinite(y0_cm) || !Number.isFinite(x1_cm) || !Number.isFinite(y1_cm)) return;
  if (cell <= 0) return;

  // Convert to grid-space coordinates (cell units).
  let x0 = x0_cm / cell;
  let y0 = y0_cm / cell;
  const x1 = x1_cm / cell;
  const y1 = y1_cm / cell;

  let cx = Math.floor(x0);
  let cy = Math.floor(y0);
  const cxEnd = Math.floor(x1);
  const cyEnd = Math.floor(y1);

  if (!inBoundsCell(grid, cx, cy)) return;

  const dx = x1 - x0;
  const dy = y1 - y0;

  // Degenerate segment: single cell.
  if (dx === 0 && dy === 0) {
    fn(cx, cy);
    return;
  }

  const stepX = dx > 0 ? 1 : dx < 0 ? -1 : 0;
  const stepY = dy > 0 ? 1 : dy < 0 ? -1 : 0;

  const invAbsDx = stepX !== 0 ? 1 / Math.abs(dx) : Infinity;
  const invAbsDy = stepY !== 0 ? 1 / Math.abs(dy) : Infinity;
  const tDeltaX = invAbsDx;
  const tDeltaY = invAbsDy;

  // How far along the ray (in [0..1] of the segment) to the first boundary.
  let tMaxX = Infinity;
  let tMaxY = Infinity;
  if (stepX > 0) tMaxX = ((cx + 1) - x0) / dx;
  else if (stepX < 0) tMaxX = (x0 - cx) / (-dx);
  if (stepY > 0) tMaxY = ((cy + 1) - y0) / dy;
  else if (stepY < 0) tMaxY = (y0 - cy) / (-dy);

  // Guard against tiny numeric negatives.
  tMaxX = Math.max(0, tMaxX);
  tMaxY = Math.max(0, tMaxY);

  // Traverse.
  // We cap steps to avoid infinite loops if something goes wrong.
  const maxSteps = grid.w * grid.h + 8;
  for (let steps = 0; steps < maxSteps; steps++) {
    if (fn(cx, cy) === false) return;
    if (cx === cxEnd && cy === cyEnd) return;

    if (tMaxX < tMaxY) {
      cx += stepX;
      tMaxX += tDeltaX;
    } else {
      cy += stepY;
      tMaxY += tDeltaY;
    }

    if (!inBoundsCell(grid, cx, cy)) return;
  }
}

// Clip an endpoint to the map rectangle if it goes outside.
// Returns { x, y, clipped } where clipped=true means the original endpoint
// was out of bounds and we moved it to the last in-bounds point.
export function clipEndpointToMap(grid, x0_cm, y0_cm, x1_cm, y1_cm) {
  const xMin = 0;
  const yMin = 0;
  const xMax = grid.mapW_cm;
  const yMax = grid.mapH_cm;

  // Match HitGrid.worldToCell() bounds: [0, max) not [0, max].
  const in0 = x0_cm >= xMin && x0_cm < xMax && y0_cm >= yMin && y0_cm < yMax;
  if (!in0) return { x: x0_cm, y: y0_cm, clipped: true };

  const in1 = x1_cm >= xMin && x1_cm < xMax && y1_cm >= yMin && y1_cm < yMax;
  if (in1) return { x: x1_cm, y: y1_cm, clipped: false };

  const dx = x1_cm - x0_cm;
  const dy = y1_cm - y0_cm;
  let tExit = 1;

  // For each boundary, compute t where the segment intersects it.
  if (dx > 0) tExit = Math.min(tExit, (xMax - x0_cm) / dx);
  else if (dx < 0) tExit = Math.min(tExit, (xMin - x0_cm) / dx);
  if (dy > 0) tExit = Math.min(tExit, (yMax - y0_cm) / dy);
  else if (dy < 0) tExit = Math.min(tExit, (yMin - y0_cm) / dy);

  tExit = clamp(tExit, 0, 1);
  // Step slightly back into the map to ensure worldToCell() succeeds.
  const eps = Math.min(0.49 * grid.cell_cm, 0.5);
  const tBack = (Math.hypot(dx, dy) > 1e-9) ? (eps / Math.hypot(dx, dy)) : 0;
  const t = clamp(tExit - tBack, 0, 1);
  return { x: x0_cm + dx * t, y: y0_cm + dy * t, clipped: true };
}
