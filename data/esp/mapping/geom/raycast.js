// SECTION: Robust grid ray traversal in a uniform 2D grid.
// WHY: Uses Amanatides & Woo traversal to avoid ad-hoc rounding artifacts.
// CONTRACT: This primitive defines FREE-space ray evidence cell-by-cell.

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function inBoundsCell(grid, cx, cy) {
  return cx >= 0 && cy >= 0 && cx < grid.w && cy < grid.h;
}

// WHY: Iterate segment cells in world-cm coordinates until out-of-bounds or callback stop.
export function forEachCellOnSegmentWorld(grid, x0_cm, y0_cm, x1_cm, y1_cm, fn) {
  const cell = grid.cell_cm;
  if (!Number.isFinite(x0_cm) || !Number.isFinite(y0_cm) || !Number.isFinite(x1_cm) || !Number.isFinite(y1_cm)) return;
  if (cell <= 0) return;

  // WHY: Convert to grid-space coordinates (cell units).
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

  // WHY: Degenerate segment: single cell.
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

  // WHY: How far along the ray (in [0..1] of the segment) to the first boundary.
  let tMaxX = Infinity;
  let tMaxY = Infinity;
  if (stepX > 0) tMaxX = ((cx + 1) - x0) / dx;
  else if (stepX < 0) tMaxX = (x0 - cx) / (-dx);
  if (stepY > 0) tMaxY = ((cy + 1) - y0) / dy;
  else if (stepY < 0) tMaxY = (y0 - cy) / (-dy);

  // WHY: Guard against tiny numeric negatives.
  tMaxX = Math.max(0, tMaxX);
  tMaxY = Math.max(0, tMaxY);

  // CONTRACT: Step cap prevents infinite loops under numeric/pathological cases.
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

// WHY: Clip out-of-bounds endpoint to last in-bounds segment point.
export function clipEndpointToMap(grid, x0_cm, y0_cm, x1_cm, y1_cm) {
  const xMin = 0;
  const yMin = 0;
  const xMax = grid.mapW_cm;
  const yMax = grid.mapH_cm;

  // WHY: Match HitGrid.worldToCell() bounds: [0, max) not [0, max].
  const in0 = x0_cm >= xMin && x0_cm < xMax && y0_cm >= yMin && y0_cm < yMax;
  if (!in0) return { x: x0_cm, y: y0_cm, clipped: true };

  const in1 = x1_cm >= xMin && x1_cm < xMax && y1_cm >= yMin && y1_cm < yMax;
  if (in1) return { x: x1_cm, y: y1_cm, clipped: false };

  const dx = x1_cm - x0_cm;
  const dy = y1_cm - y0_cm;
  let tExit = 1;

  // WHY: For each boundary, compute t where the segment intersects it.
  if (dx > 0) tExit = Math.min(tExit, (xMax - x0_cm) / dx);
  else if (dx < 0) tExit = Math.min(tExit, (xMin - x0_cm) / dx);
  if (dy > 0) tExit = Math.min(tExit, (yMax - y0_cm) / dy);
  else if (dy < 0) tExit = Math.min(tExit, (yMin - y0_cm) / dy);

  tExit = clamp(tExit, 0, 1);
  // WHY: Step slightly back into the map to ensure worldToCell() succeeds.
  const eps = Math.min(0.49 * grid.cell_cm, 0.5);
  const tBack = (Math.hypot(dx, dy) > 1e-9) ? (eps / Math.hypot(dx, dy)) : 0;
  const t = clamp(tExit - tBack, 0, 1);
  return { x: x0_cm + dx * t, y: y0_cm + dy * t, clipped: true };
}

