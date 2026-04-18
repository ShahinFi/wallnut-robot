import { degToRad, wrap360 } from "./conventions.js";
import { clipEndpointToMap, forEachCellOnSegmentWorld } from "./raycast.js";

function precomputeRot(headingDeg) {
  const h = degToRad(headingDeg);
  return { c: Math.cos(h), s: Math.sin(h) };
}

function bodyDeltaToMap(rot, xb, yb) {
  const { c, s } = rot;
  // Same as Mega mapping/frame_conventions.h:
  // East  = s*xb + c*yb
  // North = c*xb - s*yb
  return { x: s * xb + c * yb, y: c * xb - s * yb };
}

// 3-state scan-to-map scoring:
// - reward OCC/OCC and FREE/FREE
// - penalize OCC/FREE conflicts (either direction)
// - UNKNOWN is neutral
function scorePose(pose, rot, scanSamples, grid, cfg) {
  const ox = cfg?.lidarOffset?.x_cm || 0;
  const oy = cfg?.lidarOffset?.y_cm || 0;
  const wOccHit = cfg?.wOccHit ?? 5;
  const wFreeHit = cfg?.wFreeHit ?? 1;
  const wConflict = cfg?.wConflict ?? 6;
  const minDistCm = cfg?.minDistCm ?? 2;
  const maxDistCm = cfg?.maxDistCm ?? 250;

  // Sensor origin in world frame.
  const oMap = bodyDeltaToMap(rot, ox, oy);
  const x0 = pose.x + oMap.x;
  const y0 = pose.y + oMap.y;

  let score = 0;

  for (let i = 0; i < scanSamples.length; i++) {
    const s = scanSamples[i];
    const distCm = Number(s.distCm);
    const angleDeg = Number(s.angleDeg);
    if (!Number.isFinite(distCm) || !Number.isFinite(angleDeg)) continue;
    if (distCm < minDistCm || distCm > maxDistCm) continue;

    const a = degToRad(angleDeg);
    const dxB = distCm * Math.cos(a);
    const dyB = distCm * Math.sin(a);
    const eMap = bodyDeltaToMap(rot, ox + dxB, oy + dyB);
    const x1 = pose.x + eMap.x;
    const y1 = pose.y + eMap.y;

    // If endpoint is outside the map, treat this beam as "free until boundary"
    // (no occupied endpoint evidence).
    const clipped = clipEndpointToMap(grid, x0, y0, x1, y1);
    const hasOccEndpoint = clipped.clipped === false;
    const endCell = hasOccEndpoint ? grid.worldToCell(x1, y1) : null;

    // Ray free-space evidence: cells along the ray, excluding the occupied endpoint cell.
    forEachCellOnSegmentWorld(grid, x0, y0, clipped.x, clipped.y, (cx, cy) => {
      if (hasOccEndpoint && endCell && cx === endCell.cx && cy === endCell.cy) return false;
      const st = grid.cellState(cx, cy);
      if (st === "occ") score -= wConflict;
      else if (st === "free") score += wFreeHit;
      return true;
    });

    // Endpoint occupied evidence (only when endpoint is in-bounds).
    if (hasOccEndpoint && endCell) {
      const st = grid.cellState(endCell.cx, endCell.cy);
      if (st === "occ") score += wOccHit;
      else if (st === "free") score -= wConflict;
    }
  }

  return score;
}

function clampWindow(w) {
  const out = { ...w };
  if (out.xMax < out.xMin) [out.xMin, out.xMax] = [out.xMax, out.xMin];
  if (out.yMax < out.yMin) [out.yMin, out.yMax] = [out.yMax, out.yMin];
  if (out.hMax < out.hMin) [out.hMin, out.hMax] = [out.hMax, out.hMin];
  return out;
}

export function searchWindow(scanSamples, grid, window, matchCfg) {
  const w = clampWindow(window);
  const stepX = w.stepX > 0 ? w.stepX : 2;
  const stepY = w.stepY > 0 ? w.stepY : 2;
  const stepH = w.stepH > 0 ? w.stepH : 4;

  let best = { ok: false, pose: { x: 0, y: 0, headingDeg: 0 }, score: -1 };

  // Keep it bounded; initial localization can still be "wide" but must not explode.
  const kMaxCandidates = matchCfg?.maxCandidates ?? 45000;
  let candidates = 0;

  for (let hdg = w.hMin; hdg <= w.hMax + 1e-6; hdg += stepH) {
    const headingDeg = wrap360(hdg);
    const rot = precomputeRot(headingDeg);
    for (let x = w.xMin; x <= w.xMax + 1e-6; x += stepX) {
      for (let y = w.yMin; y <= w.yMax + 1e-6; y += stepY) {
        const pose = { x, y, headingDeg };
        const s = scorePose(pose, rot, scanSamples, grid, matchCfg);
        if (s > best.score) best = { ok: true, pose, score: s };
        if (++candidates >= kMaxCandidates) return best;
      }
    }
  }
  return best;
}

// Rectangle-wall initializer scoring (endpoint-only, distance-based).
//
// For initial localization against a known empty rectangle, grid occupancy
// agreement is brittle (most cells are UNKNOWN). Instead, score how close scan
// endpoints land to the rectangle perimeter.
//
// Score is the average of a Gaussian kernel in [0..1] over valid endpoints:
//   contrib = exp(-0.5 * (d/sigma)^2)
// where d is distance-to-closest-wall in cm (works inside or slightly outside).
function scorePoseRectWalls(pose, rot, scanSamples, mapW_cm, mapH_cm, cfg) {
  const ox = cfg?.lidarOffset?.x_cm || 0;
  const oy = cfg?.lidarOffset?.y_cm || 0;
  const minDistCm = cfg?.minDistCm ?? 2;
  const maxDistCm = cfg?.maxDistCm ?? 250;
  const sigmaCm = Math.max(0.1, Number(cfg?.wallSigmaCm ?? 1.5));

  // Sensor origin in world frame.
  const oMap = bodyDeltaToMap(rot, ox, oy);
  const xBase = pose.x + oMap.x;
  const yBase = pose.y + oMap.y;

  let sum = 0;
  let n = 0;

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  for (let i = 0; i < scanSamples.length; i++) {
    const s = scanSamples[i];
    const distCm = Number(s.distCm);
    const angleDeg = Number(s.angleDeg);
    if (!Number.isFinite(distCm) || !Number.isFinite(angleDeg)) continue;
    if (distCm < minDistCm || distCm > maxDistCm) continue;

    const a = degToRad(angleDeg);
    const dxB = distCm * Math.cos(a);
    const dyB = distCm * Math.sin(a);
    const eMap = bodyDeltaToMap(rot, ox + dxB, oy + dyB);
    const x1 = pose.x + eMap.x;
    const y1 = pose.y + eMap.y;

    // Distance to rectangle boundary, allowing slight overshoot.
    // For inside points, distance is min to one of the four walls.
    // For outside points, distance is shortest distance back to the rectangle.
    const xCl = clamp(x1, 0, mapW_cm);
    const yCl = clamp(y1, 0, mapH_cm);
    const outside = (x1 < 0 || x1 > mapW_cm || y1 < 0 || y1 > mapH_cm);
    let d = 0;
    if (outside) {
      d = Math.hypot(x1 - xCl, y1 - yCl);
    } else {
      d = Math.min(x1, mapW_cm - x1, y1, mapH_cm - y1);
    }

    // Gaussian reward; 1 at wall, decays smoothly with distance.
    const z = d / sigmaCm;
    const contrib = Math.exp(-0.5 * z * z);
    sum += contrib;
    n++;
  }

  if (n === 0) return 0;
  return sum / n;
}

export function searchWindowRectWalls(scanSamples, mapW_cm, mapH_cm, window, matchCfg) {
  const w = clampWindow(window);
  const stepX = w.stepX > 0 ? w.stepX : 2;
  const stepY = w.stepY > 0 ? w.stepY : 2;
  const stepH = w.stepH > 0 ? w.stepH : 4;

  let best = { ok: false, pose: { x: 0, y: 0, headingDeg: 0 }, score: -Infinity };

  // Keep it bounded; initial localization can still be "wide" but must not explode.
  const kMaxCandidates = matchCfg?.maxCandidates ?? 45000;
  let candidates = 0;

  for (let hdg = w.hMin; hdg <= w.hMax + 1e-6; hdg += stepH) {
    const headingDeg = wrap360(hdg);
    const rot = precomputeRot(headingDeg);
    for (let x = w.xMin; x <= w.xMax + 1e-6; x += stepX) {
      for (let y = w.yMin; y <= w.yMax + 1e-6; y += stepY) {
        const pose = { x, y, headingDeg };
        const s = scorePoseRectWalls(pose, rot, scanSamples, mapW_cm, mapH_cm, matchCfg);
        if (s > best.score) best = { ok: true, pose, score: s };
        if (++candidates >= kMaxCandidates) return best;
      }
    }
  }
  return best;
}
