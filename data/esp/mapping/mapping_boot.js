import { bodyToMap, polarToBody, wrap360 } from "./conventions.js";
import { HitGrid } from "./grid.js";
import { clipEndpointToMap, forEachCellOnSegmentWorld } from "./raycast.js";
import { parseTscanPayload } from "./tscan.js";
import { MapRenderer } from "./render.js";
import { searchWindowRectWalls } from "./matcher.js";

// Browser-side mapping bootstrap for /maze page.
// Keeps Mega "executor" and browser "brain".
const BUILD = "mapping_boot_v5";

const elCanvas = document.getElementById("mapCanvas");
const elStatus = document.getElementById("mapStatus");
const btnPlus = document.getElementById("mapScanPlus");
const btnMinus = document.getElementById("mapScanMinus");
const btnClear = document.getElementById("mapClear");

if (!elCanvas || !elStatus || !btnPlus || !btnMinus || !btnClear) {
  // Page doesn't have mapping UI; nothing to do.
  // (We keep this file safe to include on other pages.)
} else {
(() => {
  const dbg = {
    lastHttp: "-",
    lastSeq: 0,
    lastEvent: "none",
    pollErrors: 0,
  };

  function setStatus(t) {
    elStatus.textContent = `${t} | build=${BUILD} http=${dbg.lastHttp} seq=${dbg.lastSeq} evt=${dbg.lastEvent}`;
  }

  // Single source of truth for all tunables: `data/esp/maze.html` <canvas id="mapCanvas" data-...>.
  // No JS fallbacks for core config (prevents silent drift between files).
  function reqNum_(prop, { min = null, max = null } = {}) {
    const raw = elCanvas.dataset[prop];
    const v = Number(raw);
    if (!Number.isFinite(v)) return { ok: false, v: 0, why: `missing/invalid ${prop}` };
    if (min != null && v < min) return { ok: false, v, why: `${prop} < ${min}` };
    if (max != null && v > max) return { ok: false, v, why: `${prop} > ${max}` };
    return { ok: true, v };
  }

  const rMapW = reqNum_("mapWCm", { min: 1 });
  const rMapH = reqNum_("mapHCm", { min: 1 });
  const rCell = reqNum_("cellCm", { min: 0.1 });
  const rMargin = reqNum_("viewMarginCm", { min: 0, max: 200 });

  const rMinDist = reqNum_("matchMinDistCm", { min: 0 });
  const rMaxDist = reqNum_("matchMaxDistCm", { min: 0 });
  const rWOcc = reqNum_("matchWOccHit", { min: 0 });
  const rWFree = reqNum_("matchWFreeHit", { min: 0 });
  const rWConf = reqNum_("matchWConflict", { min: 0 });
  const rMaxCands = reqNum_("matchMaxCandidates", { min: 1 });

  const rUpdFree = reqNum_("updateFreeDelta");
  const rUpdOcc = reqNum_("updateOccDelta");

  const rInitY = reqNum_("initYmaxFrac", { min: 0, max: 1 });
  const rInitH = reqNum_("initHeadingSpanDeg", { min: 0, max: 180 });
  const rSigma = reqNum_("initWallSigmaCm", { min: 0.01, max: 50 });

  const rTDx = reqNum_("trackDxCm", { min: 0 });
  const rTDy = reqNum_("trackDyCm", { min: 0 });
  const rTDh = reqNum_("trackDheadingDeg", { min: 0, max: 180 });

  const required = [rMapW, rMapH, rCell, rMargin, rMinDist, rMaxDist, rWOcc, rWFree, rWConf, rMaxCands, rUpdFree, rUpdOcc, rInitY, rInitH, rSigma, rTDx, rTDy, rTDh];
  const bad = required.find((r) => !r.ok);
  if (bad) {
    setStatus(`CONFIG ERROR: ${bad.why}`);
    btnPlus.disabled = true;
    btnMinus.disabled = true;
    btnClear.disabled = true;
    // Stop here: avoid running with silent defaults.
    return;
  }

  btnPlus.disabled = false;
  btnMinus.disabled = false;
  btnClear.disabled = false;

  // Dimensions follow our agreed convention: width (X/East) is smaller, height (Y/North) is larger.
  const mapW_cm = rMapW.v;
  const mapH_cm = rMapH.v;
  const cell_cm = rCell.v;

  // Keep canvas aspect ratio consistent with the configured map.
  elCanvas.style.aspectRatio = `${mapW_cm} / ${mapH_cm}`;

  const grid = new HitGrid(mapW_cm, mapH_cm, cell_cm);
  const renderer = new MapRenderer(elCanvas, grid);
  renderer.setViewMarginCm(rMargin.v);

  // Goal marker (optional but expected on /maze): read from the single source of truth (#mapCanvas data-*).
  // Goal lives in map/world cm coordinates (X=east, Y=north).
  {
    const gx = Number(elCanvas.dataset.goalXCm);
    const gy = Number(elCanvas.dataset.goalYCm);
    const tolCells = Math.max(0, Math.floor(Number(elCanvas.dataset.goalTolCells || 0)));
    if (Number.isFinite(gx) && Number.isFinite(gy)) {
      const c = grid.worldToCell(gx, gy);
      if (c) renderer.setGoalCell({ cx: c.cx, cy: c.cy, tolCells });
    }
  }

  const cfg = {
    mapW_cm,
    mapH_cm,
    cell_cm,
    lidarOffset: { x_cm: 0.0, y_cm: 0.0 },
    match: {
      // Scan validity
      minDistCm: rMinDist.v,
      maxDistCm: rMaxDist.v,
      // Scoring weights (3-state: OCC/FREE/UNK)
      wOccHit: rWOcc.v,
      wFreeHit: rWFree.v,
      wConflict: rWConf.v,
      maxCandidates: rMaxCands.v,
    },
    update: {
      // Map integration deltas (log-odds). Set to 0 to disable free/occ updates.
      freeDelta: rUpdFree.v,
      occDelta: rUpdOcc.v,
    },
    // Initial localization (wide):
    // - X: full map width
    // - Y: [0..0.4*H] band
    // - Heading: around compass +/- span
    init: {
      yMaxFrac: rInitY.v,
      headingSpanDeg: rInitH.v,
      stepCmCoarse: 2,
      stepDegCoarse: 6,
      stepCmFine: 1,
      stepDegFine: 2,
      refineSpanCm: 3,
      refineSpanDeg: 10,
      // Distance-to-wall scoring kernel width (cm). Smaller = stricter.
      wallSigmaCm: rSigma.v,
    },
    track: {
      dxCm: rTDx.v,
      dyCm: rTDy.v,
      dHeadingDeg: rTDh.v,
      // Keep tracking fine and bounded.
      stepCm: 1,
      stepDeg: 2,
    },
  };

  // Pose model:
  // For initial localization we snapshot odom+compass at scan start via HTTP (no background streaming).
  // Once pose is locked, we keep using odom deltas (east/north) around an anchor.
  const state = {
    compassDeg: 0,
    odomEast: 0,
    odomNorth: 0,
    odomAnchorEast: 0,
    odomAnchorNorth: 0,
    mapPose0: { x: mapW_cm * 0.5, y: mapH_cm * 0.2, headingDeg: 0 },
    poseLocked: false, // becomes true after initial localization
    poseSentToRobot: false,

    scanActive: false,
    scanDoneSeen: false,
    scanDir: "+",
    scanPose: null,
    scanSamples: [],  // {angleDeg, distCm}
    scanBodyPts: [],  // {xb,yb} for matching
    scanWorldPts: [], // for overlay

    // Scan direction alternation (wiring safety):
    // - first scan must be '+'
    // - subsequent scans alternate on DONE
    nextScanDir: "+",
  };

  function currentPose() {
    // Pose contract:
    // - Before we align Mega world-odom to the map (no /set_pose yet), we can only
    //   display a pose relative to our matched anchor.
    // - After /set_pose, Mega's /odom reports absolute (east,north) in the map frame,
    //   so we must use it directly (otherwise we'd apply the correction twice).
    const headingDeg = wrap360(state.compassDeg);
    if (state.poseSentToRobot && Number.isFinite(state.odomEast) && Number.isFinite(state.odomNorth)) {
      return { x: state.odomEast, y: state.odomNorth, headingDeg };
    }
    const dx = state.odomEast - state.odomAnchorEast;
    const dy = state.odomNorth - state.odomAnchorNorth;
    return { x: state.mapPose0.x + dx, y: state.mapPose0.y + dy, headingDeg };
  }

  function seedOuterWalls() {
    // Fill map boundary cells as occupied (simple known-rectangle world).
    for (let cx = 0; cx < grid.w; cx++) {
      grid.setOccupied(cx, 0, 70);
      grid.setOccupied(cx, grid.h - 1, 70);
    }
    for (let cy = 0; cy < grid.h; cy++) {
      grid.setOccupied(0, cy, 70);
      grid.setOccupied(grid.w - 1, cy, 70);
    }
  }

  function clearMap() {
    grid.clear();
    seedOuterWalls();
    state.scanWorldPts = [];
    state.scanBodyPts = [];
    // reset anchor so current odom becomes "mapPose0"
    state.odomAnchorEast = state.odomEast;
    state.odomAnchorNorth = state.odomNorth;
    state.poseLocked = false;
    state.poseSentToRobot = false;
    setStatus("cleared (walls seeded, pose unlocked)");
    redraw();
  }

  async function post(path) {
    try {
      await fetch(path, { method: "POST", cache: "no-store" });
    } catch {}
  }

  async function postPoseToRobot(pose, opts = {}) {
    if (!pose) return;
    const x = Number(pose.x);
    const y = Number(pose.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const includeHeading = !!opts.includeHeading;
    const h = Number(pose.headingDeg);
    try {
      let url = `/set_pose?x=${encodeURIComponent(x.toFixed(2))}&y=${encodeURIComponent(y.toFixed(2))}`;
      if (includeHeading && Number.isFinite(h)) url += `&h=${encodeURIComponent(wrap360(h).toFixed(2))}`;
      await fetch(url, { method: "POST", cache: "no-store" });
    } catch {}
  }

  // Buttons are wired later to start a scan and poll until DONE.

  function redraw() {
    const pose = currentPose();
    renderer.setPose(pose);
    renderer.setScanPoints(state.scanWorldPts);
    renderer.draw();
  }

  function setVirtualBlocked(blocked01) {
    renderer.setVirtualBlocked(blocked01 || null);
    redraw();
  }

  function setPlannedPath(pointsWorld) {
    renderer.setPlannedPath(Array.isArray(pointsWorld) ? pointsWorld : []);
    redraw();
  }

  async function fetchCompassDeg() {
    // /compassdata returns "123,N" (or "--")
    const res = await fetch("/compassdata", { cache: "no-store" });
    const text = String(await res.text()).trim();
    if (!text || text === "--") return null;
    const parts = text.split(",");
    const deg = Number(parts[0]);
    return Number.isFinite(deg) ? deg : null;
  }

  async function fetchOdomEN() {
    // /odom returns "ODOM:e,n" or "ODOM:--"
    const res = await fetch("/odom", { cache: "no-store" });
    const text = String(await res.text()).trim();
    if (!text.startsWith("ODOM:")) return null;
    const payload = text.substring(5);
    if (payload === "--") return null;
    const parts = payload.split(",");
    const e = Number(parts[0]);
    const n = Number(parts[1]);
    if (!Number.isFinite(e) || !Number.isFinite(n)) return null;
    return { e, n };
  }

  // Live pose polling (low-rate) so the arrow moves continuously while driving.
  // We pause this during scan polling to avoid competing for bandwidth/CPU.
  let posePollInFlight = false;
  async function pollPoseOnce() {
    if (posePollInFlight) return;
    if (pollActive || state.scanActive) return;
    posePollInFlight = true;
    try {
      const [od, hdg] = await Promise.all([fetchOdomEN(), fetchCompassDeg()]);
      if (od) {
        state.odomEast = od.e;
        state.odomNorth = od.n;
      }
      if (hdg != null) state.compassDeg = hdg;
      redraw();
    } catch {
      // ignore transient errors
    } finally {
      posePollInFlight = false;
    }
  }

  function precomputeRot(headingDeg) {
    const h = (headingDeg * Math.PI) / 180;
    return { c: Math.cos(h), s: Math.sin(h) };
  }

  function bodyDeltaToMap(rot, xb, yb) {
    const { c, s } = rot;
    // East  = s*xb + c*yb
    // North = c*xb - s*yb
    return { x: s * xb + c * yb, y: c * xb - s * yb };
  }

  function applyScanToMap(pose, scanSamples) {
    if (!pose || !scanSamples?.length) return;
    const freeDelta = cfg.update.freeDelta;
    const occDelta = cfg.update.occDelta;
    if (!(freeDelta > 0) && !(occDelta > 0)) return;

    const rot = precomputeRot(pose.headingDeg);
    const ox = cfg.lidarOffset.x_cm || 0;
    const oy = cfg.lidarOffset.y_cm || 0;
    const oMap = bodyDeltaToMap(rot, ox, oy);
    const x0 = pose.x + oMap.x;
    const y0 = pose.y + oMap.y;

    const minDist = cfg.match.minDistCm;
    const maxDist = cfg.match.maxDistCm;

    for (let i = 0; i < scanSamples.length; i++) {
      const sm = scanSamples[i];
      const distCm = Number(sm.distCm);
      const angleDeg = Number(sm.angleDeg);
      if (!Number.isFinite(distCm) || !Number.isFinite(angleDeg)) continue;
      if (distCm < minDist || distCm > maxDist) continue;

      const a = (angleDeg * Math.PI) / 180;
      const dxB = distCm * Math.cos(a);
      const dyB = distCm * Math.sin(a);
      const eMap = bodyDeltaToMap(rot, ox + dxB, oy + dyB);
      const x1 = pose.x + eMap.x;
      const y1 = pose.y + eMap.y;

      const clipped = clipEndpointToMap(grid, x0, y0, x1, y1);
      const hasOccEndpoint = clipped.clipped === false;
      const endCell = hasOccEndpoint ? grid.worldToCell(x1, y1) : null;

      // Mark FREE along the ray, excluding the OCC endpoint cell.
      if (freeDelta > 0) {
        forEachCellOnSegmentWorld(grid, x0, y0, clipped.x, clipped.y, (cx, cy) => {
          if (hasOccEndpoint && endCell && cx === endCell.cx && cy === endCell.cy) return false;
          // Don't erode existing occupied structure (e.g., known outer walls) due to small discretization errors.
          if (grid.cellState(cx, cy) !== "occ") grid.addFree(cx, cy, freeDelta);
          return true;
        });
      }

      // Mark OCC at the endpoint (if in-bounds).
      if (occDelta > 0 && endCell) grid.addOccupied(endCell.cx, endCell.cy, occDelta);
    }
  }

  function initRectWallMatchCfg_() {
    return {
      ...cfg.match,
      lidarOffset: cfg.lidarOffset,
      wallSigmaCm: cfg.init.wallSigmaCm,
    };
  }

  function buildInitWindow_(priorH) {
    return {
      xMin: 0,
      xMax: cfg.mapW_cm,
      yMin: 0,
      yMax: cfg.init.yMaxFrac * cfg.mapH_cm,
      hMin: priorH - cfg.init.headingSpanDeg,
      hMax: priorH + cfg.init.headingSpanDeg,
      // Global fine search over the full uncertainty range (do not "converge early").
      stepX: cfg.init.stepCmFine,
      stepY: cfg.init.stepCmFine,
      stepH: cfg.init.stepDegFine,
    };
  }

  function buildTrackWindow_(priorPose) {
    const p = priorPose || currentPose();
    return {
      xMin: p.x - cfg.track.dxCm,
      xMax: p.x + cfg.track.dxCm,
      yMin: p.y - cfg.track.dyCm,
      yMax: p.y + cfg.track.dyCm,
      hMin: p.headingDeg - cfg.track.dHeadingDeg,
      hMax: p.headingDeg + cfg.track.dHeadingDeg,
      stepX: cfg.track.stepCm,
      stepY: cfg.track.stepCm,
      stepH: cfg.track.stepDeg,
    };
  }

  function matchRectWalls_(window) {
    return searchWindowRectWalls(state.scanSamples, cfg.mapW_cm, cfg.mapH_cm, window, initRectWallMatchCfg_());
  }

  function commitLockedPose_(pose) {
    state.mapPose0 = { x: pose.x, y: pose.y, headingDeg: pose.headingDeg };
    state.odomAnchorEast = state.odomEast;
    state.odomAnchorNorth = state.odomNorth;
    state.poseLocked = true;
    if (!state.poseSentToRobot) {
      state.poseSentToRobot = true;
      // Initial: send position only (heading is already aligned via your manual north reset).
      postPoseToRobot(state.mapPose0, { includeHeading: false });
      // Immediately snap the displayed pose to the matched pose; /odom will follow
      // once Mega rebases its world frame.
      state.odomEast = state.mapPose0.x;
      state.odomNorth = state.mapPose0.y;
    }
  }

  function handleLine(line) {
    const s = String(line || "").trim();
    if (!s) return;
    if (!s.startsWith("TSCAN:")) return;

    // "TSCAN:" is 6 chars; pass only the payload after the colon.
    const msg = parseTscanPayload(s.substring(6));
    if (!msg) return;
    if (msg.kind === "begin") {
      // BEGIN can arrive multiple times (ESP synthetic + Mega real).
      state.scanDir = msg.payload.includes("BEGIN,-") ? "-" : "+";
      dbg.lastEvent = "BEGIN";
      setStatus(`scanning ${state.scanDir}`);
      return;
    }
    if (msg.kind === "done") {
      state.scanActive = false;
      state.scanDoneSeen = true;
      dbg.lastEvent = "DONE";

      let poseStatus = "";

      // Pose update:
      // - if not locked yet: global init match against known outer walls
      // - if already locked: local track match around odom+compass prior
      if (state.scanSamples.length) {
        const wasLocked = state.poseLocked;
        const priorPose = state.scanPose || currentPose();
        const w = state.poseLocked ? buildTrackWindow_(priorPose) : buildInitWindow_(wrap360(priorPose.headingDeg));
        const best = matchRectWalls_(w);
        if (best.ok) {
          commitLockedPose_(best.pose);
          if (wasLocked) {
            // Tracking: send position + matched heading for compass offset correction.
            postPoseToRobot(state.mapPose0, { includeHeading: true });
          }
          poseStatus = `pose: x=${best.pose.x.toFixed(1)} y=${best.pose.y.toFixed(1)} h=${best.pose.headingDeg.toFixed(1)} score=${best.score.toFixed(3)}`;
        } else if (!state.poseLocked) {
          poseStatus = "pose NOT locked (no candidates)";
        }
      }

      // Apply scan to the map ONLY when we have a locked pose (avoid polluting map with a wrong initial guess).
      // For UI overlay we still show endpoints using the best available pose estimate.
      const poseForOverlay = state.poseLocked ? state.mapPose0 : state.scanPose;
      state.scanWorldPts = [];
      if (state.poseLocked) {
        applyScanToMap(state.mapPose0, state.scanSamples);
      }

      if (poseForOverlay) {
        // UI overlay: stamp endpoints only (as points).
        for (const bp of state.scanBodyPts) {
          const xb = bp.xb + (cfg.lidarOffset.x_cm || 0);
          const yb = bp.yb + (cfg.lidarOffset.y_cm || 0);
          const d = bodyToMap(poseForOverlay.headingDeg, xb, yb);
          const xw = poseForOverlay.x + d.x;
          const yw = poseForOverlay.y + d.y;
          // Overlay should never mutate the map; keep out-of-bounds points too for debugging.
          state.scanWorldPts.push({ x: xw, y: yw });
        }
      }

      setStatus(`${poseStatus || "scan done"} (${state.scanBodyPts.length} pts)`);
      redraw();

      // Enforce alternating scan direction (flip only on successful DONE).
      state.nextScanDir = (state.scanDir === "+") ? "-" : "+";

      if (scanResolve) {
        scanResolve({ ok: true, kind: "done", dir: state.scanDir });
        scanResolve = null;
        scanPromise = null;
      }
      return;
    }
    if (msg.kind === "cancel") {
      state.scanActive = false;
      state.scanDoneSeen = true;
      dbg.lastEvent = "CANCEL";
      setStatus("scan cancelled");
      redraw();
      // Do not flip direction on cancel.
      if (scanResolve) {
        scanResolve({ ok: false, kind: "cancel", dir: state.scanDir });
        scanResolve = null;
        scanPromise = null;
      }
      return;
    }
    if (msg.kind !== "sample") return;
    if (!state.scanActive) return; // ignore stray samples when not scanning

    const { xb, yb } = polarToBody(msg.angleDeg, msg.distCm);
    state.scanSamples.push({ angleDeg: msg.angleDeg, distCm: msg.distCm });
    state.scanBodyPts.push({ xb, yb });
    dbg.lastEvent = "SAMPLE";
  }

  // Transactional scanning: poll only while we are actively scanning.
  let lastSeq = 0;

  let pollActive = false;
  let pollDeadlineMs = 0;
  let scanResolve = null;
  let scanPromise = null;

  async function pollOnce() {
    const res = await fetch(`/events?from=${lastSeq}`, { cache: "no-store" });
    dbg.lastHttp = String(res.status);
    if (!res.ok) {
      dbg.pollErrors++;
      if (res.status === 403) {
        dbg.lastEvent = "403";
        setStatus("NOT ARMED");
        stopPolling();
      }
      return;
    }
    const text = await res.text();
    const rows = text.split("\n");
    for (const row of rows) {
      if (!row) continue;
      const bar = row.indexOf("|");
      if (bar <= 0) continue;
      const seq = Number(row.substring(0, bar));
      const line = row.substring(bar + 1);
      if (Number.isFinite(seq)) {
        lastSeq = Math.max(lastSeq, seq);
        dbg.lastSeq = lastSeq;
      }
      handleLine(line);
    }
  }

  function stopPolling() {
    pollActive = false;
  }

  function startPollingUntilDone() {
    if (pollActive) return;
    pollActive = true;
    pollDeadlineMs = Date.now() + 20000;

    const tick = async () => {
      if (!pollActive) return;
      if (Date.now() > pollDeadlineMs) {
        stopPolling();
        setStatus("scan timeout");
        state.scanActive = false;
        state.scanDoneSeen = true;
        if (scanResolve) {
          scanResolve({ ok: false, kind: "timeout" });
          scanResolve = null;
          scanPromise = null;
        }
        return;
      }
      try {
        await pollOnce();
      } catch {}
      // Stop when DONE/CANCEL was observed.
      if (state.scanDoneSeen) {
        redraw();
        stopPolling();
        return;
      }
      setTimeout(tick, 120);
    };
    tick();
  }

  async function startScan(path, label) {
    if (pollActive) return;
    // Drain any queued events before starting, so BEGIN is seen promptly.
    try { await pollOnce(); } catch {}
    // Freeze live pose polling during the scan transaction (bandwidth priority).
    // (pollPoseOnce() also checks pollActive/scanActive, so this is just clarity.)
    // Snapshot pose at click time (does NOT depend on receiving BEGIN from ESP).
    const od = await fetchOdomEN();
    const hdg = await fetchCompassDeg();
    if (od) {
      state.odomEast = od.e;
      state.odomNorth = od.n;
    }
    if (hdg != null) state.compassDeg = hdg;
    state.scanPose = currentPose(); // snapshot at click time
    state.scanSamples = [];
    state.scanBodyPts = [];
    state.scanWorldPts = [];
    state.scanActive = true;
    state.scanDoneSeen = false;
    // Deterministic scan direction even if BEGIN is delayed/dropped.
    state.scanDir = String(path).includes("minus") ? "-" : "+";
    dbg.lastEvent = "START";
    dbg.pollErrors = 0;
    dbg.lastHttp = "-";
    dbg.lastSeq = 0;
    setStatus(`starting ${label}...`);
    lastSeq = 0; // ring is reset on scan start by the ESP

    // Create a completion promise for callers (autonomy).
    scanPromise = new Promise((resolve) => { scanResolve = resolve; });

    await post(path);
    startPollingUntilDone();
    return scanPromise;
  }

  function requestScanAuto() {
    // Enforce alternation (wiring safety): ignore manual direction preference.
    const dir = state.nextScanDir === "-" ? "-" : "+";
    return startScan(dir === "+" ? "/scan_plus" : "/scan_minus", `scan ${dir}`);
  }

  btnPlus.addEventListener("click", () => requestScanAuto());
  btnMinus.addEventListener("click", () => requestScanAuto());
  btnClear.addEventListener("click", () => { lastSeq = 0; clearMap(); });

  // initial
  clearMap();
  setStatus("idle");
  // Start low-rate pose polling for live arrow updates (paused automatically during scans).
  pollPoseOnce();
  setInterval(pollPoseOnce, 350);

  // Expose a tiny API for autonomy orchestration (does not change mapping behavior).
  window.MappingApi = {
    grid,
    cfg,
    state,
    getPose: () => currentPose(),
    requestScanAuto,
    setVirtualBlocked,
    setPlannedPath,
  };
})();
}
