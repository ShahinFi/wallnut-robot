import { bodyToMap, polarToBody, wrap360 } from "./conventions.js";
import { HitGrid } from "./grid.js";
import { clipEndpointToMap, forEachCellOnSegmentWorld } from "./raycast.js";
import { parseTscanPayload } from "./tscan.js";
import { MapRenderer } from "./render.js";
import { searchWindow } from "./matcher.js";

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
  // Tunables live in the DOM (so you can change them later without touching JS logic).
  // See `data/esp/maze.html` <canvas id="mapCanvas" data-...>.
  // Default dimensions follow our agreed convention: width (X/East) is smaller, height (Y/North) is larger.
  const mapW_cm = Number(elCanvas.dataset.mapWCm || 26);
  const mapH_cm = Number(elCanvas.dataset.mapHCm || 46);
  const cell_cm = Number(elCanvas.dataset.cellCm || 2);

  // Keep canvas aspect ratio consistent with the configured map.
  elCanvas.style.aspectRatio = `${mapW_cm} / ${mapH_cm}`;

  const grid = new HitGrid(mapW_cm, mapH_cm, cell_cm);
  const renderer = new MapRenderer(elCanvas, grid);

  const cfg = {
    mapW_cm,
    mapH_cm,
    cell_cm,
    lidarOffset: { x_cm: 0.0, y_cm: 0.0 },
    match: {
      // Scan validity
      minDistCm: Number(elCanvas.dataset.matchMinDistCm || 2),
      maxDistCm: Number(elCanvas.dataset.matchMaxDistCm || 250),
      // Scoring weights (3-state: OCC/FREE/UNK)
      wOccHit: Number(elCanvas.dataset.matchWOccHit || 6),
      wFreeHit: Number(elCanvas.dataset.matchWFreeHit || 1),
      wConflict: Number(elCanvas.dataset.matchWConflict || 8),
      maxCandidates: Number(elCanvas.dataset.matchMaxCandidates || 45000),
    },
    update: {
      // Map integration deltas (log-odds). Set to 0 to disable free/occ updates.
      freeDelta: Number(elCanvas.dataset.updateFreeDelta || 3),
      occDelta: Number(elCanvas.dataset.updateOccDelta || 10),
    },
    // Initial localization (wide):
    // - X: full map width
    // - Y: [0..0.4*H] band
    // - Heading: around compass +/- span
    init: {
      yMaxFrac: Number(elCanvas.dataset.initYmaxFrac || 0.4),
      headingSpanDeg: Number(elCanvas.dataset.initHeadingSpanDeg || 90),
      stepCmCoarse: 2,
      stepDegCoarse: 6,
      stepCmFine: 1,
      stepDegFine: 2,
      refineSpanCm: 3,
      refineSpanDeg: 10,
      // With 3-state scoring, absolute scores are larger than the old hit-count method.
      // Keep it tunable; default is conservative.
      minScoreToAccept: Number(elCanvas.dataset.initMinScore || 120),
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

    scanActive: false,
    scanDoneSeen: false,
    scanDir: "+",
    scanPose: null,
    scanSamples: [],  // {angleDeg, distCm}
    scanBodyPts: [],  // {xb,yb} for matching
    scanWorldPts: [], // for overlay
  };

  const dbg = {
    lastHttp: "-",
    lastSeq: 0,
    lastEvent: "none",
    pollErrors: 0,
  };

  function setStatus(t) {
    elStatus.textContent = `${t} | build=${BUILD} http=${dbg.lastHttp} seq=${dbg.lastSeq} evt=${dbg.lastEvent}`;
  }

  function currentPose() {
    const dx = state.odomEast - state.odomAnchorEast;
    const dy = state.odomNorth - state.odomAnchorNorth;
    return {
      x: state.mapPose0.x + dx,
      y: state.mapPose0.y + dy,
      headingDeg: wrap360(state.compassDeg),
    };
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
    setStatus("cleared (walls seeded, pose unlocked)");
    redraw();
  }

  async function post(path) {
    try {
      await fetch(path, { method: "POST", cache: "no-store" });
    } catch {}
  }

  // Buttons are wired later to start a scan and poll until DONE.

  function redraw() {
    const pose = currentPose();
    renderer.setPose(pose);
    renderer.setScanPoints(state.scanWorldPts);
    renderer.draw();
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

      // Initial localization (wide) against the known outer walls, using scan endpoints only.
      // Once locked, we keep using odom+compass outside this file (later step).
      if (!state.poseLocked && state.scanSamples.length) {
        const priorH = wrap360(state.scanPose?.headingDeg ?? state.compassDeg);
        const wCoarse = {
          xMin: 0,
          xMax: cfg.mapW_cm,
          yMin: 0,
          yMax: cfg.init.yMaxFrac * cfg.mapH_cm,
          hMin: priorH - cfg.init.headingSpanDeg,
          hMax: priorH + cfg.init.headingSpanDeg,
          stepX: cfg.init.stepCmCoarse,
          stepY: cfg.init.stepCmCoarse,
          stepH: cfg.init.stepDegCoarse,
        };
        const coarse = searchWindow(state.scanSamples, grid, wCoarse, { ...cfg.match, lidarOffset: cfg.lidarOffset });

        let best = coarse;
        if (coarse.ok) {
          const wFine = {
            xMin: coarse.pose.x - cfg.init.refineSpanCm,
            xMax: coarse.pose.x + cfg.init.refineSpanCm,
            yMin: coarse.pose.y - cfg.init.refineSpanCm,
            yMax: coarse.pose.y + cfg.init.refineSpanCm,
            hMin: coarse.pose.headingDeg - cfg.init.refineSpanDeg,
            hMax: coarse.pose.headingDeg + cfg.init.refineSpanDeg,
            stepX: cfg.init.stepCmFine,
            stepY: cfg.init.stepCmFine,
            stepH: cfg.init.stepDegFine,
          };
          const fine = searchWindow(state.scanSamples, grid, wFine, { ...cfg.match, lidarOffset: cfg.lidarOffset });
          if (fine.ok && fine.score >= best.score) best = fine;
        }

        if (best.ok && best.score >= cfg.init.minScoreToAccept) {
          state.mapPose0 = { x: best.pose.x, y: best.pose.y, headingDeg: best.pose.headingDeg };
          state.odomAnchorEast = state.odomEast;
          state.odomAnchorNorth = state.odomNorth;
          state.poseLocked = true;
          setStatus(`pose locked: x=${best.pose.x.toFixed(1)} y=${best.pose.y.toFixed(1)} h=${best.pose.headingDeg.toFixed(1)} score=${best.score}`);
        } else {
          setStatus(`pose NOT locked (score=${best.score})`);
        }
      }

      // Apply scan to the map using the scan-start pose (or locked pose).
      const poseForStamp = state.poseLocked ? currentPose() : state.scanPose;
      state.scanWorldPts = [];
      if (poseForStamp) {
        // 1) Update the map (FREE along rays + OCC at endpoints).
        // We do this only once per scan at DONE (not on every sample) to keep it simple and deterministic.
        applyScanToMap(poseForStamp, state.scanSamples);

        // 2) For UI overlay, stamp endpoints only (as points).
        for (const bp of state.scanBodyPts) {
          const xb = bp.xb + (cfg.lidarOffset.x_cm || 0);
          const yb = bp.yb + (cfg.lidarOffset.y_cm || 0);
          const d = bodyToMap(poseForStamp.headingDeg, xb, yb);
          const xw = poseForStamp.x + d.x;
          const yw = poseForStamp.y + d.y;
          // Overlay should never mutate the map.
          if (grid.worldToCell(xw, yw)) state.scanWorldPts.push({ x: xw, y: yw });
        }
      }

      setStatus(`scan done (${state.scanBodyPts.length} pts)`);
      redraw();
      return;
    }
    if (msg.kind === "cancel") {
      state.scanActive = false;
      state.scanDoneSeen = true;
      dbg.lastEvent = "CANCEL";
      setStatus("scan cancelled");
      redraw();
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
    dbg.lastEvent = "START";
    dbg.pollErrors = 0;
    dbg.lastHttp = "-";
    dbg.lastSeq = 0;
    setStatus(`starting ${label}...`);
    lastSeq = 0; // ring is reset on scan start by the ESP
    await post(path);
    startPollingUntilDone();
  }

  btnPlus.addEventListener("click", () => startScan("/scan_plus", "scan +"));
  btnMinus.addEventListener("click", () => startScan("/scan_minus", "scan -"));
  btnClear.addEventListener("click", () => { lastSeq = 0; clearMap(); });

  // initial
  clearMap();
  setStatus("idle");
}
