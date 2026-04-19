// Browser-side autonomy for the /maze page.
// Constraints:
// - Do not change mapping behavior (we only call MappingApi.requestScanAuto()).
// - First scan is always '+' and scans must alternate (enforced by mapping_boot.js).
// - Red floor tile is a 1-cell virtual obstacle (not LiDAR-mapped).
// - Arduino/Mega reflex stops publish EVT:* lines; browser polls /alerts.

function $(id) {
  return document.getElementById(id);
}

const elStart = $("autoStart");
const elStop = $("autoStop");
const elStatus = $("autoStatus");

if (!elStart || !elStop || !elStatus) {
  // Page doesn't include autonomy UI; safe no-op.
} else {
  const kPollAlertsMs = 140;
  const kCmdTimeoutMs = 25000;

  const kBackoffAfterReflexCm = -6; // tunable, "middle" value
  const kMaxSegmentCm = 25; // cap single forward command length to reduce drift
  const kScanAfterEachSegment = true;

  let running = false;
  let stopRequested = false;

  // Single source of truth: read goal from `#mapCanvas` data-*.
  let goal = { x_cm: NaN, y_cm: NaN, tolCells: 0, cell: null };

  let alertFromSeq = 0;
  let alertTimer = 0;

  let pendingCmd = null; // { resolve, reject, t0, label }
  let reflex = null; // { kind: "red"|"front", x_cm, y_cm }

  let virtualBlocked = null; // Uint8Array grid.w*grid.h (1 => blocked)

  function setStatus(text) {
    elStatus.textContent = String(text || "");
    if (window.StatusBus) window.StatusBus.set("autonomy", { state: String(text || "") });
  }

  function mappingApi() {
    const api = window.MappingApi;
    if (!api || !api.grid || !api.cfg || !api.state || !api.getPose || !api.requestScanAuto) return null;
    return api;
  }

  function wrapDiff180(deg) {
    let d = deg;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
  }

  function roundInt(v) {
    const n = Math.round(Number(v));
    return Number.isFinite(n) ? n : 0;
  }

  async function httpPost(path) {
    if (window.DebugLog) window.DebugLog.push("http", `POST ${path}`);
    const res = await fetch(path, { method: "POST", cache: "no-store" });
    if (window.DebugLog) window.DebugLog.push("http", `POST ${path} -> ${res.status}`);
    return res;
  }

  async function cmdMove(cm) {
    const v = roundInt(cm);
    return await cmdAwait(`/move?cm=${encodeURIComponent(String(v))}`, `move ${v}cm`);
  }

  async function cmdTurn(deg) {
    const v = roundInt(deg);
    return await cmdAwait(`/turn?deg=${encodeURIComponent(String(v))}`, `turn ${v}deg`);
  }

  function cmdAwait(path, label) {
    if (!running) return Promise.resolve({ ok: false, status: "stopped" });
    if (pendingCmd) return Promise.reject(new Error("CMD_IN_FLIGHT"));

    return new Promise(async (resolve, reject) => {
      if (window.NetGate) window.NetGate.enterCmd();
      const cmdStartMs = Date.now();
      if (window.DebugLog) window.DebugLog.push("cmd", `begin ${label}`);
      if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: label, cmdAgeMs: 0 });
      pendingCmd = { resolve, reject, t0: Date.now(), label };
      try {
        const res = await httpPost(path);
        if (!res.ok) {
          if (window.DebugLog) window.DebugLog.push("cmd", `http fail ${label} (${res.status})`);
          pendingCmd = null;
          if (window.NetGate) window.NetGate.exitCmd();
          if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: null, cmdAgeMs: null });
          resolve({ ok: false, status: `http_${res.status}` });
          return;
        }
      } catch {
        if (window.DebugLog) window.DebugLog.push("cmd", `http error ${label}`);
        pendingCmd = null;
        if (window.NetGate) window.NetGate.exitCmd();
        if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: null, cmdAgeMs: null });
        resolve({ ok: false, status: "http_error" });
        return;
      }

      const tick = () => {
        if (!pendingCmd) return;
        if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: label, cmdAgeMs: Date.now() - cmdStartMs });
        if (!running) {
          const pc = pendingCmd;
          pendingCmd = null;
          if (window.NetGate) window.NetGate.exitCmd();
          if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: null, cmdAgeMs: null });
          if (window.DebugLog) window.DebugLog.push("cmd", `end ${label} (stopped)`);
          pc.resolve({ ok: false, status: "stopped" });
          return;
        }
        if (Date.now() - pendingCmd.t0 > kCmdTimeoutMs) {
          const pc = pendingCmd;
          pendingCmd = null;
          if (window.NetGate) window.NetGate.exitCmd();
          if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: null, cmdAgeMs: null });
          if (window.DebugLog) window.DebugLog.push("cmd", `timeout ${label}`);
          pc.resolve({ ok: false, status: "timeout" });
          return;
        }
        setTimeout(tick, 80);
      };
      setTimeout(tick, 80);
    });
  }

  function parseEvt(line) {
    const s = String(line || "").trim();
    if (!s.startsWith("EVT:")) return null;
    const payload = s.substring(4);
    const parts = payload.split(",");
    const tag = String(parts[0] || "").trim();
    const x = Number(parts[1]);
    const y = Number(parts[2]);
    const h = Number(parts[3]);
    const extra = parts.length >= 5 ? Number(parts[4]) : NaN;
    return { tag, x_cm: x, y_cm: y, headingDeg: h, extra };
  }

  function vbIdx(grid, cx, cy) {
    return cy * grid.w + cx;
  }

  function vbGet(grid, cx, cy) {
    if (!virtualBlocked) return 0;
    if (cx < 0 || cy < 0 || cx >= grid.w || cy >= grid.h) return 0;
    return virtualBlocked[vbIdx(grid, cx, cy)] ? 1 : 0;
  }

  function vbSet(grid, cx, cy, v) {
    if (!virtualBlocked) return;
    if (cx < 0 || cy < 0 || cx >= grid.w || cy >= grid.h) return;
    virtualBlocked[vbIdx(grid, cx, cy)] = v ? 1 : 0;
    const api = mappingApi();
    if (api?.setVirtualBlocked) api.setVirtualBlocked(virtualBlocked);
  }

  function isCellBlocked(grid, cx, cy) {
    const st = grid.cellState(cx, cy);
    if (st === "occ") return true;
    if (vbGet(grid, cx, cy)) return true;
    return false;
  }

  function bfsPlan(grid, start, goalCell) {
    const w = grid.w;
    const h = grid.h;
    const startIdx = start.cy * w + start.cx;
    const goalIdx = goalCell.cy * w + goalCell.cx;

    const prev = new Int32Array(w * h);
    prev.fill(-1);
    const q = new Int32Array(w * h);
    let qh = 0;
    let qt = 0;

    if (isCellBlocked(grid, start.cx, start.cy)) return null;
    if (isCellBlocked(grid, goalCell.cx, goalCell.cy)) return null;

    prev[startIdx] = startIdx;
    q[qt++] = startIdx;

    const dirs = [
      { dx: 1, dy: 0 },
      { dx: -1, dy: 0 },
      { dx: 0, dy: 1 },
      { dx: 0, dy: -1 },
    ];

    while (qh < qt) {
      const idx = q[qh++];
      if (idx === goalIdx) break;
      const cx = idx % w;
      const cy = (idx / w) | 0;

      for (const d of dirs) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (isCellBlocked(grid, nx, ny)) continue;
        const nidx = ny * w + nx;
        if (prev[nidx] !== -1) continue;
        prev[nidx] = idx;
        q[qt++] = nidx;
      }
    }

    if (prev[goalIdx] === -1) return null;

    const path = [];
    let cur = goalIdx;
    for (;;) {
      const cx = cur % w;
      const cy = (cur / w) | 0;
      path.push({ cx, cy });
      if (cur === startIdx) break;
      cur = prev[cur];
      if (cur < 0) return null;
    }
    path.reverse();
    return path;
  }

  function pathToSegments(path) {
    if (!path || path.length < 2) return [];
    const segs = [];
    let curDir = null;
    let run = 0;

    function flush() {
      if (!curDir || run <= 0) return;
      segs.push({ dir: curDir, steps: run });
    }

    for (let i = 1; i < path.length; i++) {
      const a = path[i - 1];
      const b = path[i];
      const dx = b.cx - a.cx;
      const dy = b.cy - a.cy;
      let dir = null;
      if (dx === 1 && dy === 0) dir = "E";
      else if (dx === -1 && dy === 0) dir = "W";
      else if (dx === 0 && dy === 1) dir = "N";
      else if (dx === 0 && dy === -1) dir = "S";
      else return []; // non-4-neighbor (shouldn't happen)

      if (curDir === null) {
        curDir = dir;
        run = 1;
      } else if (dir === curDir) {
        run++;
      } else {
        flush();
        curDir = dir;
        run = 1;
      }
    }
    flush();
    return segs;
  }

  function dirToHeadingDeg(dir) {
    if (dir === "N") return 0;
    if (dir === "E") return 90;
    if (dir === "S") return 180;
    if (dir === "W") return 270;
    return 0;
  }

  function configureGoalFromDom(api) {
    const canvas = document.getElementById("mapCanvas");
    if (!canvas) return false;
    const gx = Number(canvas.dataset.goalXCm);
    const gy = Number(canvas.dataset.goalYCm);
    const tol = Number(canvas.dataset.goalTolCells);
    if (!Number.isFinite(gx) || !Number.isFinite(gy) || !Number.isFinite(tol)) return false;
    if (Number.isFinite(gx)) goal.x_cm = gx;
    if (Number.isFinite(gy)) goal.y_cm = gy;
    if (Number.isFinite(tol)) goal.tolCells = Math.max(0, Math.floor(tol));

    const c = api.grid.worldToCell(goal.x_cm, goal.y_cm);
    goal.cell = c;
    return !!c;
  }

  function isAtGoal(api, pose) {
    if (!goal.cell) return false;
    const c = api.grid.worldToCell(pose.x, pose.y);
    if (!c) return false;
    const dx = Math.abs(c.cx - goal.cell.cx);
    const dy = Math.abs(c.cy - goal.cell.cy);
    return dx <= goal.tolCells && dy <= goal.tolCells;
  }

  function setPlannedPathOverlay_(api, pathCells) {
    if (!api?.setPlannedPath) return;
    if (!Array.isArray(pathCells) || pathCells.length < 2) {
      api.setPlannedPath([]);
      return;
    }
    const cmPerCell = api.cfg.cell_cm || api.grid.cell_cm || 5;
    const pts = [];
    for (const c of pathCells) {
      const cx = Number(c?.cx);
      const cy = Number(c?.cy);
      if (!Number.isFinite(cx) || !Number.isFinite(cy)) continue;
      pts.push({ x: (cx + 0.5) * cmPerCell, y: (cy + 0.5) * cmPerCell });
    }
    api.setPlannedPath(pts);
  }

  async function pollAlertsOnce() {
    const api = mappingApi();
    if (!api) return;
    if (!running) return;
    if (api.state.scanActive) return; // preserve scan bandwidth
    if (window.NetGate && !window.NetGate.allow("alerts")) return;

    let res;
    try {
      res = await fetch(`/alerts?from=${encodeURIComponent(String(alertFromSeq))}`, { cache: "no-store" });
    } catch {
      return;
    }
    if (!res.ok) {
      if (res.status === 403) setStatus("NOT ARMED");
      return;
    }
    const text = await res.text();
    if (window.DebugLog && text.trim()) window.DebugLog.push("alerts", text.trim().split("\n").length + " lines");
    const rows = String(text || "").split("\n");
    for (const row of rows) {
      if (!row) continue;
      const bar = row.indexOf("|");
      if (bar <= 0) continue;
      const seq = Number(row.substring(0, bar));
      const line = row.substring(bar + 1);
      if (Number.isFinite(seq)) alertFromSeq = Math.max(alertFromSeq, seq);

      const evt = parseEvt(line);
      if (!evt) continue;
      if (window.DebugLog) window.DebugLog.push("evt", line.trim());
      if (window.StatusBus) window.StatusBus.set("autonomy", { lastEvt: evt.tag, lastAlertSeq: alertFromSeq });

      if (evt.tag === "RED") {
        reflex = { kind: "red", x_cm: evt.x_cm, y_cm: evt.y_cm };
      } else if (evt.tag === "FRONTSTOP") {
        reflex = { kind: "front", x_cm: evt.x_cm, y_cm: evt.y_cm };
      } else if (evt.tag === "CMDOK" || evt.tag === "CMDFAIL" || evt.tag === "CMDCANCEL") {
        if (pendingCmd) {
          const pc = pendingCmd;
          pendingCmd = null;
          if (window.NetGate) window.NetGate.exitCmd();
          if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: null, cmdAgeMs: null });
          const ok = evt.tag === "CMDOK";
          if (window.DebugLog) window.DebugLog.push("cmd", `end ${pc.label} (${evt.tag})`);
          pc.resolve({ ok, status: evt.tag, pose: { x_cm: evt.x_cm, y_cm: evt.y_cm, h: evt.headingDeg } });
        }
      }
    }
  }

  function startAlertPolling() {
    if (alertTimer) return;
    alertTimer = setInterval(() => {
      pollAlertsOnce().catch(() => {});
    }, kPollAlertsMs);
  }

  function stopAlertPolling() {
    if (!alertTimer) return;
    clearInterval(alertTimer);
    alertTimer = 0;
  }

  async function ensureInitialLocalization(api) {
    if (api.state.poseLocked) return true;
    setStatus("initial scan...");
    const r = await api.requestScanAuto();
    if (!r?.ok) return false;
    if (!api.state.poseLocked) return false;
    return true;
  }

  async function doScan(api, reason) {
    if (!running) return { ok: false, status: "stopped" };
    setStatus(`scan (${reason})...`);
    const r = await api.requestScanAuto();
    if (!r?.ok) return { ok: false, status: r?.kind || "scan_fail" };
    return { ok: true, status: "scan_done" };
  }

  async function handleReflex(api) {
    if (!reflex) return false;
    const r = reflex;
    reflex = null;

    if (r.kind === "red") {
      const c = api.grid.worldToCell(r.x_cm, r.y_cm);
      if (c) vbSet(api.grid, c.cx, c.cy, 1);
    }

    setStatus(`${r.kind.toUpperCase()} stop -> backoff`);
    await cmdMove(kBackoffAfterReflexCm);
    await doScan(api, `${r.kind}`);
    return true;
  }

  async function runLoop() {
    const api = mappingApi();
    if (!api) {
      setStatus("MappingApi missing");
      running = false;
      return;
    }

    if (api.setPlannedPath) api.setPlannedPath([]);

    if (!virtualBlocked || virtualBlocked.length !== api.grid.w * api.grid.h) {
      virtualBlocked = new Uint8Array(api.grid.w * api.grid.h);
    }
    if (api.setVirtualBlocked) api.setVirtualBlocked(virtualBlocked);

    if (!configureGoalFromDom(api) || !goal.cell) {
      setStatus("bad goal");
      running = false;
      return;
    }

    setStatus(`goal cell=(${goal.cell.cx},${goal.cell.cy})`);
    if (window.StatusBus) {
      window.StatusBus.set("autonomy", {
        running: true,
        goalCx: goal.cell.cx,
        goalCy: goal.cell.cy,
        pathLen: null,
        segDir: null,
        segCm: null,
        turnDeg: null,
      });
    }

    const okInit = await ensureInitialLocalization(api);
    if (!okInit) {
      setStatus("init scan failed");
      running = false;
      return;
    }

    await doScan(api, "start");

    while (running && !stopRequested) {
      await handleReflex(api);
      if (!running || stopRequested) break;

      const pose = api.getPose();
      if (isAtGoal(api, pose)) {
        setStatus("GOAL REACHED");
        break;
      }

      const startCell = api.grid.worldToCell(pose.x, pose.y);
      if (!startCell) {
        // If pose drifts out of bounds, force a scan to pull it back.
        await doScan(api, "recover");
        continue;
      }
      if (window.StatusBus) window.StatusBus.set("autonomy", { startCx: startCell.cx, startCy: startCell.cy });

      const path = bfsPlan(api.grid, startCell, goal.cell);
      if (!path) {
        setStatus("NO PATH");
        setPlannedPathOverlay_(api, null);
        if (window.StatusBus) window.StatusBus.set("autonomy", { pathLen: null });
        await doScan(api, "no-path");
        continue;
      }

      setPlannedPathOverlay_(api, path);
      if (window.StatusBus) window.StatusBus.set("autonomy", { pathLen: path.length });

      const segs = pathToSegments(path);
      if (!segs.length) {
        setStatus("GOAL CELL");
        setPlannedPathOverlay_(api, path);
        break;
      }

      // Execute just one short segment per loop iteration, then rescan/replan.
      const seg = segs[0];
      const targetH = dirToHeadingDeg(seg.dir);
      const curH = Number(pose.headingDeg);
      const dTurn = wrapDiff180(targetH - curH);
      if (Math.abs(dTurn) > 3) {
        setStatus(`turn -> ${seg.dir}`);
        if (window.StatusBus) window.StatusBus.set("autonomy", { segDir: seg.dir, turnDeg: dTurn });
        await cmdTurn(dTurn);
        await handleReflex(api);
        if (!running || stopRequested) break;
      }

      const cmPerCell = api.cfg.cell_cm || api.grid.cell_cm || 2;
      const maxCells = Math.max(1, Math.floor(kMaxSegmentCm / cmPerCell));
      const cellsToMove = Math.max(1, Math.min(seg.steps, maxCells));
      const moveCm = cellsToMove * cmPerCell;
      setStatus(`move ${moveCm}cm`);
      if (window.StatusBus) window.StatusBus.set("autonomy", { segDir: seg.dir, segCm: moveCm });
      await cmdMove(moveCm);

      await handleReflex(api);
      if (!running || stopRequested) break;

      if (kScanAfterEachSegment) await doScan(api, "step");
    }

    running = false;
    stopRequested = false;
    stopAlertPolling();
    if (api?.setPlannedPath) api.setPlannedPath([]);
    if (pendingCmd) {
      const pc = pendingCmd;
      pendingCmd = null;
      if (window.NetGate) window.NetGate.exitCmd();
      if (window.StatusBus) window.StatusBus.set("autonomy", { cmdLabel: null, cmdAgeMs: null });
      pc.resolve({ ok: false, status: "stopped" });
    }
    if (window.StatusBus) window.StatusBus.set("autonomy", { running: false });
    if (elStart) elStart.disabled = false;
    setStatus("idle");
  }

  elStart.addEventListener("click", () => {
    if (running) return;
    const api = mappingApi();
    if (!api) {
      setStatus("MappingApi missing");
      return;
    }
    running = true;
    stopRequested = false;
    alertFromSeq = 0;
    reflex = null;
    startAlertPolling();
    elStart.disabled = true;
    runLoop().catch(() => {
      running = false;
      stopRequested = false;
      stopAlertPolling();
      if (elStart) elStart.disabled = false;
      setStatus("AUTO ERROR");
    });
  });

  elStop.addEventListener("click", () => {
    stopRequested = true;
    setStatus("stopping...");
  });

  setStatus("idle");
}
