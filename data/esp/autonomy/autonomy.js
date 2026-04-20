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

  const kBackoffAfterReflexCm = -8; // tunable, "middle" value
  const kMaxSegmentCm = 30; // cap single forward command length to reduce drift

  let running = false;
  let stopRequested = false;

  // Single source of truth: read goal from `#mapCanvas` data-*.
  let goal = { x_cm: NaN, y_cm: NaN, tolCells: 0, cell: null };

  let alertFromSeq = 0;
  let alertTimer = 0;

  let pendingCmd = null; // { resolve, reject, t0, label }
  let reflex = null; // { kind: "red"|"front", x_cm, y_cm }

  let virtualBlocked = null; // Uint8Array grid.w*grid.h (1 => blocked)
  let redInflate = { xCells: 0, yCells: 0 };
  // Autonomy control flow (with stale live pose):
  // - Always scan BEFORE planning/replanning so pose/heading are fresh.
  // - Never scan just because we turned or the plan changed.
  // - Plan once per iteration, then execute (optional turn + one straight move),
  //   then schedule a scan for the next iteration.
  let pendingScanReason = null; // "segment"|"recover"|"no-path"|...

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

  function vbMarkRect_(grid, cx0, cy0, rxCells, ryCells) {
    if (!virtualBlocked) return;
    const rx = Math.max(0, rxCells | 0);
    const ry = Math.max(0, ryCells | 0);
    for (let dy = -ry; dy <= ry; dy++) {
      const cy = cy0 + dy;
      if (cy < 0 || cy >= grid.h) continue;
      for (let dx = -rx; dx <= rx; dx++) {
        const cx = cx0 + dx;
        if (cx < 0 || cx >= grid.w) continue;
        virtualBlocked[vbIdx(grid, cx, cy)] = 1;
      }
    }
    const api = mappingApi();
    if (api?.setVirtualBlocked) api.setVirtualBlocked(virtualBlocked);
  }

  function isCellBlocked(grid, cx, cy) {
    const st = grid.cellState(cx, cy);
    if (st === "occ") return true;
    if (vbGet(grid, cx, cy)) return true;
    return false;
  }

  const DIRS = [
    { dx: 1, dy: 0, label: "E" },
    { dx: -1, dy: 0, label: "W" },
    { dx: 0, dy: 1, label: "N" },
    { dx: 0, dy: -1, label: "S" },
  ];

  function computeMinTurnsFromStart_(grid, startCell) {
    const w = grid.w;
    const h = grid.h;
    const startIdx = startCell.cy * w + startCell.cx;

    const INF = 1e9;
    const dist = new Int32Array(w * h * 4);
    for (let i = 0; i < dist.length; i++) dist[i] = INF;

    // 0-1 BFS deque as ring buffer of packed state ints: state = (cellIdx<<2)|dirIdx
    // 0-1 BFS can enqueue the same vertex multiple times on relaxations; keep a
    // comfortable margin to avoid ring overflow.
    const q = new Int32Array(w * h * 4 * 16 + 8);
    let head = 0;
    let tail = 0;

    function pushFront(v) {
      head = (head - 1 + q.length) % q.length;
      q[head] = v;
    }
    function pushBack(v) {
      q[tail] = v;
      tail = (tail + 1) % q.length;
    }
    function popFront() {
      const v = q[head];
      head = (head + 1) % q.length;
      return v;
    }
    function isEmpty() {
      return head === tail;
    }

    if (isCellBlocked(grid, startCell.cx, startCell.cy)) return dist;

    // Initialize: first step doesn't count as a "turn". Seed neighbors with 0 turns.
    for (let d = 0; d < 4; d++) {
      const nx = startCell.cx + DIRS[d].dx;
      const ny = startCell.cy + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (isCellBlocked(grid, nx, ny)) continue;
      const nidx = ny * w + nx;
      const sid = (nidx << 2) | d;
      dist[sid] = 0;
      pushBack(sid);
    }

    while (!isEmpty()) {
      const s = popFront();
      const cellIdx = s >> 2;
      const dirPrev = s & 3;
      const baseTurns = dist[s];
      const cx = cellIdx % w;
      const cy = (cellIdx / w) | 0;

      for (let d = 0; d < 4; d++) {
        const nx = cx + DIRS[d].dx;
        const ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (isCellBlocked(grid, nx, ny)) continue;
        const nidx = ny * w + nx;
        const ns = (nidx << 2) | d;
        const wgt = d === dirPrev ? 0 : 1;
        const nd = baseTurns + wgt;
        if (nd < dist[ns]) {
          dist[ns] = nd;
          if (wgt === 0) pushFront(ns);
          else pushBack(ns);
        }
      }
    }

    // States for "start cell with known prev dir" are not used; dist stays INF there.
    // If start == goal, caller should handle that separately.
    return dist;
  }

  function computeMinTurnsToGoal_(grid, goalCell) {
    const w = grid.w;
    const h = grid.h;
    const goalIdx = goalCell.cy * w + goalCell.cx;

    const INF = 1e9;
    const dist = new Int32Array(w * h * 4);
    for (let i = 0; i < dist.length; i++) dist[i] = INF;

    // 0-1 BFS can enqueue the same vertex multiple times on relaxations; keep a
    // comfortable margin to avoid ring overflow.
    const q = new Int32Array(w * h * 4 * 16 + 8);
    let head = 0;
    let tail = 0;

    function pushFront(v) {
      head = (head - 1 + q.length) % q.length;
      q[head] = v;
    }
    function pushBack(v) {
      q[tail] = v;
      tail = (tail + 1) % q.length;
    }
    function popFront() {
      const v = q[head];
      head = (head + 1) % q.length;
      return v;
    }
    function isEmpty() {
      return head === tail;
    }

    if (isCellBlocked(grid, goalCell.cx, goalCell.cy)) return dist;

    // At goal, "already there" costs 0 turns regardless of prev direction.
    for (let d = 0; d < 4; d++) {
      const s = (goalIdx << 2) | d;
      dist[s] = 0;
      pushBack(s);
    }

    while (!isEmpty()) {
      const s = popFront();
      const cellIdx = s >> 2;
      const dirArrived = s & 3; // direction of the forward step used to arrive to this cell
      const baseTurns = dist[s];
      const cx = cellIdx % w;
      const cy = (cellIdx / w) | 0;

      const px = cx - DIRS[dirArrived].dx;
      const py = cy - DIRS[dirArrived].dy;
      if (px < 0 || py < 0 || px >= w || py >= h) continue;
      if (isCellBlocked(grid, px, py)) continue;
      const pidx = py * w + px;

      // In forward space: from (prevDir=p) take step dirArrived to reach (dirArrived).
      for (let prevDir = 0; prevDir < 4; prevDir++) {
        const ps = (pidx << 2) | prevDir;
        const wgt = prevDir === dirArrived ? 0 : 1;
        const nd = baseTurns + wgt;
        if (nd < dist[ps]) {
          dist[ps] = nd;
          if (wgt === 0) pushFront(ps);
          else pushBack(ps);
        }
      }
    }

    return dist;
  }

  function bfsShortestLenWithTurnBudget_(grid, startState, goalCell, turnBudget) {
    // Returns { pathCells, turnsUsed } for shortest-length path that reaches goal
    // with exactly `turnBudget` turns. `startState`: { cx, cy, prevDirIdx }.
    const w = grid.w;
    const h = grid.h;
    const goalIdx = goalCell.cy * w + goalCell.cx;

    const strideT = turnBudget + 1;
    const stateCount = w * h * 4 * strideT;
    const dist = new Int32Array(stateCount);
    dist.fill(-1);
    const prev = new Int32Array(stateCount);
    prev.fill(-1);

    function enc(cellIdx, dirIdx, t) {
      return ((cellIdx << 2) | dirIdx) * strideT + t;
    }

    const startIdx = startState.cy * w + startState.cx;
    if (isCellBlocked(grid, startState.cx, startState.cy)) return null;
    if (isCellBlocked(grid, goalCell.cx, goalCell.cy)) return null;

    const startId = enc(startIdx, startState.prevDirIdx, 0);
    dist[startId] = 0;

    const q = new Int32Array(stateCount + 8);
    let qh = 0;
    let qt = 0;
    q[qt++] = startId;

    let endId = -1;

    while (qh < qt) {
      const curId = q[qh++];
      const curLen = dist[curId];
      const t = curId % strideT;
      const tmp = (curId / strideT) | 0;
      const dirPrev = tmp & 3;
      const cellIdx = tmp >> 2;
      const cx = cellIdx % w;
      const cy = (cellIdx / w) | 0;

      if (cellIdx === goalIdx && t === turnBudget) {
        endId = curId;
        break;
      }

      for (let d = 0; d < 4; d++) {
        const nx = cx + DIRS[d].dx;
        const ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (isCellBlocked(grid, nx, ny)) continue;
        const nidx = ny * w + nx;
        const nt = t + (d === dirPrev ? 0 : 1);
        if (nt > turnBudget) continue;
        const nid = enc(nidx, d, nt);
        if (dist[nid] !== -1) continue;
        dist[nid] = curLen + 1;
        prev[nid] = curId;
        q[qt++] = nid;
      }
    }

    if (endId < 0) return null;

    const cellsRev = [];
    let cur = endId;
    for (;;) {
      const tmp = (cur / strideT) | 0;
      const cellIdx = tmp >> 2;
      const cx = cellIdx % w;
      const cy = (cellIdx / w) | 0;
      cellsRev.push({ cx, cy });
      if (cur === startId) break;
      cur = prev[cur];
      if (cur < 0) return null;
    }
    cellsRev.reverse();
    return cellsRev;
  }

  function planLexicographic_(grid, startCell, goalCell) {
    const w = grid.w;
    const h = grid.h;
    if (!startCell || !goalCell) return null;
    if (isCellBlocked(grid, startCell.cx, startCell.cy)) return null;
    if (isCellBlocked(grid, goalCell.cx, goalCell.cy)) return null;

    if (startCell.cx === goalCell.cx && startCell.cy === goalCell.cy) {
      return [{ cx: startCell.cx, cy: startCell.cy }];
    }

    const distFromStart = computeMinTurnsFromStart_(grid, startCell);
    const goalIdx = goalCell.cy * w + goalCell.cx;
    let minTurnsGoal = 1e9;
    for (let d = 0; d < 4; d++) {
      const sid = (goalIdx << 2) | d;
      const v = distFromStart[sid];
      if (v < minTurnsGoal) minTurnsGoal = v;
    }
    if (!Number.isFinite(minTurnsGoal) || minTurnsGoal >= 1e9) return null;

    const distToGoal = computeMinTurnsToGoal_(grid, goalCell);

    // Secondary objective: maximize the length of the first straight run from the robot.
    const bestRunByDir = new Int32Array(4);
    bestRunByDir.fill(0);
    let bestRun = 0;

    for (let d = 0; d < 4; d++) {
      let cx = startCell.cx;
      let cy = startCell.cy;
      let run = 0;
      for (;;) {
        const nx = cx + DIRS[d].dx;
        const ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) break;
        if (isCellBlocked(grid, nx, ny)) break;
        cx = nx;
        cy = ny;
        run++;

        const idx = (((cy * w + cx) << 2) | d);
        const needTurns = distToGoal[idx];
        if (needTurns <= minTurnsGoal) {
          bestRunByDir[d] = run;
          if (run > bestRun) bestRun = run;
        }
      }
    }

    if (bestRun <= 0) return null;

    let bestPath = null;
    let bestLen = 1e9;

    for (let d = 0; d < 4; d++) {
      if (bestRunByDir[d] !== bestRun) continue;

      // Forced prefix: bestRun steps straight in direction d.
      const prefix = [{ cx: startCell.cx, cy: startCell.cy }];
      let cx = startCell.cx;
      let cy = startCell.cy;
      for (let i = 0; i < bestRun; i++) {
        cx += DIRS[d].dx;
        cy += DIRS[d].dy;
        if (cx < 0 || cy < 0 || cx >= w || cy >= h) {
          cx = NaN;
          break;
        }
        if (isCellBlocked(grid, cx, cy)) {
          cx = NaN;
          break;
        }
        prefix.push({ cx, cy });
      }
      if (!Number.isFinite(cx)) continue;

      const last = prefix[prefix.length - 1];
      const suffix = bfsShortestLenWithTurnBudget_(
        grid,
        { cx: last.cx, cy: last.cy, prevDirIdx: d },
        goalCell,
        minTurnsGoal,
      );
      if (!suffix) continue;

      const full = prefix.concat(suffix.slice(1));
      const len = full.length;
      if (len < bestLen) {
        bestLen = len;
        bestPath = full;
      }
    }

    return bestPath;
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

  function configureRedInflateFromDom_() {
    const canvas = document.getElementById("mapCanvas");
    if (!canvas) return;
    const rx = Number(canvas.dataset.redInflateXCells);
    const ry = Number(canvas.dataset.redInflateYCells);
    redInflate.xCells = Number.isFinite(rx) ? Math.max(0, Math.floor(rx)) : 0;
    redInflate.yCells = Number.isFinite(ry) ? Math.max(0, Math.floor(ry)) : 0;
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

  async function ensureFacingNorth_(api) {
    if (!running) return false;
    const pose = api.getPose();
    const curH = Number(pose?.headingDeg);
    if (!Number.isFinite(curH)) return true;
    const dTurn = wrapDiff180(0 - curH);
    // Small deadband to avoid jitter.
    if (Math.abs(dTurn) <= 3) return true;
    setStatus("turn -> N");
    if (window.StatusBus) window.StatusBus.set("autonomy", { turnDeg: dTurn });
    await cmdTurn(dTurn);
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
    // Reflex stop already forces a scan; drop any queued scan so we don't scan twice.
    pendingScanReason = null;

    if (r.kind === "red") {
      // For red floor tiles: scan first to get the best corrected pose, then mark
      // the virtual obstacle at that corrected pose (avoids odom/pose handoff jitter),
      // then back off. No extra scan after the backoff to avoid scan thrash.
      setStatus("RED stop -> scan");
      await doScan(api, "red");
      const pose = api.getPose();
      const c = api.grid.worldToCell(pose.x, pose.y);
      if (c) {
        const rx = redInflate.xCells | 0;
        const ry = redInflate.yCells | 0;
        if (rx > 0 || ry > 0) vbMarkRect_(api.grid, c.cx, c.cy, rx, ry);
        else vbSet(api.grid, c.cx, c.cy, 1);
      }
      setStatus("RED stop -> backoff");
      await cmdMove(kBackoffAfterReflexCm);
      return true;
    }

    // For front obstacle stops, back off then rescan to re-localize in case of slip.
    setStatus("FRONT stop -> backoff");
    await cmdMove(kBackoffAfterReflexCm);
    await doScan(api, "front");
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
    configureRedInflateFromDom_();

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

    // Project convention: robot heading 0 is aligned with map +Y (north).
    // Always turn to north before the initial scan so the first match is stable.
    await ensureFacingNorth_(api);
    await handleReflex(api);
    if (!running || stopRequested) return;

    const okInit = await ensureInitialLocalization(api);
    if (!okInit) {
      setStatus("init scan failed");
      running = false;
      return;
    }

    while (running && !stopRequested) {
      await handleReflex(api);
      if (!running || stopRequested) break;

      // If a scan is pending, do it as the only primitive for this iteration,
      // then replan on the next iteration.
      if (pendingScanReason) {
        const reason = pendingScanReason;
        pendingScanReason = null;
        await doScan(api, reason);
        continue;
      }

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

      const path = planLexicographic_(api.grid, startCell, goal.cell);
      if (!path) {
        setStatus("NO PATH");
        setPlannedPathOverlay_(api, null);
        if (window.StatusBus) window.StatusBus.set("autonomy", { pathLen: null });
        await doScan(api, "no-path");
        continue;
      }

      setPlannedPathOverlay_(api, path);
      if (window.StatusBus) window.StatusBus.set("autonomy", { pathLen: path.length });

      // Drive along the planned path until the first required turn:
      // take the first segment direction and move as far as possible in that direction.
      const segs = pathToSegments(path);
      if (!segs.length) {
        setStatus("GOAL CELL");
        setPlannedPathOverlay_(api, path);
        break;
      }
      const run = { dir: segs[0].dir, steps: Math.max(1, segs[0].steps | 0) };

      const targetH = dirToHeadingDeg(run.dir);
      const curH = Number(pose.headingDeg);
      const dTurn = wrapDiff180(targetH - curH);
      if (Math.abs(dTurn) > 3) {
        setStatus(`turn -> ${run.dir}`);
        if (window.StatusBus) window.StatusBus.set("autonomy", { segDir: run.dir, turnDeg: dTurn });
        await cmdTurn(dTurn);
      }

      const cmPerCell = api.cfg.cell_cm || api.grid.cell_cm || 5;
      const maxCells = Math.max(1, Math.floor(kMaxSegmentCm / cmPerCell));
      const cellsToMove = Math.max(1, Math.min(run.steps, maxCells));
      const moveCm = cellsToMove * cmPerCell;
      setStatus(`move ${moveCm}cm`);
      if (window.StatusBus) window.StatusBus.set("autonomy", { segDir: run.dir, segCm: moveCm });
      await cmdMove(moveCm);

      // Pose is stale between scans: always scan before the next replan iteration.
      pendingScanReason = "segment";
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
