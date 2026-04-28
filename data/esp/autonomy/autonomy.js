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
  // ===== Tunables =====
  const kCmdTimeoutMs = 25000;

  const kBackoffAfterReflexCm = -15; // tunable, "middle" value
  const kMaxSegmentCm = 30; // cap single forward command length to reduce drift

  const MissionState = Object.freeze({
    Iter: "ITER",
    Scan: "SCAN",
    Plan: "PLAN",
    Turn: "TURN",
    Move: "MOVE",
  });

  // ===== Mission runtime flags =====
  let running = false;
  let stopRequested = false;

  // ===== Goal configuration (source of truth: #mapCanvas data-*) =====
  // Single source of truth: read goal from `#mapCanvas` data-*.
  let goal = { x_cm: NaN, y_cm: NaN, tolCells: 0, cell: null };

  // ===== Alerts stream cursor =====
  let alertFromSeq = 0;
  let alertsUnsub = null;

  // ===== In-flight command tracking (single command owner) =====
  let pendingCmd = null; // { resolve, reject, t0, label, startSeq, token }
  let cmdTokenSeq_ = 1;

  // ===== Mailbox (filled by alerts; consumed only by the state machine) =====
  // Mailbox filled by alert polling; consumed only by the mission state machine.
  let mailbox = {
    reflex: null, // { kind: "red"|"front", x_cm, y_cm, headingDeg }
    lastEvt: null,
    lastAlertSeq: 0,
  };

  // ===== Virtual obstacles (red tiles) =====
  let virtualBlocked = null; // Uint8Array grid.w*grid.h (1 => blocked)
  let redInflate = { xCells: 0, yCells: 0 }; // inflation radius in cells (UI-configured)
  // Autonomy control flow (with stale live pose):
  // - Always scan BEFORE planning/replanning so pose/heading are fresh.
  // - Never scan just because we turned or the plan changed.
  // - Plan once per iteration, then execute (optional turn + one straight move),
  //   then schedule a scan for the next iteration.
  let pendingScanReason = null; // "segment"|"recover"|"no-path"|...

  // ===== UI (single writer) =====
  let lastAutoStatusRaw = "idle";

  const uiState = {
    state: "idle",
    running: false,
    lastEvt: null,
    lastAlertSeq: 0,
    goalCx: null,
    goalCy: null,
    startCx: null,
    startCy: null,
    pathLen: null,
    segDir: null,
    segCm: null,
    turnDeg: null,
  };

  function isArmed_() {
    const authEl = document.getElementById("authStatus");
    const authText = authEl ? String(authEl.textContent || "").trim().toUpperCase() : "";
    return authText.startsWith("ARMED");
  }

  function refreshAutoStatusDisplay_() {
    const out = isArmed_() ? String(lastAutoStatusRaw || "") : "Not armed";
    elStatus.textContent = out;
  }

  function renderUi_() {
    // Make the UI 100% consistent/atomic:
    // - While a command is in-flight, the authoritative "what is happening now"
    //   is the in-flight command label (not the planner's next-intent string).
    // - "Next segment" information still shows via segDir/segCm/turnDeg.
    const armed = isArmed_();
    const inFlightLabel = pendingCmd && pendingCmd.label ? String(pendingCmd.label) : "";
    const displayState = !armed
      ? "Not armed"
      : (inFlightLabel ? `Executing ${inFlightLabel}` : String(uiState.state || ""));

    // Auto status pill text.
    lastAutoStatusRaw = displayState;
    refreshAutoStatusDisplay_();

    // Detailed status (single writer).
    if (window.StatusBus) {
      const cmdLabel = pendingCmd && pendingCmd.label ? String(pendingCmd.label) : null;
      const cmdAgeMs = pendingCmd && Number.isFinite(pendingCmd.t0) ? (Date.now() - pendingCmd.t0) : null;
      window.StatusBus.set("autonomy", {
        state: displayState,
        running: !!uiState.running,
        cmdLabel,
        cmdAgeMs,
        lastEvt: uiState.lastEvt,
        lastAlertSeq: uiState.lastAlertSeq,
        goalCx: uiState.goalCx,
        goalCy: uiState.goalCy,
        startCx: uiState.startCx,
        startCy: uiState.startCy,
        pathLen: uiState.pathLen,
        segDir: uiState.segDir,
        segCm: uiState.segCm,
        turnDeg: uiState.turnDeg,
      });
    }
  }

  function setUi_(patch) {
    Object.assign(uiState, patch || {});
    renderUi_();
  }

  function setStatus(text) {
    setUi_({ state: String(text || "") });
  }

  function clearMailbox_() {
    mailbox.reflex = null;
    mailbox.lastEvt = null;
    mailbox.lastAlertSeq = 0;
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

  function currentAlertSeq_() {
    const st = window.TelemetryStore ? window.TelemetryStore.get() : null;
    const storeSeq = Number(st?.alerts?.lastSeq);
    const a = Number.isFinite(storeSeq) ? storeSeq : 0;
    const b = Number.isFinite(alertFromSeq) ? alertFromSeq : 0;
    return Math.max(a, b);
  }

  // ===== HTTP wrappers =====
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
    // Autonomy must always take the shortest-path turn. Use the dedicated endpoint
    // that normalizes into [-180, 180] and routes to TurnDegShortest on Mega.
    return await cmdAwait(`/turn_short?deg=${encodeURIComponent(String(v))}`, `turn ${v}deg`);
  }

  // ===== Command await (single outstanding cmd at a time) =====
  function cmdAwait(path, label) {
    if (!running) return Promise.resolve({ ok: false, status: "stopped" });
    if (pendingCmd) return Promise.reject(new Error("CMD_IN_FLIGHT"));

    return new Promise(async (resolve, reject) => {
      if (window.NetGate) window.NetGate.enterCmd();
      const cmdStartMs = Date.now();
      const startSeq = currentAlertSeq_();
      if (window.DebugLog) window.DebugLog.push("cmd", `begin ${label}`);
      const token = cmdTokenSeq_++;
      pendingCmd = { resolve, reject, t0: Date.now(), label, startSeq, token };
      renderUi_();
      try {
        const res = await httpPost(path);
        if (!res.ok) {
          if (window.DebugLog) window.DebugLog.push("cmd", `http fail ${label} (${res.status})`);
          pendingCmd = null;
          if (window.NetGate) window.NetGate.exitCmd();
          renderUi_();
          resolve({ ok: false, status: `http_${res.status}` });
          return;
        }
      } catch {
        if (window.DebugLog) window.DebugLog.push("cmd", `http error ${label}`);
        pendingCmd = null;
        if (window.NetGate) window.NetGate.exitCmd();
        renderUi_();
        resolve({ ok: false, status: "http_error" });
        return;
      }

      const tick = () => {
        if (!pendingCmd) return;
        // Abort stale tickers from previous commands (prevents UI flicker/backtracking).
        if (pendingCmd.token !== token) return;
        // Only update UI while THIS command is still in flight.
        renderUi_();
        if (!running) {
          const pc = pendingCmd;
          pendingCmd = null;
          if (window.NetGate) window.NetGate.exitCmd();
          renderUi_();
          if (window.DebugLog) window.DebugLog.push("cmd", `end ${label} (stopped)`);
          pc.resolve({ ok: false, status: "stopped" });
          return;
        }
        if (Date.now() - pendingCmd.t0 > kCmdTimeoutMs) {
          const pc = pendingCmd;
          pendingCmd = null;
          if (window.NetGate) window.NetGate.exitCmd();
          renderUi_();
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

  // ===== Virtual obstacle grid helpers =====
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

  function isHardBlocked_(grid, cx, cy) {
    // "Hard" blocks are physical occupied cells (walls/obstacles).
    // Virtual red tiles are treated as blocked for planning *except* we must allow
    // starting inside a red cell (localization jitter) so the robot can escape.
    return grid.cellState(cx, cy) === "occ";
  }

  const DIRS = [
    { dx: 1, dy: 0, label: "E" },
    { dx: -1, dy: 0, label: "W" },
    { dx: 0, dy: 1, label: "N" },
    { dx: 0, dy: -1, label: "S" },
  ];

  // ===== Planner configuration (knobs) =====
  const plannerCfg_ = {
    wLen: 1.0,   // shortest path weight
    wTurn: 3.0,  // turn penalty weight
    wRisk: 6.0,  // obstacle/red proximity penalty weight
    wFirst: 0.0, // prefer longer first straight run from robot pose
    rRed: 3,     // risk radius around virtual red (cells)
    rOcc: 2,     // risk radius around occupied cells (cells)
    // Code-only calibration gains (UI stays as pure intent sliders).
    kLen: 1.00,
    kTurn: 0.35,
    kRisk: 4.00,
    kFirst: 0.25,
    // Effective calibrated weights (computed from slider * k*).
    wLenEff: 1.0,
    wTurnEff: 1.0,
    wRiskEff: 1.0,
    wFirstEff: 0.0,
  };

  function readPlannerConfigFromDom_() {
    const canvas = document.getElementById("mapCanvas");
    if (!canvas) return plannerCfg_;

    function readNum(id, fallback) {
      const el = document.getElementById(id);
      if (!el) return fallback;
      const v = Number(el.value);
      return Number.isFinite(v) ? v : fallback;
    }

    // Allow UI inputs to override, otherwise fall back to data-*.
    const wLen = readNum("planWLen", Number(canvas.dataset.planWLen));
    const wTurn = readNum("planWTurn", Number(canvas.dataset.planWTurn));
    const wRisk = readNum("planWRisk", Number(canvas.dataset.planWRisk));
    const wFirst = readNum("planWFirst", Number(canvas.dataset.planWFirst));

    const rRed = Number(canvas.dataset.planRiskRadiusRedCells);
    const rOcc = Number(canvas.dataset.planRiskRadiusOccCells);

    function clamp01_(v, lo, hi) {
      const x = Number(v);
      if (!Number.isFinite(x)) return null;
      if (x < lo) return lo;
      if (x > hi) return hi;
      return x;
    }

    // Uncapped weights for massive detour scaling
    const wl = clamp01_(wLen, 0, 10000);
    const wt = clamp01_(wTurn, 0, 10000);
    const wr = clamp01_(wRisk, 0, 10000);
    const wf = clamp01_(wFirst, 0, 10000);
    if (wl != null) plannerCfg_.wLen = wl;
    if (wt != null) plannerCfg_.wTurn = wt;
    if (wr != null) plannerCfg_.wRisk = wr;
    if (wf != null) plannerCfg_.wFirst = wf;

    plannerCfg_.rRed = Number.isFinite(rRed) && rRed >= 0 ? Math.floor(rRed) : 3;
    plannerCfg_.rOcc = Number.isFinite(rOcc) && rOcc >= 0 ? Math.floor(rOcc) : 2;
    plannerCfg_.wLenEff = plannerCfg_.wLen * plannerCfg_.kLen;
    plannerCfg_.wTurnEff = plannerCfg_.wTurn * plannerCfg_.kTurn;
    plannerCfg_.wRiskEff = plannerCfg_.wRisk * plannerCfg_.kRisk;
    plannerCfg_.wFirstEff = plannerCfg_.wFirst * plannerCfg_.kFirst;

    return plannerCfg_;
  }

  function syncPlannerInputsFromDataset_() {
    const canvas = document.getElementById("mapCanvas");
    if (!canvas) return;
    const wl = Number(canvas.dataset.planWLen);
    const wt = Number(canvas.dataset.planWTurn);
    const wr = Number(canvas.dataset.planWRisk);
    const wf = Number(canvas.dataset.planWFirst);
    const elWL = document.getElementById("planWLen");
    const elWT = document.getElementById("planWTurn");
    const elWR = document.getElementById("planWRisk");
    const elWF = document.getElementById("planWFirst");
    if (elWL && Number.isFinite(wl)) elWL.value = String(wl);
    if (elWT && Number.isFinite(wt)) elWT.value = String(wt);
    if (elWR && Number.isFinite(wr)) elWR.value = String(wr);
    if (elWF && Number.isFinite(wf)) elWF.value = String(wf);
    updatePlannerSliderValues_();
  }

  function updatePlannerSliderValues_() {
    const keys = ["Len", "Turn", "Risk", "First"];
    for (const k of keys) {
      const input = document.getElementById(`planW${k}`);
      const out = document.getElementById(`planW${k}Val`);
      if (!input || !out) continue;
      const v = Number(input.value);
      out.textContent = Number.isFinite(v) ? v.toFixed(1) : "--";
    }
  }

  function bindPlannerSliders_() {
    const ids = ["planWLen", "planWTurn", "planWRisk", "planWFirst"];
    for (const id of ids) {
      const el = document.getElementById(id);
      if (!el) continue;
      const onChange = () => updatePlannerSliderValues_();
      el.addEventListener("input", onChange);
      el.addEventListener("change", onChange);
    }
    updatePlannerSliderValues_();
  }

  // ===== Risk precompute (distance transforms) =====
  function computeDistToSources_(grid, isSource) {
    const w = grid.w;
    const h = grid.h;
    const n = w * h;
    const dist = new Int16Array(n);
    dist.fill(-1);

    const q = new Int32Array(n + 8);
    let qh = 0;
    let qt = 0;

    for (let cy = 0; cy < h; cy++) {
      for (let cx = 0; cx < w; cx++) {
        const idx = cy * w + cx;
        if (!isSource(cx, cy, idx)) continue;
        dist[idx] = 0;
        q[qt++] = idx;
      }
    }

    while (qh < qt) {
      const cur = q[qh++];
      const cx = cur % w;
      const cy = (cur / w) | 0;
      const base = dist[cur];
      const nextD = base + 1;

      for (let d = 0; d < 4; d++) {
        const nx = cx + DIRS[d].dx;
        const ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (dist[ni] !== -1) continue;
        dist[ni] = nextD;
        q[qt++] = ni;
      }
    }

    return dist;
  }

  function riskFromDist_(d, radiusCells) {
    const r = Number(radiusCells);
    if (!(r > 0)) return 0.0;
    const di = Number(d);
    if (!Number.isFinite(di) || di < 0) return 0.0;
    if (di > r) return 0.0;
    const t = (r - di) / r;
    return t * t;
  }

  function computeRiskMap_(grid, rRedCells, rOccCells) {
    const w = grid.w;
    const h = grid.h;
    const n = w * h;
    const risk = new Float32Array(n);
    risk.fill(0);

    const distRed = computeDistToSources_(grid, (cx, cy, idx) => (virtualBlocked ? !!virtualBlocked[idx] : false));
    const distOcc = computeDistToSources_(grid, (cx, cy) => grid.cellState(cx, cy) === "occ");

    for (let i = 0; i < n; i++) {
      const rr = riskFromDist_(distRed[i], rRedCells);
      const ro = riskFromDist_(distOcc[i], rOccCells);
      // Non-explosive merge: repulsion depends on nearest strong source, not source count.
      risk[i] = Math.max(rr, ro);
    }

    return risk;
  }

  function headingToDirIdx_(headingDeg) {
    const h = wrapDiff180(Number(headingDeg));
    if (!Number.isFinite(h)) return 2; // default N
    // Convert to [0,360)
    let w = h;
    while (w < 0) w += 360;
    while (w >= 360) w -= 360;
    // N=0, E=90, S=180, W=270
    const idx = Math.round(w / 90) % 4;
    // DIRS order is E,W,N,S; map to that:
    // cardinal idx: 0=N,1=E,2=S,3=W
    if (idx === 0) return 2; // N
    if (idx === 1) return 0; // E
    if (idx === 2) return 3; // S
    return 1; // W
  }

  function dirStepsBetween_(a, b) {
    const da = a | 0;
    const db = b | 0;
    const d = Math.abs(da - db) % 4;
    return d > 2 ? 4 - d : d; // 0..2
  }

  class MinHeap_ {
    constructor() { this.ids = []; this.fs = []; }
    get size() { return this.ids.length; }
    push(id, f) {
      const ids = this.ids;
      const fs = this.fs;
      let i = ids.length;
      ids.push(id);
      fs.push(f);
      while (i > 0) {
        const p = ((i - 1) / 2) | 0;
        if (fs[p] <= f) break;
        ids[i] = ids[p]; fs[i] = fs[p];
        i = p;
      }
      ids[i] = id; fs[i] = f;
    }
    pop() {
      const ids = this.ids;
      const fs = this.fs;
      const n = ids.length;
      if (n === 0) return null;
      const outId = ids[0];
      const outF = fs[0];
      const lastId = ids.pop();
      const lastF = fs.pop();
      if (n > 1) {
        let i = 0;
        while (true) {
          const l = i * 2 + 1;
          const r = l + 1;
          if (l >= ids.length) break;
          let s = l;
          if (r < ids.length && fs[r] < fs[l]) s = r;
          if (fs[s] >= lastF) break;
          ids[i] = ids[s]; fs[i] = fs[s];
          i = s;
        }
        ids[i] = lastId; fs[i] = lastF;
      }
      return { id: outId, f: outF };
    }
  }

  function planWeightedAStar_(grid, startCell, goalCell, robotDirIdx, cfg, riskMap) {
    const w = grid.w;
    const h = grid.h;
    if (!startCell || !goalCell) return null;
    if (isHardBlocked_(grid, startCell.cx, startCell.cy)) return null;
    if (isCellBlocked(grid, goalCell.cx, goalCell.cy)) return null;
    if (startCell.cx === goalCell.cx && startCell.cy === goalCell.cy) {
      return [{ cx: startCell.cx, cy: startCell.cy }];
    }

    // State: (cell, moveDir, firstDir, leftFirst)
    // - moveDir is the direction of the last step (used for turn cost).
    // - firstDir is the direction of the first straight run we want to maximize.
    // - leftFirst is true once we take a step not equal to firstDir.
    const nStates = w * h * 4 * 4 * 2;
    const gScore = new Float64Array(nStates);
    const came = new Int32Array(nStates);
    gScore.fill(Infinity);
    came.fill(-1);

    function stateId(cx, cy, moveDir, firstDir, leftFirst) {
      const cell = (cy * w + cx) >>> 0;
      const md = (moveDir & 3) >>> 0;
      const fd = (firstDir & 3) >>> 0;
      const lf = leftFirst ? 1 : 0;
      // cell * 32 + firstDir*8 + moveDir*2 + leftFirst
      return ((cell * 32) + (fd * 8) + (md * 2) + lf) >>> 0;
    }

    function decodeState(id) {
      const lf = id & 1;
      const md = (id >>> 1) & 3;
      const fd = (id >>> 3) & 3;
      const cell = (id / 32) | 0;
      const cx = cell % w;
      const cy = (cell / w) | 0;
      return { cx, cy, moveDir: md, firstDir: fd, leftFirst: !!lf };
    }

    function heuristic(cx, cy) {
      const dx = Math.abs(cx - goalCell.cx);
      const dy = Math.abs(cy - goalCell.cy);
      // Keep heuristic admissible with first-run reward: use the smallest guaranteed
      // non-negative per-step floor.
      const minStep = Math.max(0.001, cfg.wLenEff - cfg.wFirstEff);
      return minStep * (dx + dy);
    }

    const open = new MinHeap_();
    // Seed: choose firstDir, paying the initial turn from robot heading into that firstDir.
    for (let firstDir = 0; firstDir < 4; firstDir++) {
      const initialTurnSteps = dirStepsBetween_(robotDirIdx, firstDir);
      const initCost = cfg.wTurnEff * initialTurnSteps;
      const sid = stateId(startCell.cx, startCell.cy, firstDir, firstDir, false);
      if (initCost < gScore[sid]) {
        gScore[sid] = initCost;
        open.push(sid, initCost + heuristic(startCell.cx, startCell.cy));
      }
    }

    const goalIdx = goalCell.cy * w + goalCell.cx;
    let goalState = -1;

    while (open.size) {
      const cur = open.pop();
      if (!cur) break;
      const id = cur.id;
      const baseG = gScore[id];
      if (!Number.isFinite(baseG)) continue;

      const st = decodeState(id);
      const cx = st.cx;
      const cy = st.cy;

      const cellIdx = cy * w + cx;
      if (cellIdx === goalIdx) {
        goalState = id;
        break;
      }

      for (let nd = 0; nd < 4; nd++) {
        const nx = cx + DIRS[nd].dx;
        const ny = cy + DIRS[nd].dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (isCellBlocked(grid, nx, ny)) continue;

        const nCellIdx = ny * w + nx;
        const nextLeft = st.leftFirst || (nd !== st.firstDir);
        const nId = stateId(nx, ny, nd, st.firstDir, nextLeft);

        const turnSteps = dirStepsBetween_(st.moveDir, nd);
        const riskCost = cfg.wRiskEff * (riskMap ? riskMap[nCellIdx] : 0.0);

        // First-run preference: reward staying on the initial direction from the
        // robot side. This is applied only while we are still on the first run.
        const onFirstRun = !nextLeft;
        const baseStepCost = cfg.wLenEff + (cfg.wTurnEff * turnSteps) + riskCost;
        const firstRunReward = onFirstRun ? Math.min(cfg.wFirstEff, baseStepCost * 0.8) : 0.0;
        const stepCost = baseStepCost - firstRunReward;
        const ng = baseG + stepCost;
        if (ng >= gScore[nId]) continue;

        gScore[nId] = ng;
        came[nId] = id;
        const nf = ng + heuristic(nx, ny);
        open.push(nId, nf);
      }
    }

    if (goalState < 0) return null;

    // Reconstruct state path.
    const revCells = [];
    let curId = goalState;
    while (curId >= 0) {
      const st = decodeState(curId);
      revCells.push({ cx: st.cx, cy: st.cy });
      const p = came[curId];
      if (p < 0) break;
      curId = p;
      if (revCells.length > w * h + 16) break;
    }
    revCells.reverse();

    // Deduplicate consecutive same-cell entries (defensive).
    const out = [];
    for (const c of revCells) {
      const prev = out.length ? out[out.length - 1] : null;
      if (prev && prev.cx === c.cx && prev.cy === c.cy) continue;
      out.push(c);
    }
    return out;
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

  // ===== UI configuration reads =====
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

  function handleAlertRow_(row) {
    const api = mappingApi();
    if (!api) return;
    if (!running) return;
    if (api.state.scanActive) return; // preserve scan bandwidth

    const seq = Number(row && row.seq);
    const line = String(row && row.line ? row.line : "").trim();
    if (!line) return;
    if (Number.isFinite(seq)) alertFromSeq = Math.max(alertFromSeq, seq);

    const evt = parseEvt(line);
    if (!evt) return;
    if (window.DebugLog) window.DebugLog.push("evt", line.trim());

    mailbox.lastEvt = evt.tag;
    mailbox.lastAlertSeq = alertFromSeq;

    if (evt.tag === "RED") {
      mailbox.reflex = { kind: "red", x_cm: evt.x_cm, y_cm: evt.y_cm, headingDeg: evt.headingDeg };
    } else if (evt.tag === "FRONTSTOP") {
      mailbox.reflex = { kind: "front", x_cm: evt.x_cm, y_cm: evt.y_cm, headingDeg: evt.headingDeg };
    } else if (evt.tag === "CMDOK" || evt.tag === "CMDFAIL" || evt.tag === "CMDCANCEL") {
      if (pendingCmd) {
        // Fundamental: ignore stale completions from before this command started.
        if (Number.isFinite(seq) && seq <= pendingCmd.startSeq) return;
        const pc = pendingCmd;
        pendingCmd = null;
        if (window.NetGate) window.NetGate.exitCmd();
        renderUi_();
        const ok = evt.tag === "CMDOK";
        if (window.DebugLog) window.DebugLog.push("cmd", `end ${pc.label} (${evt.tag})`);
        pc.resolve({ ok, status: evt.tag, pose: { x_cm: evt.x_cm, y_cm: evt.y_cm, h: evt.headingDeg } });
      }
    }
  }

  // ===== Alerts stream control =====
  async function startAlertStream_() {
    if (alertsUnsub) return true;
    const am = window.AlertsManager;
    if (!am || !am.subscribe) return false;
    // Do not replay old alert history: start from the newest seq on the ESP.
    let tail = 0;
    try {
      const res = await fetch("/alerts_tail", { cache: "no-store" });
      if (res && res.ok) {
        const t = Number(String(await res.text()).trim());
        if (Number.isFinite(t) && t >= 0) tail = Math.floor(t);
      }
    } catch {}
    try { am.setCursor && am.setCursor(Math.max(tail, currentAlertSeq_())); } catch {}
    alertsUnsub = am.subscribe(handleAlertRow_);
    try { am.start && am.start(); } catch {}
    return true;
  }

  function stopAlertStream_() {
    if (alertsUnsub) {
      try { alertsUnsub(); } catch {}
      alertsUnsub = null;
    }
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
    setUi_({ segDir: "N", turnDeg: dTurn, segCm: null });
    await cmdTurn(dTurn);
    return true;
  }

  async function doScan(api, reason) {
    if (!running) return { ok: false, status: "stopped" };
    setStatus(`scan (${reason})...`);
    const r = await api.requestScanAuto();
    if (!r?.ok) return { ok: false, status: r?.kind || "scan_fail" };
    // After a scan match, mapping queues /set_pose to rebase odom->map on Mega.
    // Do not plan until that pose sync is complete.
    await waitPoseSync_(api);
    return { ok: true, status: "scan_done" };
  }

  async function waitPoseSync_(api) {
    if (!running) return;
    if (!api?.state) return;
    // Show a clear state while waiting.
    setStatus("pose syncing...");
    while (running && !stopRequested && (api.state.posePostPending || api.state.posePostInFlight)) {
      // If something is wrong, keep waiting (user asked for guaranteed sync).
      // The scan already succeeded; this is just alignment posting.
      await new Promise((r) => setTimeout(r, 80));
    }
    // Restore to a neutral state; caller will set the next status (plan/turn/move).
    if (running && !stopRequested) setStatus("pose synced");
  }

  async function handleReflex(api) {
    if (!mailbox.reflex) return false;
    const r = mailbox.reflex;
    mailbox.reflex = null;
    // Reflex stop already forces a scan; drop any queued scan so we don't scan twice.
    pendingScanReason = null;

    if (r.kind === "red") {
      // For red floor tiles: scan first to get the corrected pose, then mark the
      // virtual obstacle at that corrected pose, then back off. No scan is needed
      // after the backoff; we trust odometry for this short reverse move.
      setStatus("RED stop -> scan");
      await doScan(api, "red");
      // Stamp at the scan-matched pose anchor (mapPose0) so the mark aligns with
      // the scan correction (and thus with the map update).
      const p0 = api?.state?.mapPose0;
      const c = api.grid.worldToCell(p0?.x, p0?.y);
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

    setUi_({
      running: true,
      state: `goal cell=(${goal.cell.cx},${goal.cell.cy})`,
      goalCx: goal.cell.cx,
      goalCy: goal.cell.cy,
      pathLen: null,
      segDir: null,
      segCm: null,
      turnDeg: null,
      lastEvt: null,
      lastAlertSeq: mailbox.lastAlertSeq || 0,
    });

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

    // Mission state machine. The invariant:
    // At any time, autonomy is doing exactly one blocking wait:
    // scan OR a single motion command OR planning.
    let state = MissionState.Iter;
    let nextTurnDeg = null;
    let nextMoveCm = null;
    let nextSegDir = null;

    while (running && !stopRequested) {
      // Sync UI-only fields from mailbox (single writer).
      setUi_({
        lastEvt: mailbox.lastEvt,
        lastAlertSeq: mailbox.lastAlertSeq || 0,
      });

      // Reflex has priority over everything else.
      if (mailbox.reflex) {
        await handleReflex(api);
        state = MissionState.Iter;
        continue;
      }

      switch (state) {
        case MissionState.Iter: {
          // If a scan is pending, do it as the only primitive for this iteration,
          // then plan on the next iteration.
          state = pendingScanReason ? MissionState.Scan : MissionState.Plan;
          break;
        }

        case MissionState.Scan: {
          const reason = pendingScanReason || "segment";
          pendingScanReason = null;
          await doScan(api, reason);
          state = MissionState.Plan;
          break;
        }

        case MissionState.Plan: {
          const pose = api.getPose();
          if (isAtGoal(api, pose)) {
            setStatus("GOAL REACHED");
            stopRequested = true;
            break;
          }

        const startCell = api.grid.worldToCell(pose.x, pose.y);
        if (!startCell) {
          pendingScanReason = "recover";
          state = MissionState.Scan;
          break;
        }
        setUi_({ startCx: startCell.cx, startCy: startCell.cy });

        const cfg = readPlannerConfigFromDom_();
        const riskMap = computeRiskMap_(api.grid, cfg.rRed, cfg.rOcc);
        const robotDirIdx = headingToDirIdx_(pose.headingDeg);
        const path = planWeightedAStar_(api.grid, startCell, goal.cell, robotDirIdx, cfg, riskMap);
        if (!path) {
          setStatus("NO PATH");
          setPlannedPathOverlay_(api, null);
          setUi_({ pathLen: null, segDir: null, segCm: null, turnDeg: null });
          pendingScanReason = "no-path";
            state = MissionState.Scan;
            break;
          }

          setPlannedPathOverlay_(api, path);
          setUi_({ pathLen: path.length });

          const segs = pathToSegments(path);
          if (!segs.length) {
            setStatus("GOAL CELL");
            stopRequested = true;
            break;
          }

          const run = { dir: segs[0].dir, steps: Math.max(1, segs[0].steps | 0) };
          const targetH = dirToHeadingDeg(run.dir);
          const curH = Number(pose.headingDeg);
          const dTurn = wrapDiff180(targetH - curH);
          nextSegDir = run.dir;
          nextTurnDeg = Math.abs(dTurn) > 3 ? dTurn : null;

          const cmPerCell = api.cfg.cell_cm || api.grid.cell_cm || 5;
          const maxCells = Math.max(1, Math.floor(kMaxSegmentCm / cmPerCell));
          const cellsToMove = Math.max(1, Math.min(run.steps, maxCells));
          nextMoveCm = cellsToMove * cmPerCell;

          state = nextTurnDeg !== null ? MissionState.Turn : MissionState.Move;
          break;
        }

        case MissionState.Turn: {
          setStatus(`turn -> ${nextSegDir}`);
          setUi_({ segDir: nextSegDir, turnDeg: nextTurnDeg, segCm: null });
          await cmdTurn(nextTurnDeg);
          state = MissionState.Move;
          break;
        }

        case MissionState.Move: {
          setStatus(`move ${nextMoveCm}cm`);
          setUi_({ segDir: nextSegDir, segCm: nextMoveCm, turnDeg: null });
          await cmdMove(nextMoveCm);
          // Pose is stale between scans: always scan before the next replan iteration.
          pendingScanReason = "segment";
          state = MissionState.Iter;
          break;
        }

        default: {
          state = MissionState.Iter;
          break;
        }
      }
    }

    running = false;
    stopRequested = false;
    stopAlertStream_();
    if (api?.setPlannedPath) api.setPlannedPath([]);
    if (pendingCmd) {
      const pc = pendingCmd;
      pendingCmd = null;
      if (window.NetGate) window.NetGate.exitCmd();
      renderUi_();
      pc.resolve({ ok: false, status: "stopped" });
    }
    setUi_({ running: false });
    if (elStart) elStart.disabled = false;
    setStatus("idle");
    try { window.CommsOrchestrator && window.CommsOrchestrator.setAutonomyRunning(false); } catch {}
  }

  elStart.addEventListener("click", async () => {
    if (running) return;
    const api = mappingApi();
    if (!api) {
      setStatus("MappingApi missing");
      return;
    }
    running = true;
    stopRequested = false;
    alertFromSeq = 0;
    clearMailbox_();

    try {
      await startAlertStream_();
    } catch {}
    try { window.CommsOrchestrator && window.CommsOrchestrator.setAutonomyRunning(true); } catch {}
    elStart.disabled = true;
    runLoop().catch(() => {
      running = false;
      stopRequested = false;
      stopAlertStream_();
      if (elStart) elStart.disabled = false;
      setStatus("AUTO ERROR");
    });
  });

  elStop.addEventListener("click", () => {
    stopRequested = true;
    setStatus("stopping...");
  });

  // Initialize planner knobs from the map canvas data-* defaults (so UI reflects
  // the current configured weights on load).
  try { syncPlannerInputsFromDataset_(); } catch {}
  try { bindPlannerSliders_(); } catch {}

  setUi_({ running: false, state: "idle" });

  // If auth state flips (DISARMED -> ARMED), refresh display-only status text
  // without changing any autonomy behavior.
  try {
    const authEl = document.getElementById("authStatus");
    if (authEl && typeof MutationObserver === "function") {
      const mo = new MutationObserver(() => refreshAutoStatusDisplay_());
      mo.observe(authEl, { subtree: true, characterData: true, childList: true });
    }
  } catch {}
}
