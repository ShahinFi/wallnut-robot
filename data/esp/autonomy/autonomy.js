// SECTION: Browser-side autonomy for /maze.
// SECTION: Constraints.
// CONTRACT: Mapping remains owned by MappingApi; autonomy only requests scans and motion commands.
// CONTRACT: Scan direction alternation and first-scan '+' behavior are enforced by mapping_boot.js.
// CONTRACT: Reflex events are consumed from `/alerts` and red tiles are handled as virtual obstacles.

function $(id) {
  return document.getElementById(id);
}

const elStart = $("autoStart");
const elStop = $("autoStop");
const elStatus = $("autoStatus");

if (!elStart || !elStop || !elStatus) {
  // WHY: Page doesn't include autonomy UI; safe no-op.
} else {
  // SECTION: Tunables.
  const kCmdTimeoutMs = 25000;

  // WHY: Moderate backoff distance balances clearance recovery and path churn.
  const kBackoffAfterReflexCm = -15;
  // WHY: Segment cap limits drift accumulation between scan-based relocalizations.
  const kMaxSegmentCm = 30;

  const MissionState = Object.freeze({
    Iter: "ITER",
    Scan: "SCAN",
    Plan: "PLAN",
    Turn: "TURN",
    Move: "MOVE",
  });

  // SECTION: Mission runtime flags.
  let running = false;
  let stopRequested = false;

  // SECTION: Goal configuration (source of truth: #mapCanvas data-*).
  let goal = { x_cm: NaN, y_cm: NaN, tolCells: 0, cell: null };

  // SECTION: Alerts stream cursor.
  let alertFromSeq = 0;
  let alertsUnsub = null;

  // SECTION: In-flight command tracking (single owner).
  // CONTRACT: `pendingCmd` is null or `{ resolve, reject, t0, label, startSeq, token }`.
  let pendingCmd = null;
  let cmdTokenSeq_ = 1;

  // SECTION: Mailbox (alerts -> mission state machine).
  // CONTRACT: Mailbox is written by alert polling and consumed only by runLoop state transitions.
  let mailbox = {
    // CONTRACT: `reflex` is null or `{ kind, x_cm, y_cm, headingDeg }`.
    reflex: null,
    lastEvt: null,
    lastAlertSeq: 0,
  };

  // SECTION: Virtual obstacles (red tiles).
  // CONTRACT: `virtualBlocked` is null or Uint8Array sized `grid.w * grid.h`.
  let virtualBlocked = null;
  // WHY: Inflation radii are UI-configured in cell units.
  let redInflate = { xCells: 0, yCells: 0 };
  // CONTRACT: Control loop is scan -> plan -> optional turn -> one move, then rescan before replanning.
  // CONTRACT: `pendingScanReason` is null or a symbolic scan-trigger reason.
  let pendingScanReason = null;

  // SECTION: UI projection (single writer).
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
    // WHY: While a command is in-flight, status prioritizes command label over planner intent.
    const armed = isArmed_();
    const inFlightLabel = pendingCmd && pendingCmd.label ? String(pendingCmd.label) : "";
    const displayState = !armed
      ? "Not armed"
      : (inFlightLabel ? `Executing ${inFlightLabel}` : String(uiState.state || ""));

    lastAutoStatusRaw = displayState;
    refreshAutoStatusDisplay_();

    // CONTRACT: StatusBus autonomy payload is emitted only from this UI writer.
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

  // SECTION: HTTP wrappers.
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
    // CONTRACT: Autonomy must always take the shortest-path turn. Use the dedicated endpoint
    // WHY: that normalizes into [-180, 180] and routes to TurnDegShortest on Mega.
    return await cmdAwait(`/turn_short?deg=${encodeURIComponent(String(v))}`, `turn ${v}deg`);
  }

  // SECTION: Command await (single outstanding command).
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
        // WHY: Ignore stale tick loops from prior commands.
        if (pendingCmd.token !== token) return;
        // CONTRACT: Only update UI while THIS command is still in flight.
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

  // SECTION: Virtual obstacle grid helpers.
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
    // CONTRACT: Hard block check is physical occupancy only; virtual red is handled separately.
    return grid.cellState(cx, cy) === "occ";
  }

  const DIRS = [
    { dx: 1, dy: 0, label: "E" },
    { dx: -1, dy: 0, label: "W" },
    { dx: 0, dy: 1, label: "N" },
    { dx: 0, dy: -1, label: "S" },
  ];

  // SECTION: Planner configuration.
  const plannerCfg_ = {
    // WHY: Path length weight.
    wLen: 1.0,
    // WHY: Heading-change penalty weight.
    wTurn: 3.0,
    // WHY: Proximity-to-obstacle penalty weight.
    wRisk: 6.0,
    // WHY: Reward for preserving first straight run.
    wFirst: 0.0,
    // WHY: Risk radius around virtual red cells (in grid cells).
    rRed: 3,
    // WHY: Risk radius around occupied cells (in grid cells).
    rOcc: 2,
    // CONTRACT: Code-only calibration gains (UI stays as pure intent sliders).
    kLen: 1.00,
    kTurn: 0.35,
    kRisk: 4.00,
    kFirst: 0.25,
    // WHY: Effective calibrated weights (computed from slider * k*).
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

    // WHY: Sliders override dataset defaults when present.
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

    // WHY: Large upper cap allows aggressive detour tuning.
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

  // SECTION: Risk precompute (distance transforms)
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
      // WHY: Non-explosive merge: repulsion depends on nearest strong source, not source count.
      risk[i] = Math.max(rr, ro);
    }

    return risk;
  }

  function headingToDirIdx_(headingDeg) {
    const h = wrapDiff180(Number(headingDeg));
    // CONTRACT: Invalid heading falls back to north-facing index.
    if (!Number.isFinite(h)) return 2;
    // WHY: Convert heading into [0,360) cardinal space.
    let w = h;
    while (w < 0) w += 360;
    while (w >= 360) w -= 360;
    const idx = Math.round(w / 90) % 4;
    // CONTRACT: Cardinal heading index maps to DIRS index order E,W,N,S.
    if (idx === 0) return 2;
    if (idx === 1) return 0;
    if (idx === 2) return 3;
    return 1;
  }

  function dirStepsBetween_(a, b) {
    const da = a | 0;
    const db = b | 0;
    const d = Math.abs(da - db) % 4;
    // CONTRACT: Minimal quarter-turn count is constrained to range [0, 2].
    return d > 2 ? 4 - d : d;
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

    // SECTION: Search state (cell, moveDir, firstDir, leftFirst).
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
      // WHY: cell * 32 + firstDir*8 + moveDir*2 + leftFirst
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
      // WHY: Use a non-negative step floor to keep heuristic admissible with first-run reward.
      const minStep = Math.max(0.001, cfg.wLenEff - cfg.wFirstEff);
      return minStep * (dx + dy);
    }

    const open = new MinHeap_();
    // WHY: Seed: choose firstDir, paying the initial turn from robot heading into that firstDir.
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

        // CONTRACT: First-run reward applies only while still moving along firstDir.
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

    // WHY: Reconstruct state path.
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

    // WHY: Deduplicate consecutive same-cell entries (defensive).
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
      // CONTRACT: Planner path must remain 4-neighbor; otherwise reject path.
      else return [];

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

  // SECTION: UI configuration reads.
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
    // CONTRACT: Alert handling is paused while scan transport is active.
    if (api.state.scanActive) return;

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
        // WHY: Fundamental: ignore stale completions from before this command started.
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

  // SECTION: Alerts stream control.
  async function startAlertStream_() {
    if (alertsUnsub) return true;
    const am = window.AlertsManager;
    if (!am || !am.subscribe) return false;
    // CONTRACT: Do not replay old alert history: start from the newest seq on the ESP.
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
    // WHY: Small deadband to avoid jitter.
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
    // WHY: After a scan match, mapping queues /set_pose to rebase odom->map on Mega.
    // CONTRACT: Do not plan until that pose sync is complete.
    await waitPoseSync_(api);
    return { ok: true, status: "scan_done" };
  }

  async function waitPoseSync_(api) {
    if (!running) return;
    if (!api?.state) return;
    // WHY: Keep status explicit while waiting on pose synchronization.
    setStatus("pose syncing...");
    while (running && !stopRequested && (api.state.posePostPending || api.state.posePostInFlight)) {
      // WHY: Scan already succeeded; this wait is only for alignment posting.
      await new Promise((r) => setTimeout(r, 80));
    }
    // WHY: Caller sets the next actionable status after sync completes.
    if (running && !stopRequested) setStatus("pose synced");
  }

  async function handleReflex(api) {
    if (!mailbox.reflex) return false;
    const r = mailbox.reflex;
    mailbox.reflex = null;
    // WHY: Reflex stop already forces a scan; drop any queued scan so we don't scan twice.
    pendingScanReason = null;

    if (r.kind === "red") {
      // WHY: Red reflex flow is scan -> stamp virtual block -> short backoff.
      setStatus("RED stop -> scan");
      await doScan(api, "red");
      // WHY: Stamp at matched anchor so virtual block aligns with corrected map pose.
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

    // WHY: Front-stop reflex backs off, then rescans to recover localization.
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

    // WHY: Turn north before first scan to stabilize initial matching convention.
    await ensureFacingNorth_(api);
    await handleReflex(api);
    if (!running || stopRequested) return;

    const okInit = await ensureInitialLocalization(api);
    if (!okInit) {
      setStatus("init scan failed");
      running = false;
      return;
    }

    // CONTRACT: Mission state machine has one blocking primitive at a time: scan, command, or plan.
    let state = MissionState.Iter;
    let nextTurnDeg = null;
    let nextMoveCm = null;
    let nextSegDir = null;

    while (running && !stopRequested) {
      // CONTRACT: Sync UI-only fields from mailbox (single writer).
      setUi_({
        lastEvt: mailbox.lastEvt,
        lastAlertSeq: mailbox.lastAlertSeq || 0,
      });

      // WHY: Reflex has priority over everything else.
      if (mailbox.reflex) {
        await handleReflex(api);
        state = MissionState.Iter;
        continue;
      }

      switch (state) {
        case MissionState.Iter: {
          // CONTRACT: Pending scan owns this iteration; planning resumes next iteration.
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
          // WHY: Pose is stale between scans: always scan before the next replan iteration.
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

  // WHY: Initialize planner controls from map-canvas dataset defaults.
  try { syncPlannerInputsFromDataset_(); } catch {}
  try { bindPlannerSliders_(); } catch {}

  setUi_({ running: false, state: "idle" });

  // CONTRACT: Auth status observer updates display text only; it does not alter autonomy behavior.
  try {
    const authEl = document.getElementById("authStatus");
    if (authEl && typeof MutationObserver === "function") {
      const mo = new MutationObserver(() => refreshAutoStatusDisplay_());
      mo.observe(authEl, { subtree: true, characterData: true, childList: true });
    }
  } catch {}
}


