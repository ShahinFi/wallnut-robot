// Debug/status bus for the /maze UI.
// Pure UI/diagnostics: must not change robot behavior.
//
// Exposes `window.StatusBus` and renders a structured status panel if an element
// with id `debugStatus` exists.

(function initStatusBus() {
  const state = {
    startedMs: Date.now(),
    mapping: {},
    autonomy: {},
    telemetry: {},
  };

  function shallowMerge(dst, src) {
    if (!src) return dst;
    for (const k of Object.keys(src)) dst[k] = src[k];
    return dst;
  }

  function set(ns, patch) {
    const key = String(ns || "");
    if (!Object.prototype.hasOwnProperty.call(state, key)) return;
    if (patch && typeof patch === "object") shallowMerge(state[key], patch);
  }

  function snap() {
    return {
      up_s: Math.round((Date.now() - state.startedMs) / 100) / 10,
      net: window.NetGate ? String(window.NetGate.mode()) : "none",
      mapping: { ...state.mapping },
      autonomy: { ...state.autonomy },
      telemetry: { ...state.telemetry },
    };
  }

  function fmtNum(n, d = 1) {
    const v = Number(n);
    if (!Number.isFinite(v)) return "--";
    return v.toFixed(d);
  }

  function fmtInt(n) {
    const v = Number(n);
    if (!Number.isFinite(v)) return "--";
    return String(Math.trunc(v));
  }

  function render() {
    const el = document.getElementById("debugStatus");
    if (!el) return;
    const s = snap();
    const m = s.mapping || {};
    const a = s.autonomy || {};

    const lines = [];
    lines.push(`net=${s.net} up=${s.up_s}s`);

    // Pose
    if (m.poseX != null || m.poseY != null || m.poseH != null) {
      lines.push(
        `pose x=${fmtNum(m.poseX, 1)} y=${fmtNum(m.poseY, 1)} h=${fmtNum(m.poseH, 1)} cell=(${fmtInt(m.cellCx)},${fmtInt(m.cellCy)})`
      );
    }

    // Mapping scan
    lines.push(
      `scan active=${m.scanActive ? "1" : "0"} dir=${m.scanDir || "--"} next=${m.nextScanDir || "--"} poll=${m.scanPollActive ? "1" : "0"} seq=${fmtInt(m.scanSeq)} evt=${m.scanLastEvt || "--"}`
    );

    // Autonomy
    lines.push(
      `auto running=${a.running ? "1" : "0"} state=${a.state || "--"} start=(${fmtInt(a.startCx)},${fmtInt(a.startCy)}) goal=(${fmtInt(a.goalCx)},${fmtInt(a.goalCy)}) pathLen=${fmtInt(a.pathLen)}`
    );
    lines.push(
      `cmd inFlight=${a.cmdLabel || "none"} ageMs=${fmtInt(a.cmdAgeMs)} lastEvt=${a.lastEvt || "--"} alertSeq=${fmtInt(a.lastAlertSeq)}`
    );
    if (a.segDir || a.segCm) {
      lines.push(`seg dir=${a.segDir || "--"} moveCm=${fmtInt(a.segCm)} turnDeg=${fmtInt(a.turnDeg)}`);
    }

    el.textContent = lines.join("\n");
  }

  window.StatusBus = { set, snap };

  // Render loop (UI-only). Keep it light.
  setInterval(render, 200);
  render();
})();

