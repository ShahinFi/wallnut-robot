// SECTION: /maze debug status bus.
// CONTRACT: UI diagnostics only; no behavior side effects.

(function initStatusBus() {
  // SECTION: Local diagnostic state cache.
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
    if (n == null) return "--";
    const v = Number(n);
    if (!Number.isFinite(v)) return "--";
    return String(Math.trunc(v));
  }

  function fmtSec(ms) {
    const v = Number(ms);
    if (!Number.isFinite(v) || v < 0) return "--";
    const s = v / 1000;
    return s >= 10 ? `${s.toFixed(0)}s` : `${s.toFixed(1)}s`;
  }

  function fmtBool01(v) {
    return v ? "ON" : "OFF";
  }

  function fmtEvt(tag) {
    const t = String(tag || "").trim();
    if (!t) return "--";
    const u = t.toUpperCase();
    if (u === "BEGIN") return "Begin";
    if (u === "START") return "Start";
    if (u === "SAMPLE") return "Sample";
    if (u === "DONE") return "Done";
    if (u === "CANCEL") return "Cancel";
    if (u === "TIMEOUT") return "Timeout";
    if (u === "OVERFLOW") return "Overflow";
    if (u === "403") return "Not Armed";
    return t;
  }

  function fmtNetMode(m) {
    const t = String(m || "").trim().toLowerCase();
    if (t === "scan") return "Scanning";
    if (t === "cmd") return "Command";
    if (t === "idle") return "Idle";
    if (t === "none") return "NoGate";
    return t || "--";
  }

  function render() {
    // SECTION: Status panel projection.
    const el = document.getElementById("debugStatus");
    if (!el) return;

    // CONTRACT: Hide live status when not armed; display-only policy.
    const authEl = document.getElementById("authStatus");
    const authText = authEl ? String(authEl.textContent || "").trim().toUpperCase() : "";
    if (!authText.startsWith("ARMED")) {
      el.textContent = "";
      return;
    }

    const s = snap();
    const m = s.mapping || {};
    const a = s.autonomy || {};

    const lines = [];

    // SECTION: Mode and uptime.
    lines.push(`MODE: ${fmtNetMode(s.net)} | up ${fmtNum(s.up_s, 1)}s`);

    // SECTION: Pose summary.
    const hasPose = (m.poseX != null || m.poseY != null || m.poseH != null);
    if (hasPose) {
      lines.push(
        `POSE: E=${fmtNum(m.poseX, 1)} N=${fmtNum(m.poseY, 1)} H=${fmtNum(m.poseH, 0)}° | cell (${fmtInt(m.cellCx)},${fmtInt(m.cellCy)})`
      );
    }

    // SECTION: Scan summary.
    const scanActive = !!m.scanActive;
    const scanParts = [];
    scanParts.push(`SCAN: ${scanActive ? "ACTIVE" : "idle"}`);
    if (scanActive) scanParts.push(`dir ${m.scanDir || "--"}`);
    scanParts.push(`next ${m.nextScanDir || "--"}`);
    scanParts.push(`evt ${fmtEvt(m.scanLastEvt)}`);
    const poseSyncOn = !!(m.posePostPending || m.posePostInFlight);
    scanParts.push(`pose ${poseSyncOn ? "SYNC" : "ok"}`);
    if (scanActive) scanParts.push(`poll ${m.scanPollActive ? "ON" : "OFF"}`);
    if (scanActive) scanParts.push(`seq ${fmtInt(m.scanSeq)}`);
    lines.push(scanParts.join(" | "));

    // SECTION: Autonomy and command summary.
    const autoOn = !!a.running;
    if (!autoOn) {
      lines.push(`AUTO: OFF`);
    } else {
      const autoParts = [];
      autoParts.push(`AUTO: ON`);
      if (a.state) autoParts.push(String(a.state));
      autoParts.push(`at (${fmtInt(a.startCx)},${fmtInt(a.startCy)})`);
      autoParts.push(`goal (${fmtInt(a.goalCx)},${fmtInt(a.goalCy)})`);
      if (a.pathLen != null) autoParts.push(`path ${fmtInt(a.pathLen)}`);

      const cmdParts = [];
      cmdParts.push(`CMD ${a.cmdLabel || "none"}`);
      if (a.cmdLabel) cmdParts.push(`age ${fmtSec(a.cmdAgeMs)}`);
      if (a.lastEvt) cmdParts.push(`last ${fmtEvt(a.lastEvt)}`);

      const segParts = [];
      if (a.segDir) segParts.push(`next ${a.segDir}`);
      if (a.segCm != null) segParts.push(`${fmtInt(a.segCm)}cm`);
      if (a.turnDeg != null && Number.isFinite(Number(a.turnDeg))) segParts.push(`turn ${fmtInt(a.turnDeg)}°`);

      const tail = [];
      tail.push(cmdParts.join(" | "));
      if (segParts.length) tail.push(segParts.join(" "));

      lines.push(autoParts.join(" | "));
      lines.push(tail.join(" | "));
    }

    // WHY: Keep the panel compact and scannable.
    el.textContent = lines.slice(0, 5).join("\n");
  }

  window.StatusBus = { set, snap };

  // CONTRACT: Lightweight UI-only refresh loop.
  setInterval(render, 200);
  render();
})();
