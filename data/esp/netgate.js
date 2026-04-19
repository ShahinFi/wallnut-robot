// Shared network polling gate (single source of truth for bandwidth priority).
//
// Modes (priority order):
// - scan: mapping scan transaction (/events) in progress => pause everything else
// - cmd : waiting for motion completion events (/alerts) => pause nonessential polls
// - idle: normal polling
//
// This file is intentionally a classic script (not a module) so both module and
// non-module scripts can access `window.NetGate`.

(function initNetGate() {
  const state = { scan: 0, cmd: 0 };

  function clampNonNeg(n) {
    return n < 0 ? 0 : n;
  }

  function mode() {
    if (state.scan > 0) return "scan";
    if (state.cmd > 0) return "cmd";
    return "idle";
  }

  function allow(kind) {
    const m = mode();
    const k = String(kind || "");
    if (m === "scan") {
      // Only scan transport should run.
      return k === "scan";
    }
    if (m === "cmd") {
      // While waiting for command completion, keep alerts responsive and allow
      // low-rate pose updates (arrow) so UI stays live. Pause heavy telemetry.
      return k === "alerts" || k === "cmd" || k === "pose" || k === "debug";
    }
    // idle: everything allowed
    return true;
  }

  function enter(kind) {
    if (kind === "scan") state.scan++;
    else if (kind === "cmd") state.cmd++;
  }

  function exit(kind) {
    if (kind === "scan") state.scan = clampNonNeg(state.scan - 1);
    else if (kind === "cmd") state.cmd = clampNonNeg(state.cmd - 1);
  }

  window.NetGate = {
    mode,
    allow,
    enterScan: () => enter("scan"),
    exitScan: () => exit("scan"),
    enterCmd: () => enter("cmd"),
    exitCmd: () => exit("cmd"),
    _state: state,
  };
})();

