// SECTION: Shared network polling gate.
// CONTRACT: Priority order is scan > cmd > idle.
// WHY: This is a classic script so module and non-module pages share one gate.

(function initNetGate() {
  const state = { scan: 0, cmd: 0 };

  // SECTION: Gate state and mode projection.
  function clampNonNeg(n) {
    return n < 0 ? 0 : n;
  }

  function mode() {
    if (state.scan > 0) return "scan";
    if (state.cmd > 0) return "cmd";
    return "idle";
  }

  function allow(kind) {
    // CONTRACT: Gate decisions depend only on current mode and caller kind.
    const m = mode();
    const k = String(kind || "");
    if (m === "scan") {
      // CONTRACT: During scan, only scan transport is permitted.
      return k === "scan";
    }
    if (m === "cmd") {
      // WHY: Keep command completion and pose responsive while reducing telemetry load.
      return k === "alerts" || k === "cmd" || k === "pose" || k === "debug";
    }
    // CONTRACT: Idle mode allows all pollers.
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

