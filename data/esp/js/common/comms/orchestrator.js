// WHY: Top-level comms orchestrator (classic script).
// WHY: Controls TelemetryManager + AlertsManager rates based on NetGate mode and whether autonomy is running.

(function initCommsOrchestrator() {
  if (window.CommsOrchestrator) return;

  const tm = window.TelemetryManager;
  const am = window.AlertsManager || null;
  const store = window.TelemetryStore;
  if (!tm || !store) {
    console.warn("CommsOrchestrator missing dependency");
    return;
  }

  // SECTION: Polling period tunables (ms).
  const cfg = {
    // WHY: Idle telemetry runs near 4 Hz for smooth UI updates.
    telemetryIdleMs: 250,
    // WHY: Command mode keeps telemetry alive while awaiting command completion.
    telemetryCmdMs: 500,
    // WHY: Scan mode keeps telemetry nearly paused to preserve scan bandwidth.
    telemetryScanMs: 2000,
    // WHY: Idle alerts run at a low rate when autonomy is active.
    alertsIdleMs: 500,
    // WHY: Command mode alerts run fast for reflex and command completion events.
    alertsCmdMs: 140,
    alertsOffMs: 9999999,
  };

  let running = true;
  let autonomyRunning = false;
  let timer = 0;
  let lastKey = "";
  let lastArmed = false;

  // SECTION: Policy projection helpers.
  function netMode_() {
    const g = window.NetGate;
    return g ? g.mode() : "idle";
  }

  function apply_() {
    // CONTRACT: Same policy key should not re-apply timers or poll rates.
    const mode = netMode_();
    const st0 = store.get ? store.get() : null;
    const auth0 = String(st0 && st0.auth && st0.auth.state ? st0.auth.state : "DISARMED").toUpperCase();
    const key = `${mode}|${auth0}|${autonomyRunning ? 1 : 0}`;
    if (key === lastKey && timer) return;
    lastKey = key;

    if (!running) {
      tm.stop();
      if (am) am.stop();
      return;
    }

    const armed = auth0 === "ARMED";
    if (!armed) {
      // CONTRACT: Leaving ARMED state clears telemetry once to avoid stale control data.
      if (lastArmed) {
        if (typeof store.clearTelemetry === "function") store.clearTelemetry();
      }
      lastArmed = false;
      tm.stop();
      if (am) am.stop();
      return;
    }
    lastArmed = true;

    if (mode === "scan") {
      tm.setPeriod(cfg.telemetryScanMs);
      tm.start();
      // CONTRACT: Scan mode pauses alerts to preserve scan bandwidth.
      if (am) am.stop();
      return;
    }

    if (mode === "cmd") {
      tm.setPeriod(cfg.telemetryCmdMs);
      tm.start();
      if (am) {
        am.setPeriod(cfg.alertsCmdMs);
        am.start();
      }
      return;
    }

    // SECTION: Idle mode policy.
    tm.setPeriod(cfg.telemetryIdleMs);
    tm.start();
    if (autonomyRunning) {
      if (am) {
        am.setPeriod(cfg.alertsIdleMs);
        am.start();
      }
    } else {
      if (am) am.stop();
    }
  }

  function tick_() {
    apply_();
    timer = setTimeout(tick_, 250);
  }

  function start() {
    running = true;
    if (timer) return;
    tick_();
  }

  function stop() {
    running = false;
    if (timer) clearTimeout(timer);
    timer = 0;
    tm.stop();
    if (am) am.stop();
  }

  function setAutonomyRunning(v) {
    autonomyRunning = !!v;
    apply_();
  }

  window.CommsOrchestrator = { start, stop, setAutonomyRunning, _cfg: cfg, _mode: netMode_ };
})();

