// Top-level comms orchestrator (classic script).
// Controls TelemetryManager + AlertsManager rates based on NetGate mode and whether autonomy is running.

(function initCommsOrchestrator() {
  if (window.CommsOrchestrator) return;

  const tm = window.TelemetryManager;
  const am = window.AlertsManager || null;
  const store = window.TelemetryStore;
  if (!tm || !store) {
    console.warn("CommsOrchestrator missing dependency");
    return;
  }

  // Tunables (ms). Keep them conservative and stable.
  const cfg = {
    telemetryIdleMs: 250,   // ~4 Hz
    telemetryCmdMs: 500,    // keep UI alive while waiting for CMDOK
    telemetryScanMs: 2000,  // effectively paused (still alive if scan is long)
    alertsIdleMs: 500,      // low-rate when running
    alertsCmdMs: 140,       // responsive for reflex + cmd completion
    alertsOffMs: 9999999,
  };

  let running = true;
  let autonomyRunning = false;
  let timer = 0;
  let lastKey = "";
  let lastArmed = false;

  function netMode_() {
    const g = window.NetGate;
    return g ? g.mode() : "idle";
  }

  function apply_() {
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
      // During scan we pause alerts to preserve bandwidth (scan is transactional).
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

    // idle
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
