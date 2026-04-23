// Shared browser-side store for ESP telemetry + events.
// Classic script (not a module) so both modules and non-modules can use it via window.TelemetryStore.

(function initTelemetryStore() {
  if (window.TelemetryStore) return;

  const state = {
    // Auth/link
    auth: { state: "DISARMED", triesLeft: 3, pending: false },
    link: { megaAgeMs: null, megaAgeMsAvg: null },

    // Telemetry snapshot (normalized where useful)
    telemetry: {
      lidarCm: NaN,
      lidarPacket: "",
      compassDeg: NaN,
      compassLabel: "",
      compassRaw: "",
      odomEast: NaN,
      odomNorth: NaN,
      odomRaw: "",
      rgb: { r: null, g: null, b: null },
      rgbRaw: "",
      rgbClass: 0,
      rgbRefs: null, // [{r,g,b},...]
      encCal: "--",
      turretCal: "--",
    },

    // Latest alerts meta (for UI/debug)
    alerts: { lastSeq: 0, lastLine: "" },

    // Scan meta (display only)
    scan: { active: false, dir: "+", nextDir: "+", last: "" },
  };

  let version = 0;
  const subs = new Set();

  function snap() {
    return { state, version };
  }

  function set(partial) {
    if (!partial || typeof partial !== "object") return;
    // Shallow merge only at known top-level keys to avoid surprises.
    if (partial.auth) Object.assign(state.auth, partial.auth);
    if (partial.link) Object.assign(state.link, partial.link);
    if (partial.telemetry) Object.assign(state.telemetry, partial.telemetry);
    if (partial.alerts) Object.assign(state.alerts, partial.alerts);
    if (partial.scan) Object.assign(state.scan, partial.scan);
    version++;
    for (const cb of subs) {
      try {
        cb(snap());
      } catch {}
    }
  }

  function subscribe(cb) {
    if (typeof cb !== "function") return () => {};
    subs.add(cb);
    // Fire once immediately so caller can render current values.
    try {
      cb(snap());
    } catch {}
    return () => subs.delete(cb);
  }

  function clearTelemetry() {
    state.telemetry = {
      lidarCm: NaN,
      lidarPacket: "",
      compassDeg: NaN,
      compassLabel: "",
      compassRaw: "",
      odomEast: NaN,
      odomNorth: NaN,
      odomRaw: "",
      rgb: { r: null, g: null, b: null },
      rgbRaw: "",
      rgbClass: 0,
      rgbRefs: null,
      encCal: "--",
      turretCal: "--",
    };
    version++;
    for (const cb of subs) {
      try {
        cb(snap());
      } catch {}
    }
  }

  window.TelemetryStore = { get: () => state, set, subscribe, clearTelemetry, _snap: snap };
})();
