// WHY: Shared browser-side store for ESP telemetry + events.
// WHY: Classic script (not a module) so both modules and non-modules can use it via window.TelemetryStore.

(function initTelemetryStore() {
  if (window.TelemetryStore) return;

  const state = {
    // SECTION: Auth and link state.
    auth: { state: "DISARMED", triesLeft: 3, pending: false },
    link: { megaAgeMs: null, megaAgeMsAvg: null },

    // SECTION: Telemetry snapshot cache.
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
      // CONTRACT: `rgbRefs` is either null or an array of `{r,g,b}` entries.
      rgbRefs: null,
      encCal: "--",
      turretCal: "--",
    },

    // SECTION: Alerts metadata.
    alerts: { lastSeq: 0, lastLine: "" },

    // SECTION: Scan metadata (display only).
    scan: { active: false, dir: "+", nextDir: "+", last: "" },
  };

  let version = 0;
  const subs = new Set();

  // SECTION: Snapshot and mutation API.
  function snap() {
    return { state, version };
  }

  function set(partial) {
    if (!partial || typeof partial !== "object") return;
    // CONTRACT: Restrict merges to known top-level keys.
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
    // WHY: Subscribers render immediately from current state.
    try {
      cb(snap());
    } catch {}
    return () => subs.delete(cb);
  }

  function clearTelemetry() {
    // CONTRACT: Clearing telemetry preserves auth/link/alerts/scan state.
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

