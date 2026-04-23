// Maze page JS: reuses existing telemetry endpoints and the existing /maze command.
// Note: encoder mm/pulse needs firmware support to populate; stays "--" for now.

let pendingCommands = 0;
const kFetchTimeoutMs = 1200;

function fetchWithTimeout(path, options = {}) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), kFetchTimeoutMs);
  return fetch(path, { ...options, signal: controller.signal }).finally(() => clearTimeout(timeout));
}
function sendCommand(path, options = {}) {
  pendingCommands++;
  return fetchWithTimeout(path, { cache: "no-store", ...options })
    .catch(() => {})
    .finally(() => {
      pendingCommands = Math.max(0, pendingCommands - 1);
    });
}
function pollingPaused() {
  // Keep legacy helper for command flows; page telemetry is centralized via /telemetry.
  return pendingCommands > 0;
}

const DEG = "\u00B0";

const CalUiState = Object.freeze({
  Idle: "idle",
  Running: "running",
  Saving: "saving",
  Done: "done",
  Error: "error",
});

let encCalUiState = CalUiState.Idle;

function setControlsEnabled(enabled) {
  document.querySelectorAll("[data-requires-arm='1']").forEach((el) => {
    el.disabled = !enabled;
    if (!enabled) el.classList.add("is-disabled");
    else el.classList.remove("is-disabled");
  });
}

function setSensorPlaceholdersLocked(locked) {
  const lidarEl = document.getElementById("lidarValue");
  const compassEl = document.getElementById("compassWebValue");
  const colorEl = document.getElementById("colorValue");
  const swatchEl = document.getElementById("colorSwatch");
  const encEl = document.getElementById("encCalValue");
  const turretEl = document.getElementById("turretCalValue");
  const warnEl = document.getElementById("lidarWarning");
  if (lidarEl) lidarEl.innerText = locked ? "LOCKED" : "-- cm";
  if (compassEl) compassEl.innerText = "--";
  if (colorEl) colorEl.innerText = "--";
  if (encEl) encEl.innerText = "--";
  if (turretEl) turretEl.innerText = "--";
  if (locked) {
    encCalUiState = CalUiState.Idle;
  }
  if (swatchEl) {
    swatchEl.style.backgroundColor = "transparent";
    swatchEl.classList.add("is-empty");
  }
  if (warnEl) warnEl.classList.add("hidden");
}

function setAuthStatus(text) {
  const el = document.getElementById("authStatus");
  if (el) el.innerText = text;
}

function requiresArmDisabled() {
  const el = document.querySelector("[data-requires-arm='1']");
  return !!(el && el.disabled);
}

// Auth/link/telemetry are rendered from window.TelemetryStore (polled via /telemetry).

const linkStatusEl = document.getElementById("linkStatus");
function setLinkStatus(text) {
  if (linkStatusEl) linkStatusEl.innerText = text;
}



async function armRobot() {
  const input = document.getElementById("passcodeInput");
  const code = input ? String(input.value || "").trim() : "";
  if (!code) {
    setAuthStatus("MISSING CODE");
    return;
  }
  setAuthStatus("ARMING...");
  setControlsEnabled(false);
  try {
    const res = await fetchWithTimeout("/arm", {
      method: "POST",
      cache: "no-store",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: `code=${encodeURIComponent(code)}`,
    });
    const text = (await res.text()).trim();
    if (res.status === 202 || text === "PENDING") {
      setAuthStatus("ARMING...");
      setControlsEnabled(false);
      setSensorPlaceholdersLocked(false);
      return;
    }
    if (text === "OK") {
      setAuthStatus("ARMED");
      setControlsEnabled(true);
    } else if (text === "LOCKED") {
      setAuthStatus("LOCKED (RESET REQUIRED)");
      setControlsEnabled(false);
      setSensorPlaceholdersLocked(true);
    } else if (text.startsWith("FAIL:")) {
      const tries = text.split(":")[1] || "";
      setAuthStatus(`WRONG PASSCODE (PASSCODE TRIES LEFT: ${tries})`);
      setControlsEnabled(false);
      setSensorPlaceholdersLocked(false);
    } else if (text === "NO_REPLY") {
      setAuthStatus("NO REPLY FROM ROBOT");
      setControlsEnabled(false);
      setSensorPlaceholdersLocked(false);
    } else {
      setAuthStatus(text || "ARM ERROR");
      setControlsEnabled(false);
      setSensorPlaceholdersLocked(false);
    }
  } catch (e) {
    setAuthStatus("ARM ERROR");
  }
}

async function disarmRobot() {
  setAuthStatus("DISARMING...");
  setControlsEnabled(false);
  try {
    const res = await fetchWithTimeout("/disarm", { method: "POST", cache: "no-store" });
    const text = (await res.text()).trim();
    if (res.status === 202 || text === "PENDING") {
      setAuthStatus("DISARMING...");
      setControlsEnabled(false);
      return;
    }
    if (text === "LOCKED") {
      setAuthStatus("LOCKED (RESET REQUIRED)");
      setSensorPlaceholdersLocked(true);
    } else if (text === "NO_REPLY") {
      setAuthStatus("NO REPLY FROM ROBOT");
    }
  } catch (e) {}
}

function startEncoderCalibration() {
  sendCommand("/enc_cal", { method: "POST" })
    .then(() => {
      const el = document.getElementById("encCalValue");
      encCalUiState = CalUiState.Running;
      if (el) el.innerText = "RUNNING...";
    })
    .catch(() => {});
}

function setTurretZero() {
  const btn = document.getElementById("turretZeroBtn");
  const flash = (cls) => {
    if (!btn) return;
    if (btn._flashRestoreHtml === undefined) {
      btn._flashRestoreHtml = btn.innerHTML;
    }
    btn.classList.remove("is-flash-ok", "is-flash-fail");
    btn.classList.add(cls);
    clearTimeout(btn._flashTimer);
    btn.innerText = cls === "is-flash-ok" ? "SUCCESS" : "FAILED";
    btn._flashTimer = setTimeout(() => {
      btn.classList.remove("is-flash-ok", "is-flash-fail");
      if (btn._flashRestoreHtml !== undefined) btn.innerHTML = btn._flashRestoreHtml;
    }, 2500);
  };

  sendCommand("/turret_zero", { method: "POST" })
    .then((res) => {
      if (res && res.ok) flash("is-flash-ok");
      else flash("is-flash-fail");
    })
    .catch(() => {
      flash("is-flash-fail");
    });
}

function setTurretTpr() {
  const posEl = document.getElementById("turretTprPos");
  const negEl = document.getElementById("turretTprNeg");
  const btn = document.getElementById("turretTprSetBtn");
  const flash = (cls) => {
    if (!btn) return;
    if (btn._flashRestoreHtml === undefined) {
      btn._flashRestoreHtml = btn.innerHTML;
    }
    btn.classList.remove("is-flash-ok", "is-flash-fail");
    btn.classList.add(cls);
    clearTimeout(btn._flashTimer);
    btn.innerText = cls === "is-flash-ok" ? "SUCCESS" : "FAILED";
    btn._flashTimer = setTimeout(() => {
      btn.classList.remove("is-flash-ok", "is-flash-fail");
      if (btn._flashRestoreHtml !== undefined) btn.innerHTML = btn._flashRestoreHtml;
    }, 2500);
  };
  const pos = posEl ? parseInt(posEl.value, 10) : NaN;
  const neg = negEl ? parseInt(negEl.value, 10) : NaN;
  if (!Number.isFinite(pos) || !Number.isFinite(neg) || pos <= 0 || neg <= 0) return;
  const qs = `pos=${encodeURIComponent(pos)}&neg=${encodeURIComponent(neg)}`;
  const outEl = document.getElementById("turretCalValue");
  if (outEl) outEl.innerText = "SETTING...";
  sendCommand(`/turret_tpr?${qs}`, { method: "POST" })
    .then((res) => {
      if (res && res.ok) flash("is-flash-ok");
      else flash("is-flash-fail");
    })
    .catch(() => {
      flash("is-flash-fail");
    });
  // Kick a quicker snapshot so the UI shows updated values promptly.
  setTimeout(() => {
    try { window.TelemetryManager && window.TelemetryManager.pollOnce && window.TelemetryManager.pollOnce(); } catch (e) {}
  }, 250);
}

function renderFromStore_() {
  const st = window.TelemetryStore ? window.TelemetryStore.get() : null;
  if (!st) return;

  // Auth + UI gating.
  const a = st.auth || {};
  const auth = String(a.state || "DISARMED").toUpperCase();
  const pending = !!a.pending || auth === "PENDING";
  const triesLeft = Number(a.triesLeft);

  if (pending) {
    setAuthStatus("PENDING...");
    setControlsEnabled(false);
    setSensorPlaceholdersLocked(false);
  } else if (auth === "ARMED") {
    setAuthStatus("ARMED");
    setControlsEnabled(true);
  } else if (auth === "LOCKED") {
    setAuthStatus("LOCKED (RESET REQUIRED)");
    setControlsEnabled(false);
    setSensorPlaceholdersLocked(true);
  } else {
    const tries = Number.isFinite(triesLeft) ? triesLeft : 3;
    setAuthStatus(`DISARMED (PASSCODE TRIES LEFT: ${tries})`);
    setControlsEnabled(false);
    setSensorPlaceholdersLocked(false);
  }

  // Link status.
  const age = Number(st.link && st.link.megaAgeMs);
  if (!Number.isFinite(age) || age === 0xffffffff) setLinkStatus("LINK: NO DATA");
  else if (age > 2000) setLinkStatus(`LINK: OFFLINE (${age}ms)`);
  else setLinkStatus(`LINK: OK (${age}ms)`);

  const t = st.telemetry || {};

  // Lidar.
  const lidarEl = document.getElementById("lidarValue");
  const warnEl = document.getElementById("lidarWarning");
  if (lidarEl) {
    lidarEl.innerText = Number.isFinite(t.lidarCm) ? `${t.lidarCm.toFixed(1)} cm` : "-- cm";
  }
  if (warnEl) {
    if (Number.isFinite(t.lidarCm) && t.lidarCm < 10) warnEl.classList.remove("hidden");
    else warnEl.classList.add("hidden");
  }

  // Compass.
  const compassEl = document.getElementById("compassWebValue");
  if (compassEl) {
    if (Number.isFinite(t.compassDeg)) compassEl.innerText = `${t.compassDeg}${DEG} ${t.compassLabel || ""}`.trim();
    else compassEl.innerText = "--";
  }

  // Encoder cal.
  const encEl = document.getElementById("encCalValue");
  if (encEl) {
    const pkt = String(t.encCal || "").trim();
    const payload = pkt.startsWith("ENC_CAL:") ? pkt.substring(8).trim() : pkt;
    if (!payload || payload === "--") {
      if (encCalUiState !== CalUiState.Running) encEl.innerText = "--";
    } else {
      encCalUiState = CalUiState.Done;
      encEl.innerText = payload;
    }
  }

  // Turret cal.
  const turEl = document.getElementById("turretCalValue");
  if (turEl) {
    const pkt = String(t.turretCal || "").trim();
    const payload = pkt.startsWith("TURCAL:") ? pkt.substring(7).trim() : pkt;
    if (!payload || payload === "--") turEl.innerText = "--";
    else if (payload === "FAIL") turEl.innerText = "FAILED";
    else turEl.innerText = payload;
  }

  // RGB live.
  const colorEl = document.getElementById("colorValue");
  const swatchEl = document.getElementById("colorSwatch");
  const r = t.rgb && t.rgb.r;
  const g = t.rgb && t.rgb.g;
  const b = t.rgb && t.rgb.b;
  if (colorEl) {
    if ([r, g, b].every((v) => Number.isFinite(v))) colorEl.innerText = `RGB ${r}, ${g}, ${b}`;
    else colorEl.innerText = "--";
  }
  if (swatchEl) {
    if ([r, g, b].every((v) => Number.isFinite(v))) {
      swatchEl.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;
      swatchEl.classList.remove("is-empty");
    } else {
      swatchEl.style.backgroundColor = "transparent";
      swatchEl.classList.add("is-empty");
    }
  }

  // RGB class + swatch.
  const clsEl = document.getElementById("classValue");
  const clsSw = document.getElementById("classSwatch");
  const cls = Number(t.rgbClass);
  if (clsEl) clsEl.innerText = cls > 0 ? `#${cls}` : "NONE";
  if (clsSw) {
    const refs = t.rgbRefs;
    if (cls > 0 && refs && refs[cls - 1]) {
      const c = refs[cls - 1];
      clsSw.style.backgroundColor = `rgb(${c.r}, ${c.g}, ${c.b})`;
      clsSw.classList.remove("is-empty");
    } else {
      clsSw.style.backgroundColor = "transparent";
      clsSw.classList.add("is-empty");
    }
  }
}

// Start centralized comms and render on store updates.
try {
  if (window.CommsOrchestrator) window.CommsOrchestrator.start();
} catch {}
try {
  if (window.TelemetryStore && window.TelemetryStore.subscribe) window.TelemetryStore.subscribe(() => renderFromStore_());
} catch {}
