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
  return pendingCommands > 0;
}

function netAllowsTelemetry() {
  const g = window.NetGate;
  if (!g) return true; // fail-open
  return g.allow("telemetry");
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

async function refreshAuth() {
  try {
    const res = await fetchWithTimeout("/auth", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
    if (text === "PENDING") {
      setAuthStatus("PENDING...");
      setControlsEnabled(false);
      setSensorPlaceholdersLocked(false);
      return;
    }
    if (text === "ARMED") {
      setAuthStatus("ARMED");
      setControlsEnabled(true);
      return;
    }
    if (text === "LOCKED") {
      setAuthStatus("LOCKED (RESET REQUIRED)");
      setControlsEnabled(false);
      setSensorPlaceholdersLocked(true);
      return;
    }
    if (text.startsWith("DISARMED:")) {
      const tries = text.split(":")[1] || "";
      setAuthStatus(`DISARMED (PASSCODE TRIES LEFT: ${tries})`);
    } else {
      setAuthStatus("DISARMED");
    }
    setControlsEnabled(false);
    setSensorPlaceholdersLocked(false);
  } catch (e) {}
}

const linkStatusEl = document.getElementById("linkStatus");
function setLinkStatus(text) {
  if (linkStatusEl) linkStatusEl.innerText = text;
}

async function refreshStatus() {
  try {
    const res = await fetchWithTimeout("/status", { cache: "no-store" });
    if (!res.ok) return;
    const s = await res.json();
    const age = Number(s.mega_age_ms);
    if (!Number.isFinite(age) || age === 0xffffffff) {
      setLinkStatus("LINK: NO DATA");
      return;
    }
    if (age > 2000) {
      setLinkStatus(`LINK: OFFLINE (${age}ms)`);
      return;
    }
    setLinkStatus(`LINK: OK (${age}ms)`);
  } catch (e) {}
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
      setTimeout(refreshAuth, 200);
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
    setTimeout(refreshAuth, 200);
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
      setTimeout(refreshAuth, 200);
      return;
    }
    if (text === "LOCKED") {
      setAuthStatus("LOCKED (RESET REQUIRED)");
      setSensorPlaceholdersLocked(true);
    } else if (text === "NO_REPLY") {
      setAuthStatus("NO REPLY FROM ROBOT");
    }
  } catch (e) {}
  refreshAuth();
}

function startMazeSolver() {
  // Use POST so GET /maze can serve the page.
  sendCommand("/maze", { method: "POST" });
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
  // Kick a quicker refresh instead of waiting for the 700ms interval.
  setTimeout(() => {
    try { pollTurretCal(); } catch (e) {}
  }, 250);
}

// Encoder calibration polling
const encCalValueEl = document.getElementById("encCalValue");
async function pollEncCal() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  if (!netAllowsTelemetry()) return;
  if (window.StatusBus) window.StatusBus.set("telemetry", { encCalPollMs: Date.now() });
  try {
    const res = await fetchWithTimeout("/enc_cal", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
    if (!encCalValueEl) return;
    if (text.startsWith("ENC_CAL:")) {
      const payload = text.substring(8).trim();
      if (payload === "--" || payload.length === 0) {
        // Firmware hasn't produced a value yet; don't clobber "RUNNING...".
        if (encCalUiState !== CalUiState.Running) encCalValueEl.innerText = "--";
        return;
      }
      encCalUiState = CalUiState.Done;
      encCalValueEl.innerText = payload;
      return;
    }
    if (text === "--" || text.length === 0) {
      // Don't clobber the optimistic "RUNNING..." label while a run is in progress.
      if (encCalUiState !== CalUiState.Running) encCalValueEl.innerText = "--";
      return;
    }
    encCalValueEl.innerText = text;
  } catch (e) {}
}

setInterval(pollEncCal, 700);
pollEncCal();

// Turret calibration polling
const turretCalValueEl = document.getElementById("turretCalValue");
async function pollTurretCal() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  if (!netAllowsTelemetry()) return;
  if (window.StatusBus) window.StatusBus.set("telemetry", { turretCalPollMs: Date.now() });
  try {
    const res = await fetchWithTimeout("/turret_cal", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
    if (!turretCalValueEl) return;
    if (text.startsWith("TURCAL:")) {
      const payload = text.substring(7).trim();
      if (payload === "--" || payload.length === 0) {
        turretCalValueEl.innerText = "--";
        return;
      }
      if (payload === "FAIL") {
        turretCalValueEl.innerText = "FAILED";
        return;
      }
      turretCalValueEl.innerText = payload;
      return;
    }
    if (text === "--" || text.length === 0) {
      turretCalValueEl.innerText = "--";
      return;
    }
    turretCalValueEl.innerText = text;
  } catch (e) {}
}

setInterval(pollTurretCal, 700);
pollTurretCal();

// Lidar polling
const lidarValueEl = document.getElementById("lidarValue");
const lidarWarningEl = document.getElementById("lidarWarning");

async function pollLidar() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  if (!netAllowsTelemetry()) return;
  if (window.StatusBus) window.StatusBus.set("telemetry", { lidarPollMs: Date.now() });
  try {
    const res = await fetchWithTimeout("/lidar", { cache: "no-store" });
    if (!res.ok) return;
    const text = await res.text();
    const trimmed = text.trim();
    let display = trimmed;
    if (trimmed.includes(",")) {
      const first = trimmed.split(",")[0];
      if (first.startsWith("LIDAR:")) display = first.substring(6);
    } else if (trimmed.startsWith("LIDAR:")) {
      display = trimmed.substring(6);
    }
    const raw = display.trim();
    const out = raw.toLowerCase().endsWith("cm") ? raw : `${raw} cm`;
    if (lidarValueEl) lidarValueEl.innerText = out;

    const dist = parseFloat(display);
    if (lidarWarningEl) {
      if (!isNaN(dist) && dist < 10) lidarWarningEl.classList.remove("hidden");
      else lidarWarningEl.classList.add("hidden");
    }
  } catch (e) {}
}

setInterval(pollLidar, 400);
pollLidar();

// Compass polling
const compassWebValueEl = document.getElementById("compassWebValue");
async function pollCompass() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  if (!netAllowsTelemetry()) return;
  if (window.StatusBus) window.StatusBus.set("telemetry", { compassPollMs: Date.now() });
  try {
    const res = await fetchWithTimeout("/compassdata", { cache: "no-store" });
    if (!res.ok) return;
    const text = await res.text();
    if (compassWebValueEl) {
      const parts = text.trim().split(",");
      const deg = parts[0] || "--";
      const label = parts[1] || "";
      compassWebValueEl.innerText = `${deg}${DEG} ${label}`;
    }
  } catch (e) {}
}

setInterval(pollCompass, 400);
pollCompass();

// RGB polling
const colorValueEl = document.getElementById("colorValue");
const colorSwatchEl = document.getElementById("colorSwatch");
const classValueEl = document.getElementById("classValue");
const classSwatchEl = document.getElementById("classSwatch");
let rgbRefs = null; // [{r,g,b}, ...] or null
async function pollRgb() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  if (!netAllowsTelemetry()) return;
  if (window.StatusBus) window.StatusBus.set("telemetry", { rgbPollMs: Date.now() });
  try {
    const res = await fetchWithTimeout("/rgb", { cache: "no-store" });
    if (!res.ok) return;
    const text = await res.text();
    const trimmed = text.trim();
    if (!trimmed.startsWith("RGB:")) return;
    const parts = trimmed.substring(4).split(",");
    if (parts.length < 3) return;
    const r = parseInt(parts[0], 10);
    const g = parseInt(parts[1], 10);
    const b = parseInt(parts[2], 10);
    if ([r, g, b].some((v) => isNaN(v))) return;

    if (colorValueEl) colorValueEl.innerText = `RGB ${r}, ${g}, ${b}`;
    if (colorSwatchEl) {
      colorSwatchEl.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;
      colorSwatchEl.classList.remove("is-empty");
    }
  } catch (e) {}
}

setInterval(pollRgb, 400);
pollRgb();

async function pollRgbRefs() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  if (!netAllowsTelemetry()) return;
  try {
    const res = await fetchWithTimeout("/rgb_refs", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
    if (!text.startsWith("REFS:")) return;
    const payload = text.substring(5).trim();
    if (!payload || payload === "--") {
      rgbRefs = null;
      return;
    }
    const parts = payload.split(";");
    const out = [];
    for (const p of parts) {
      const rgb = p.split(",");
      if (rgb.length < 3) continue;
      const r = parseInt(rgb[0], 10);
      const g = parseInt(rgb[1], 10);
      const b = parseInt(rgb[2], 10);
      if ([r, g, b].some((v) => isNaN(v))) continue;
      out.push({ r, g, b });
    }
    rgbRefs = out.length ? out : null;
  } catch (e) {}
}

async function pollRgbClass() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  if (!netAllowsTelemetry()) return;
  try {
    const res = await fetchWithTimeout("/rgb_class", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
    if (!text.startsWith("CLASS:")) return;
    const v = parseInt(text.substring(6), 10);
    if (isNaN(v)) return;

    if (classValueEl) classValueEl.innerText = v <= 0 ? "NONE" : `#${v}`;
    if (classSwatchEl) {
      if (v > 0 && rgbRefs && rgbRefs[v - 1]) {
        const c = rgbRefs[v - 1];
        classSwatchEl.style.backgroundColor = `rgb(${c.r}, ${c.g}, ${c.b})`;
        classSwatchEl.classList.remove("is-empty");
      } else {
        classSwatchEl.style.backgroundColor = "transparent";
        classSwatchEl.classList.add("is-empty");
      }
    }
  } catch (e) {}
}

setInterval(pollRgbRefs, 2000);
pollRgbRefs();
setInterval(pollRgbClass, 400);
pollRgbClass();

// Initialize auth gating
setControlsEnabled(false);
setSensorPlaceholdersLocked(false);
refreshAuth();
setInterval(refreshAuth, 1500);
refreshStatus();
setInterval(refreshStatus, 1500);
