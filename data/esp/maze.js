// Maze page JS: reuses existing telemetry endpoints and the existing /maze command.
// Note: encoder mm/pulse needs firmware support to populate; stays "--" for now.

let pendingCommands = 0;
function sendCommand(path, options = {}) {
  pendingCommands++;
  return fetch(path, { cache: "no-store", ...options })
    .catch(() => {})
    .finally(() => {
      pendingCommands = Math.max(0, pendingCommands - 1);
    });
}
function pollingPaused() {
  return pendingCommands > 0;
}

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
    const res = await fetch("/auth", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
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
    const res = await fetch("/arm", {
      method: "POST",
      cache: "no-store",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: `code=${encodeURIComponent(code)}`,
    });
    const text = (await res.text()).trim();
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
    const res = await fetch("/disarm", { method: "POST", cache: "no-store" });
    const text = (await res.text()).trim();
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
      if (el) el.innerText = "RUNNING...";
    })
    .catch(() => {});
}

function startTurretCalibration() {
  sendCommand("/turret_cal_start", { method: "POST" })
    .then(() => {
      const el = document.getElementById("turretCalValue");
      if (el) el.innerText = "CALIBRATING...";
    })
    .catch(() => {});
}

function finishTurretCalibration() {
  sendCommand("/turret_cal_done", { method: "POST" })
    .then(() => {
      const el = document.getElementById("turretCalValue");
      if (el) el.innerText = "SAVING...";
    })
    .catch(() => {});
}

function setTurretZero() {
  sendCommand("/turret_zero", { method: "POST" }).catch(() => {});
}

// Encoder calibration polling
const encCalValueEl = document.getElementById("encCalValue");
async function pollEncCal() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  try {
    const res = await fetch("/enc_cal", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
    if (!encCalValueEl) return;
    if (text === "--" || text.length === 0) {
      encCalValueEl.innerText = "--";
      return;
    }
    if (text.startsWith("ENC_CAL:")) {
      encCalValueEl.innerText = text.substring(8).trim();
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
  try {
    const res = await fetch("/turret_cal", { cache: "no-store" });
    if (!res.ok) return;
    const text = (await res.text()).trim();
    if (!turretCalValueEl) return;
    if (text === "--" || text.length === 0) {
      turretCalValueEl.innerText = "--";
      return;
    }
    if (text.startsWith("TURCAL:")) {
      turretCalValueEl.innerText = text.substring(7).trim();
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
  try {
    const res = await fetch("/lidar", { cache: "no-store" });
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
  try {
    const res = await fetch("/compassdata", { cache: "no-store" });
    if (!res.ok) return;
    const text = await res.text();
    if (compassWebValueEl) {
      const parts = text.trim().split(",");
      const deg = parts[0] || "--";
      const label = parts[1] || "";
      compassWebValueEl.innerText = `${deg}° ${label}`;
    }
  } catch (e) {}
}

setInterval(pollCompass, 400);
pollCompass();

// RGB polling
const colorValueEl = document.getElementById("colorValue");
const colorSwatchEl = document.getElementById("colorSwatch");
async function pollRgb() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  try {
    const res = await fetch("/rgb", { cache: "no-store" });
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

// Initialize auth gating
setControlsEnabled(false);
setSensorPlaceholdersLocked(false);
refreshAuth();
setInterval(refreshAuth, 1500);
