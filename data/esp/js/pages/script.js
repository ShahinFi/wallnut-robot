const compassSlider = document.getElementById("compassSlider"); // Get the compass slider element from the HTML page

// Pause polling while commands are in flight to prioritize control.
let pendingCommands = 0;
const DEG = "\u00B0";
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

// Check if the compassSlider element exists and reset it to 0
if (compassSlider) {
  const middleValue = 0;
  compassSlider.value = middleValue; // Set slider to the middle position (0 degrees)
  updateCompass(middleValue); // Set the compass value to initial value of 0
}

// Function to update the compass display (show the current compass position)
function updateCompass(pos) {
  document.getElementById("compassValue").innerText = `${pos}°`; // Update the text value for compassValue when value is changing on page element with the current position and add "°" for degrees
}

// Function to send the current compass value to the server
function sendCompassValue(pos) {
  sendCommand(`/compass?value=${pos}`); // Send the compass value to the server using a fetch request
  console.log("Compass value", pos); // Log the compass value to the console for debugging
}

// Functions for moving the motor forward and backward with specific distances
function forwards5() {
  move("forwards", 5);
} // Calling move function with separated parameters
function forwards20() {
  move("forwards", 20);
} // Calling move function with separated parameters
function backwards5() {
  move("backwards", 5);
} // Calling move function with separated parameters
function backwards20() {
  move("backwards", 20);
} // Calling move function with separated parameters

// This is a general-purpose function that can handle different combinations of direction and distance
function move(dir, dis) {
  sendCommand(`/${dir}${dis}`); // Sends a request to the server with a URL constructed from the received direction and distance (e.g., /forwards5)
  console.log("Drive", dir, dis); // Log the movement command to the console for debugging
}

// Function to face north
function faceNorth() {
  sendCommand("/north");
  console.log("face north");
}

// Set current heading as "north" reference (used by FACE NORTH)
function setNorth() {
  sendCommand("/setnorth");
  console.log("set north");
}

// Open the dedicated maze solver page (separate from color maze).
function openMazeSolver() {
  window.location.href = "/maze";
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
  const warnEl = document.getElementById("lidarWarning");
  if (lidarEl) lidarEl.innerText = locked ? "LOCKED" : "-- cm";
  if (compassEl) compassEl.innerText = "--";
  if (colorEl) colorEl.innerText = "--";
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
    // Follow up with a real state read (covers FAIL:n, LOCKED, etc.)
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

// Initialize
setControlsEnabled(false);
setSensorPlaceholdersLocked(false);
refreshAuth();
setInterval(refreshAuth, 1500);
refreshStatus();
setInterval(refreshStatus, 1500);

// Lidar polling
const lidarValueEl = document.getElementById("lidarValue");
const lidarWarningEl = document.getElementById("lidarWarning");

async function pollLidar() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
  try {
    const res = await fetchWithTimeout("/lidar", { cache: "no-store" });
    if (!res.ok) return;
    const text = await res.text();
    const trimmed = text.trim();
    // Expected format: LIDAR:<cm>,SEQ:<n>,T:<ms>
    let display = trimmed;
    if (trimmed.includes(",")) {
      const first = trimmed.split(",")[0];
      if (first.startsWith("LIDAR:")) {
        display = first.substring(6);
      }
    } else if (trimmed.startsWith("LIDAR:")) {
      display = trimmed.substring(6);
    }
    const raw = display.trim();
    const out = raw.toLowerCase().endsWith("cm") ? raw : `${raw} cm`;
    if (lidarValueEl) lidarValueEl.innerText = out;

    const dist = parseFloat(display);
    if (lidarWarningEl) {
      if (!isNaN(dist) && dist < 10) {
        lidarWarningEl.classList.remove("hidden");
      } else {
        lidarWarningEl.classList.add("hidden");
      }
    }

    // Console-only stats
    if (trimmed.includes("SEQ:") && trimmed.includes("T:")) {
      const parts = trimmed.split(",");
      const seqStr = parts[1].split(":")[1];
      const tStr = parts[2].split(":")[1];
      const seq = parseInt(seqStr, 10);
      const t = parseInt(tStr, 10);
      if (!isNaN(seq)) {
        if (window._lastSeq !== undefined && seq > window._lastSeq + 1) {
          window._dropCount = (window._dropCount || 0) + (seq - window._lastSeq - 1);
        }
        window._lastSeq = seq;
      }
      if (!isNaN(t)) {
        window._recvCount = (window._recvCount || 0) + 1;
        window._lastTs = t;
      }
    }
  } catch (e) {
    // ignore transient errors
  }
}

setInterval(pollLidar, 400);
pollLidar();

// Console-only summary every 5s
setInterval(() => {
  const recv = window._recvCount || 0;
  const drop = window._dropCount || 0;
  const now = Date.now();
  if (!window._startTs) window._startTs = now;
  const elapsed = (now - window._startTs) / 1000.0;
  if (elapsed <= 0) return;
  const hz = recv / elapsed;
  const dropsPct = (recv + drop) > 0 ? (drop / (recv + drop)) * 100.0 : 0;
  console.log(`WEB rate ${hz.toFixed(1)} Hz, drops ${dropsPct.toFixed(1)}%`);
}, 5000);

// Compass polling
const compassWebValueEl = document.getElementById("compassWebValue");
const colorValueEl = document.getElementById("colorValue");
const colorSwatchEl = document.getElementById("colorSwatch");
async function pollCompass() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
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
  } catch (e) {
    // ignore transient errors
  }
}

setInterval(pollCompass, 400);
pollCompass();

// RGB polling
async function pollRgb() {
  if (pollingPaused()) return;
  if (requiresArmDisabled()) return;
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
  } catch (e) {
    // ignore transient errors
  }
}

setInterval(pollRgb, 400);
pollRgb();
