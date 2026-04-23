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
  if (lidarEl) lidarEl.innerText = Number.isFinite(t.lidarCm) ? `${t.lidarCm.toFixed(1)} cm` : "-- cm";
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
}

// Start centralized comms and render on store updates.
try {
  if (window.CommsOrchestrator) window.CommsOrchestrator.start();
} catch {}
try {
  if (window.TelemetryStore && window.TelemetryStore.subscribe) window.TelemetryStore.subscribe(() => renderFromStore_());
} catch {}
