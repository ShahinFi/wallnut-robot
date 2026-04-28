// SECTION: Manual dashboard controls and telemetry rendering.
const compassSlider = document.getElementById("compassSlider");

// WHY: Keep nonessential polling behind command traffic to reduce control latency.
let pendingCommands = 0;
const DEG = "\u00B0";
const kFetchTimeoutMs = 1200;

// SECTION: HTTP command helpers.
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
  // CONTRACT: Legacy helper reflects only in-flight command count on this page.
  return pendingCommands > 0;
}

// SECTION: Compass controls.
if (compassSlider) {
  const middleValue = 0;
  compassSlider.value = middleValue;
  updateCompass(middleValue);
}

// SECTION: Compass display helper.
function updateCompass(pos) {
  document.getElementById("compassValue").innerText = `${pos}${DEG}`;
}

// CONTRACT: Compass commands are fire-and-forget from the dashboard.
function sendCompassValue(pos) {
  sendCommand(`/compass?value=${pos}`);
  console.log("Compass value", pos);
}

// SECTION: Discrete movement shortcuts.
function forwards5() {
  move("forwards", 5);
}
function forwards20() {
  move("forwards", 20);
}
function backwards5() {
  move("backwards", 5);
}
function backwards20() {
  move("backwards", 20);
}

// SECTION: Drive command helper.
function move(dir, dis) {
  sendCommand(`/${dir}${dis}`);
  console.log("Drive", dir, dis);
}

// SECTION: Page navigation.
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

function setAuthStatus(text) {
  const el = document.getElementById("authStatus");
  if (el) el.innerText = text;
}

const linkStatusEl = document.getElementById("linkStatus");
function setLinkStatus(text) {
  if (linkStatusEl) linkStatusEl.innerText = text;
}

async function armRobot() {
  // SECTION: Auth command handlers.
  const input = document.getElementById("passcodeInput");
  const code = input ? String(input.value || "").trim() : "";
  if (!code) {
    setAuthStatus("MISSING CODE");
    return;
  }
  setAuthStatus("ARMING...");
  setControlsEnabled(false);
  try {
    await fetchWithTimeout("/arm", {
      method: "POST",
      cache: "no-store",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: `code=${encodeURIComponent(code)}`,
    });
  } catch (e) {
    setAuthStatus("ARM ERROR");
  }
}

async function disarmRobot() {
  setAuthStatus("DISARMING...");
  setControlsEnabled(false);
  try {
    await fetchWithTimeout("/disarm", { method: "POST", cache: "no-store" });
  } catch (e) {}
}

function renderFromStore_() {
  // SECTION: Store-to-UI projection.
  const st = window.TelemetryStore ? window.TelemetryStore.get() : null;
  if (!st) return;

  // CONTRACT: Motion controls remain disabled unless auth is ARMED.
  const a = st.auth || {};
  const auth = String(a.state || "DISARMED").toUpperCase();
  const pending = !!a.pending || auth === "PENDING";
  const triesLeft = Number(a.triesLeft);

  if (pending) {
    setAuthStatus("PENDING...");
    setControlsEnabled(false);
  } else if (auth === "ARMED") {
    setAuthStatus("ARMED");
    setControlsEnabled(true);
  } else if (auth === "LOCKED") {
    setAuthStatus("LOCKED (RESET REQUIRED)");
    setControlsEnabled(false);
  } else {
    const tries = Number.isFinite(triesLeft) ? triesLeft : 3;
    setAuthStatus(`DISARMED (PASSCODE TRIES LEFT: ${tries})`);
    setControlsEnabled(false);
  }

  // SECTION: Link status.
  const age = Number(st.link && st.link.megaAgeMs);
  const ageAvg = Number(st.link && st.link.megaAgeMsAvg);
  if (!Number.isFinite(age) || age === 0xffffffff) setLinkStatus("LINK: NO DATA");
  else if (age > 2000) setLinkStatus(`LINK: OFFLINE (${age}ms)`);
  else if (Number.isFinite(ageAvg)) setLinkStatus(`LINK: OK (avg ${ageAvg}ms)`);
  else setLinkStatus(`LINK: OK (${age}ms)`);

  const t = st.telemetry || {};

  // SECTION: LiDAR.
  const lidarEl = document.getElementById("lidarValue");
  const warnEl = document.getElementById("lidarWarning");
  if (lidarEl) lidarEl.innerText = Number.isFinite(t.lidarCm) ? `${t.lidarCm.toFixed(1)} cm` : "-- cm";
  if (warnEl) {
    if (Number.isFinite(t.lidarCm) && t.lidarCm < 10) warnEl.classList.remove("hidden");
    else warnEl.classList.add("hidden");
  }

  // SECTION: Compass telemetry.
  const compassEl = document.getElementById("compassWebValue");
  if (compassEl) {
    if (Number.isFinite(t.compassDeg)) compassEl.innerText = `${t.compassDeg}${DEG} ${t.compassLabel || ""}`.trim();
    else compassEl.innerText = "--";
  }

  // SECTION: Live RGB.
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

// SECTION: Comms startup.
try {
  if (window.AuthManager) window.AuthManager.start();
} catch {}
try {
  if (window.CommsOrchestrator) window.CommsOrchestrator.start();
} catch {}
try {
  if (window.TelemetryStore && window.TelemetryStore.subscribe) window.TelemetryStore.subscribe(() => renderFromStore_());
} catch {}


