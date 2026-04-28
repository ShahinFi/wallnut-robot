// WHY: Telemetry snapshot polling manager (classic script).
// WHY: Polls GET /telemetry and updates window.TelemetryStore.

(function initTelemetryManager() {
  if (window.TelemetryManager) return;

  const store = window.TelemetryStore;
  if (!store) {
    console.warn("TelemetryStore missing");
    return;
  }

  let enabled = true;
  let periodMs = 250;
  let timer = 0;
  let inFlight = false;

  // SECTION: Telemetry packet parsers.
  function parseRgbRefs_(s) {
    // SECTION: Parse `REFS:r,g,b;...` calibration snapshot.
    const text = String(s || "").trim();
    if (!text.startsWith("REFS:")) return null;
    const payload = text.substring(5).trim();
    if (!payload || payload === "--") return null;
    const parts = payload.split(";");
    const out = [];
    for (const p of parts) {
      const rgb = p.split(",");
      if (rgb.length < 3) continue;
      const r = parseInt(rgb[0], 10);
      const g = parseInt(rgb[1], 10);
      const b = parseInt(rgb[2], 10);
      if ([r, g, b].some((v) => Number.isNaN(v))) continue;
      out.push({ r, g, b });
    }
    return out.length ? out : null;
  }

  function parseRgbClass_(s) {
    // SECTION: Parse `CLASS:n` color classifier output.
    const text = String(s || "").trim();
    if (!text.startsWith("CLASS:")) return 0;
    const v = parseInt(text.substring(6), 10);
    return Number.isFinite(v) ? v : 0;
  }

  function parseRgb_(s) {
    // SECTION: Parse `RGB:r,g,b` live sensor packet.
    const text = String(s || "").trim();
    if (!text.startsWith("RGB:")) return { r: null, g: null, b: null, raw: text };
    const parts = text.substring(4).split(",");
    if (parts.length < 3) return { r: null, g: null, b: null, raw: text };
    const r = parseInt(parts[0], 10);
    const g = parseInt(parts[1], 10);
    const b = parseInt(parts[2], 10);
    if ([r, g, b].some((v) => Number.isNaN(v))) return { r: null, g: null, b: null, raw: text };
    return { r, g, b, raw: text };
  }

  function parseCompass_(s) {
    // SECTION: Parse `deg,label` compact compass packet.
    const text = String(s || "").trim();
    if (!text || text === "--") return { deg: NaN, label: "", raw: text };
    const parts = text.split(",");
    const deg = Number(parts[0]);
    const label = String(parts[1] || "").trim();
    return { deg: Number.isFinite(deg) ? deg : NaN, label, raw: text };
  }

  function parseOdom_(s) {
    // SECTION: Parse `ODOM:east,north` world-frame pose packet.
    const text = String(s || "").trim();
    if (!text.startsWith("ODOM:")) return { e: NaN, n: NaN, raw: text };
    const payload = text.substring(5);
    if (payload === "--") return { e: NaN, n: NaN, raw: text };
    const parts = payload.split(",");
    const e = Number(parts[0]);
    const n = Number(parts[1]);
    return { e: Number.isFinite(e) ? e : NaN, n: Number.isFinite(n) ? n : NaN, raw: text };
  }

  function parseLidarCm_(s) {
    // SECTION: Parse `LIDAR:cm,...` packet (first numeric field is distance).
    const text = String(s || "").trim();
    if (!text.startsWith("LIDAR:")) return { cm: NaN, raw: text };
    const first = text.includes(",") ? text.split(",")[0] : text;
    const cm = parseFloat(first.substring(6));
    return { cm: Number.isFinite(cm) ? cm : NaN, raw: text };
  }

  async function fetchJsonWithTimeout_(url, timeoutMs) {
    // CONTRACT: Network failures and timeouts degrade to `null` without throwing.
    const t = Number(timeoutMs);
    const ms = Number.isFinite(t) && t > 0 ? t : 1500;
    if (typeof AbortController !== "function") {
      const res = await fetch(url, { cache: "no-store" });
      if (!res.ok) return null;
      return await res.json();
    }
    const ac = new AbortController();
    const to = setTimeout(() => {
      try { ac.abort(); } catch {}
    }, ms);
    try {
      const res = await fetch(url, { cache: "no-store", signal: ac.signal });
      if (!res.ok) return null;
      return await res.json();
    } catch {
      return null;
    } finally {
      clearTimeout(to);
    }
  }

  async function pollOnce() {
    // SECTION: One telemetry snapshot cycle.
    if (!enabled) return;
    if (inFlight) return;
    // CONTRACT: Yield telemetry polling when scan bandwidth is prioritized.
    if (window.NetGate) {
      const ok = window.NetGate.allow("telemetry") || window.NetGate.allow("pose");
      if (!ok) return;
    }

    inFlight = true;
    try {
      const j = await fetchJsonWithTimeout_("/telemetry", 1500);
      if (!j) return;

      const lidar = parseLidarCm_(j.lidar);
      const comp = parseCompass_(j.compass);
      const od = parseOdom_(j.odom);
      const rgb = parseRgb_(j.rgb);
      const rgbClass = parseRgbClass_(j.rgb_class);
      const rgbRefs = parseRgbRefs_(j.rgb_refs);

      // CONTRACT: TelemetryStore receives one atomic snapshot per poll cycle.
      store.set({
        telemetry: {
          lidarCm: lidar.cm,
          lidarPacket: lidar.raw,
          compassDeg: comp.deg,
          compassLabel: comp.label,
          compassRaw: comp.raw,
          odomEast: od.e,
          odomNorth: od.n,
          odomRaw: od.raw,
          rgb: { r: rgb.r, g: rgb.g, b: rgb.b },
          rgbRaw: rgb.raw,
          rgbClass,
          rgbRefs,
          encCal: String(j.enc_cal || "--"),
          turretCal: String(j.turret_cal || "--"),
        },
      });
    } finally {
      inFlight = false;
    }
  }

  function scheduleNext_() {
    // CONTRACT: Poll loop must not overlap; each cycle schedules after completion.
    if (!enabled) return;
    timer = setTimeout(async () => {
      await pollOnce();
      scheduleNext_();
    }, periodMs);
  }

  function start() {
    // CONTRACT: Start is idempotent while timer is active.
    if (timer) return;
    enabled = true;
    scheduleNext_();
  }

  function stop() {
    enabled = false;
    if (timer) clearTimeout(timer);
    timer = 0;
  }

  function setPeriod(ms) {
    // CONTRACT: Poll period is clamped to avoid pathological tight loops.
    const v = Number(ms);
    if (!Number.isFinite(v) || v <= 0) return;
    periodMs = Math.max(50, Math.floor(v));
  }

  window.TelemetryManager = { start, stop, setPeriod, pollOnce, _state: () => ({ enabled, periodMs, inFlight }) };
})();

