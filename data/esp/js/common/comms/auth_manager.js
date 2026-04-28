// WHY: Auth + link polling manager (classic script).
// WHY: Polls GET /auth_state and updates window.TelemetryStore.auth/link.

(function initAuthManager() {
  if (window.AuthManager) return;

  const store = window.TelemetryStore;
  if (!store) {
    console.warn("TelemetryStore missing");
    return;
  }

  let enabled = true;
  let periodMs = 350;
  let timer = 0;
  let inFlight = false;

  // SECTION: Link-age smoothing state.
  // WHY: Smoothed link age keeps UI stable while preserving trend.
  const kMegaAgeWindow = 6;
  const megaAgeBuf = [];

  async function fetchJsonWithTimeout_(url, timeoutMs) {
    const t = Number(timeoutMs);
    const ms = Number.isFinite(t) && t > 0 ? t : 1200;
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

  function noteAge_(ageMs) {
    // CONTRACT: Sentinel or non-finite ages collapse to null display values.
    const ok = Number.isFinite(ageMs) && ageMs !== 0xffffffff;
    if (!ok) return { age: null, avg: null };
    megaAgeBuf.push(ageMs);
    while (megaAgeBuf.length > kMegaAgeWindow) megaAgeBuf.shift();
    const avg =
        megaAgeBuf.length > 0 ? megaAgeBuf.reduce((acc, v) => acc + v, 0) / megaAgeBuf.length : NaN;
    return { age: ageMs, avg: Number.isFinite(avg) ? Math.round(avg) : null };
  }

  async function pollOnce() {
    // SECTION: One auth/link snapshot cycle.
    if (!enabled) return;
    if (inFlight) return;
    inFlight = true;
    try {
      const j = await fetchJsonWithTimeout_("/auth_state", 1200);
      if (!j) return;

      const authText = String(j.auth || "DISARMED").toUpperCase();
      const triesLeft = Number(j.tries_left);
      const megaAgeMs = Number(j.mega_age_ms);
      const age = noteAge_(megaAgeMs);

      const pending = authText === "PENDING";
      // CONTRACT: Auth and link snapshots are written together to keep UI state coherent.
      store.set({
        auth: { state: authText, triesLeft: Number.isFinite(triesLeft) ? triesLeft : 3, pending },
        link: { megaAgeMs: age.age, megaAgeMsAvg: age.avg },
      });
    } finally {
      inFlight = false;
    }
  }

  function scheduleNext_() {
    // WHY: Timeout loop gives precise control over backpressure and stop semantics.
    if (!enabled) return;
    timer = setTimeout(async () => {
      await pollOnce();
      scheduleNext_();
    }, periodMs);
  }

  function start() {
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
    const v = Number(ms);
    if (!Number.isFinite(v) || v <= 0) return;
    periodMs = Math.max(100, Math.floor(v));
  }

  window.AuthManager = { start, stop, setPeriod, pollOnce, _state: () => ({ enabled, periodMs, inFlight }) };
})();


