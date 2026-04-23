// Alerts stream polling manager (classic script).
// Polls GET /alerts?from= and emits lines to subscribers.

(function initAlertsManager() {
  if (window.AlertsManager) return;

  const store = window.TelemetryStore;
  if (!store) {
    console.warn("TelemetryStore missing");
    return;
  }

  let enabled = false;
  let periodMs = 140;
  let timer = 0;
  let inFlight = false;
  let fromSeq = 0;
  const subs = new Set(); // cb({seq,line})

  async function fetchTextWithTimeout_(url, timeoutMs) {
    const t = Number(timeoutMs);
    const ms = Number.isFinite(t) && t > 0 ? t : 2000;
    if (typeof AbortController !== "function") {
      const res = await fetch(url, { cache: "no-store" });
      if (!res.ok) return { ok: false, status: res.status, text: "" };
      return { ok: true, status: res.status, text: String(await res.text()) };
    }
    const ac = new AbortController();
    const to = setTimeout(() => {
      try { ac.abort(); } catch {}
    }, ms);
    try {
      const res = await fetch(url, { cache: "no-store", signal: ac.signal });
      const text = res.ok ? String(await res.text()) : "";
      return { ok: res.ok, status: res.status, text };
    } catch {
      return { ok: false, status: 0, text: "" };
    } finally {
      clearTimeout(to);
    }
  }

  async function pollOnce() {
    if (!enabled) return;
    if (inFlight) return;
    if (window.NetGate && !window.NetGate.allow("alerts")) return;

    inFlight = true;
    try {
      const r = await fetchTextWithTimeout_(`/alerts?from=${encodeURIComponent(String(fromSeq))}`, 2000);
      if (!r.ok) return;
      if (r.status === 403) return;
      const text = String(r.text || "");
      const rows = text.split("\n");
      for (const row of rows) {
        if (!row) continue;
        const bar = row.indexOf("|");
        if (bar <= 0) continue;
        const seq = Number(row.substring(0, bar));
        const line = row.substring(bar + 1);
        if (Number.isFinite(seq)) {
          fromSeq = Math.max(fromSeq, seq);
          store.set({ alerts: { lastSeq: fromSeq, lastLine: line } });
        }
        for (const cb of subs) {
          try {
            cb({ seq, line });
          } catch {}
        }
      }
    } finally {
      inFlight = false;
    }
  }

  function scheduleNext_() {
    if (!enabled) return;
    timer = setTimeout(async () => {
      await pollOnce();
      scheduleNext_();
    }, periodMs);
  }

  function start() {
    if (enabled && timer) return;
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
    periodMs = Math.max(50, Math.floor(v));
  }

  function subscribe(cb) {
    if (typeof cb !== "function") return () => {};
    subs.add(cb);
    return () => subs.delete(cb);
  }

  function resetCursor() {
    fromSeq = 0;
  }

  window.AlertsManager = { start, stop, setPeriod, pollOnce, subscribe, resetCursor, _state: () => ({ enabled, periodMs, inFlight, fromSeq }) };
})();

