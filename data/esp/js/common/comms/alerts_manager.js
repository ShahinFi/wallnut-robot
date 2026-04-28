// WHY: Alerts stream polling manager (classic script).
// WHY: Polls GET /alerts?from= and emits lines to subscribers.

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
  const subs = new Set(); // CONTRACT: subscriber payload is {seq,line}.

  // SECTION: Poll transport helpers.
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
    // SECTION: One alerts stream cycle.
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
      // CONTRACT: `/alerts` is `seq|line` per row; malformed rows are ignored.
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
    // WHY: Self-scheduling timeout avoids overlapping intervals under slow network.
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
    // CONTRACT: Subscriber callback must tolerate replay and burst delivery.
    subs.add(cb);
    return () => subs.delete(cb);
  }

  function resetCursor() {
    // CONTRACT: Reset allows explicit replay from sequence zero.
    fromSeq = 0;
  }

  function setCursor(seq) {
    // CONTRACT: Cursor accepts only finite non-negative integers.
    const v = Number(seq);
    if (!Number.isFinite(v) || v < 0) return;
    fromSeq = Math.floor(v);
  }

  function cursor() {
    return fromSeq;
  }

  window.AlertsManager = { start, stop, setPeriod, pollOnce, subscribe, resetCursor, setCursor, cursor, _state: () => ({ enabled, periodMs, inFlight, fromSeq }) };
})();

