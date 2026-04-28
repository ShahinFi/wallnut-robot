// SECTION: Optional browser/ESP debug timeline.
// CONTRACT: Debug logging is opt-in and must not affect robot behavior.

(function initDebugLog() {
  const el = document.getElementById("debugLog");
  const wrap = el ? el.closest("details") : null;
  if (!el) return;

  const kMaxLines = 240;
  const kPollMs = 250;

  const lines = [];
  let enabled = false;
  let timer = 0;
  let fromSeq = 0;
  const qs = new URLSearchParams(window.location.search);
  const pollEsp = qs.get("espdebug") === "1" || qs.get("espdebug") === "true";

  // SECTION: Render and append helpers.
  function ts() {
    const d = new Date();
    const hh = String(d.getHours()).padStart(2, "0");
    const mm = String(d.getMinutes()).padStart(2, "0");
    const ss = String(d.getSeconds()).padStart(2, "0");
    const ms = String(d.getMilliseconds()).padStart(3, "0");
    return `${hh}:${mm}:${ss}.${ms}`;
  }

  function render() {
    el.textContent = lines.join("\n") || (enabled ? "" : "disabled");
  }

  function push(tag, msg) {
    const t = ts();
    const s = `${t} ${String(tag || "").padEnd(7)} ${String(msg || "")}`.trimEnd();
    lines.push(s);
    while (lines.length > kMaxLines) lines.shift();
    render();
  }

  async function pollOnce() {
    // SECTION: Optional ESP log stream cycle.
    if (!enabled) return;
    if (!pollEsp) return;
    if (window.NetGate && !window.NetGate.allow("debug")) return;
    // CONTRACT: Debug polling must yield to scan transport.
    if (window.NetGate && window.NetGate.mode() === "scan") return;

    let res;
    try {
      res = await fetch(`/debuglog?from=${encodeURIComponent(String(fromSeq))}`, { cache: "no-store" });
    } catch {
      return;
    }
    if (!res.ok) return;
    const text = await res.text();
    const rows = String(text || "").split("\n");
    for (const row of rows) {
      if (!row) continue;
      const bar1 = row.indexOf("|");
      const bar2 = bar1 >= 0 ? row.indexOf("|", bar1 + 1) : -1;
      if (bar1 <= 0 || bar2 <= bar1) continue;
      const seq = Number(row.substring(0, bar1));
      const ms = row.substring(bar1 + 1, bar2);
      const msg = row.substring(bar2 + 1);
      if (Number.isFinite(seq)) fromSeq = Math.max(fromSeq, seq);
      push("esp", `[${ms}ms] ${msg}`);
    }
  }

  function start() {
    // CONTRACT: Start is idempotent and creates at most one active poll loop.
    if (enabled) return;
    enabled = true;
    push("dbg", "enabled");
    let inFlight = false;
    if (timer) clearTimeout(timer);
    const tick = async () => {
      if (!enabled) return;
      if (inFlight) {
        timer = setTimeout(tick, kPollMs);
        return;
      }
      inFlight = true;
      try {
        await pollOnce();
      } catch {}
      finally {
        inFlight = false;
        if (enabled) timer = setTimeout(tick, kPollMs);
      }
    };
    timer = setTimeout(tick, 0);
  }

  function stop() {
    if (!enabled) return;
    enabled = false;
    if (timer) clearTimeout(timer);
    timer = 0;
    push("dbg", "disabled");
  }

  // SECTION: Public debug API.
  window.DebugLog = {
    push,
    enabled: () => enabled,
    enable: start,
    disable: stop,
  };

  // WHY: Allow quick enable via query param or debug panel toggle.
  const want = qs.get("debug");
  if (want === "1" || want === "true") start();

  if (wrap) {
    wrap.addEventListener("toggle", () => {
      if (wrap.open) start();
      else stop();
    });
  }

  render();
})();
