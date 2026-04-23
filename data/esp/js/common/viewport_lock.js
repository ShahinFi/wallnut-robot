// Lock a stable viewport height on mobile Safari so UI doesn't "breathe"
// when the address bar expands/collapses while scrolling.
//
// Strategy:
// - Compute a baseline height once on load.
// - Expose it as CSS variables:
//     --app-h: <px>   (full locked height)
//     --vh:    <px>   (1% of locked height)
// - Update only on orientation changes (width changes), not on scroll-driven height changes.

(function () {
  function measure() {
    const vv = window.visualViewport;
    // Use visual viewport when available; fallback to innerHeight.
    const h = Math.round((vv && vv.height) || window.innerHeight || 0);
    const w = Math.round((vv && vv.width) || window.innerWidth || 0);
    return { h, w };
  }

  let last = measure();

  function apply(h) {
    if (!h || !Number.isFinite(h) || h <= 0) return;
    const root = document.documentElement;
    root.style.setProperty("--app-h", `${h}px`);
    root.style.setProperty("--vh", `${h * 0.01}px`);
  }

  function maybeUpdate() {
    const cur = measure();
    // Only treat as "real" resize when width changes meaningfully (orientation / split-view),
    // not when the address bar animates.
    const dw = Math.abs(cur.w - last.w);
    if (dw >= 20) {
      last = cur;
      apply(cur.h);
    }
  }

  function init() {
    apply(last.h);
    window.addEventListener("orientationchange", () => {
      // Give Safari time to settle after rotation.
      setTimeout(() => {
        last = measure();
        apply(last.h);
      }, 300);
    });
    window.addEventListener("resize", () => {
      // Resize can fire for address bar changes; we ignore unless width changes.
      maybeUpdate();
    });
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init, { once: true });
  } else {
    init();
  }
})();

