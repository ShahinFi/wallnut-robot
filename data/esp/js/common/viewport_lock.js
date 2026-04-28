// WHY: Lock a stable viewport height on mobile Safari so UI doesn't "breathe"
// WHY: when the address bar expands/collapses while scrolling.
// SECTION: Strategy:
// SECTION: - Compute a baseline height once on load.
// SECTION: - Expose it as CSS variables:
// SECTION: --app-h: <px>   (full locked height)
// SECTION: --vh:    <px>   (1% of locked height)
// CONTRACT: - Update only on orientation changes (width changes), not on scroll-driven height changes.

(function () {
  // SECTION: Viewport sampling and CSS variable projection.
  function measure() {
    const vv = window.visualViewport;
    // WHY: visualViewport reflects visible height more reliably on mobile.
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
    // CONTRACT: Only treat as "real" resize when width changes meaningfully (orientation / split-view),
    // WHY: not when the address bar animates.
    const dw = Math.abs(cur.w - last.w);
    if (dw >= 20) {
      last = cur;
      apply(cur.h);
    }
  }

  function init() {
    apply(last.h);
    window.addEventListener("orientationchange", () => {
      // WHY: Safari reports transient values during orientation transitions.
      setTimeout(() => {
        last = measure();
        apply(last.h);
      }, 300);
    });
    window.addEventListener("resize", () => {
      // CONTRACT: Ignore resize events that are only browser chrome animation.
      maybeUpdate();
    });
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init, { once: true });
  } else {
    init();
  }
})();


