function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

export class MapRenderer {
  constructor(canvas, grid) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.grid = grid;
    this.showScan = true;
    this.scanPts = [];
    this.pose = { x: 0, y: 0, headingDeg: 0 };
    // Extra world-space margin (cm) rendered around the map boundary so you can
    // see out-of-bounds scan endpoints. Visual-only; does not change mapping.
    this.viewMarginCm = 6;
  }

  setViewMarginCm(marginCm) {
    const m = Number(marginCm);
    if (!Number.isFinite(m)) return;
    this.viewMarginCm = Math.max(0, Math.min(200, m));
  }

  setPose(pose) {
    this.pose = { ...this.pose, ...pose };
  }

  setScanPoints(pointsWorld) {
    this.scanPts = Array.isArray(pointsWorld) ? pointsWorld : [];
  }

  draw() {
    const ctx = this.ctx;

    // HiDPI-safe resize: draw in CSS pixels but back the canvas with device pixels.
    const dpr = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
    const rect = this.canvas.getBoundingClientRect();
    const cssW = Math.max(1, rect.width || this.canvas.width);
    const cssH = Math.max(1, rect.height || this.canvas.height);
    const pxW = Math.round(cssW * dpr);
    const pxH = Math.round(cssH * dpr);
    if (this.canvas.width !== pxW) this.canvas.width = pxW;
    if (this.canvas.height !== pxH) this.canvas.height = pxH;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const width = cssW;
    const height = cssH;
    const g = this.grid;

    // Fit map into canvas with padding.
    const pad = Math.max(18, Math.floor(Math.min(width, height) * 0.045));
    const mCm = Math.max(0, this.viewMarginCm || 0);
    const viewW_cm = g.mapW_cm + 2 * mCm;
    const viewH_cm = g.mapH_cm + 2 * mCm;
    const sx = (width - 2 * pad) / viewW_cm;
    const sy = (height - 2 * pad) / viewH_cm;
    // Real scale (uniform): 1 cm maps to s pixels in both axes.
    const s = Math.min(sx, sy);

    const viewWpx = viewW_cm * s;
    const viewHpx = viewH_cm * s;
    const ox = pad + (width - 2 * pad - viewWpx) * 0.5;
    const oy = pad + (height - 2 * pad - viewHpx) * 0.5;

    const toPx = (x_cm, y_cm) => {
      // world coords: (0,0) at map bottom-left; canvas: y down
      // Apply margin so we can render beyond the map rectangle.
      return { x: ox + (x_cm + mCm) * s, y: oy + viewHpx - (y_cm + mCm) * s };
    };

    // background
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "rgba(7,3,15,0.12)";
    ctx.fillRect(0, 0, width, height);

    // map border
    ctx.strokeStyle = "rgba(203,215,255,0.30)";
    ctx.lineWidth = 2;
    // Border is the true map extents within the view.
    ctx.strokeRect(ox + mCm * s, oy + mCm * s, g.mapW_cm * s, g.mapH_cm * s);

    // subtle major grid (about every 10cm) for orientation (does not affect scale)
    const majorEvery = Math.max(1, Math.floor(10 / g.cell_cm)); // ~10cm
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(203,215,255,0.05)";
    for (let cx = 0; cx <= g.w; cx += majorEvery) {
      const x = ox + (mCm + cx * g.cell_cm) * s;
      ctx.beginPath();
      ctx.moveTo(x, oy + mCm * s);
      ctx.lineTo(x, oy + (mCm + g.mapH_cm) * s);
      ctx.stroke();
    }
    for (let cy = 0; cy <= g.h; cy += majorEvery) {
      const y = oy + viewHpx - (mCm + cy * g.cell_cm) * s;
      ctx.beginPath();
      ctx.moveTo(ox + mCm * s, y);
      ctx.lineTo(ox + (mCm + g.mapW_cm) * s, y);
      ctx.stroke();
    }

    // hit cells (true squares, uniform scale)
    const maxV = 35;
    for (let cy = 0; cy < g.h; cy++) {
      for (let cx = 0; cx < g.w; cx++) {
        const v = g.cellLogOdds(cx, cy);
        const st = g.cellState(cx, cy);
        if (st === "unk") continue;

        // Map confidence to alpha based on magnitude of log-odds.
        const mag = Math.min(maxV, Math.abs(v));
        const t = clamp(mag / maxV, 0, 1);
        const alpha = 0.08 + 0.55 * t;

        // Opposite colors:
        // - OCC: cyan
        // - FREE: red (complement of cyan)
        if (st === "occ") ctx.fillStyle = `rgba(88,243,255,${alpha.toFixed(3)})`;
        else ctx.fillStyle = `rgba(255,92,92,${alpha.toFixed(3)})`;

        const x_cm = cx * g.cell_cm;
        const y_cm = cy * g.cell_cm;
        const p = toPx(x_cm, y_cm + g.cell_cm); // top-left
        const sz = g.cell_cm * s;
        // Slight inset so dense regions look cleaner.
        const inset = Math.min(1.2, sz * 0.08);
        ctx.fillRect(p.x + inset, p.y + inset, sz - 2 * inset, sz - 2 * inset);
      }
    }

    // scan overlay
    if (this.showScan && this.scanPts.length) {
      ctx.fillStyle = "rgba(255,95,135,0.85)";
      for (const pt of this.scanPts) {
        const p = toPx(pt.x, pt.y);
        ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
      }
    }

    // robot pose arrow
    const rp = toPx(this.pose.x, this.pose.y);
    ctx.save();
    ctx.translate(rp.x, rp.y);
    // heading: 0=N (up), 90=E (right), clockwise.
    const ang = ((this.pose.headingDeg - 90) * Math.PI) / 180; // convert to canvas angle (0=+x)
    ctx.rotate(ang);
    ctx.strokeStyle = "rgba(255,255,255,0.9)";
    ctx.fillStyle = "rgba(255,255,255,0.15)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(10, 0);
    ctx.lineTo(-8, -6);
    ctx.lineTo(-5, 0);
    ctx.lineTo(-8, 6);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}
