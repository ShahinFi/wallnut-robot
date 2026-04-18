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
    const sx = (width - 2 * pad) / g.mapW_cm;
    const sy = (height - 2 * pad) / g.mapH_cm;
    // Real scale (uniform): 1 cm maps to s pixels in both axes.
    const s = Math.min(sx, sy);

    const mapWpx = g.mapW_cm * s;
    const mapHpx = g.mapH_cm * s;
    const ox = pad + (width - 2 * pad - mapWpx) * 0.5;
    const oy = pad + (height - 2 * pad - mapHpx) * 0.5;

    const toPx = (x_cm, y_cm) => {
      // map coords: (0,0) bottom-left; canvas: y down
      return { x: ox + x_cm * s, y: oy + mapHpx - y_cm * s };
    };

    // background
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "rgba(7,3,15,0.12)";
    ctx.fillRect(0, 0, width, height);

    // map border
    ctx.strokeStyle = "rgba(203,215,255,0.30)";
    ctx.lineWidth = 2;
    ctx.strokeRect(ox, oy, mapWpx, mapHpx);

    // subtle major grid (about every 10cm) for orientation (does not affect scale)
    const majorEvery = Math.max(1, Math.floor(10 / g.cell_cm)); // ~10cm
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(203,215,255,0.05)";
    for (let cx = 0; cx <= g.w; cx += majorEvery) {
      const x = ox + cx * g.cell_cm * s;
      ctx.beginPath();
      ctx.moveTo(x, oy);
      ctx.lineTo(x, oy + mapHpx);
      ctx.stroke();
    }
    for (let cy = 0; cy <= g.h; cy += majorEvery) {
      const y = oy + mapHpx - cy * g.cell_cm * s;
      ctx.beginPath();
      ctx.moveTo(ox, y);
      ctx.lineTo(ox + mapWpx, y);
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
