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
    // WHY: Virtual obstacles are a separate hard-block layer (Uint8Array, 1 blocked / 0 free).
    this.virtualBlocked = null;
    // CONTRACT: Goal marker shape is `{ cx, cy, tolCells }` in cell coordinates.
    this.goal = null;
    // CONTRACT: Planned path overlay is world-coordinate points `{x,y}`.
    this.plannedPath = [];
    // WHY: Extra world-space margin keeps out-of-bounds scan endpoints visible.
    // CONTRACT: Margin is visual-only and does not alter mapping data.
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

  setVirtualBlocked(blocked01) {
    this.virtualBlocked = blocked01 || null;
  }

  setGoalCell(goalCell) {
    if (!goalCell) { this.goal = null; return; }
    const cx = Math.floor(Number(goalCell.cx));
    const cy = Math.floor(Number(goalCell.cy));
    const tolCells = Math.max(0, Math.floor(Number(goalCell.tolCells || 0)));
    if (!Number.isFinite(cx) || !Number.isFinite(cy)) { this.goal = null; return; }
    this.goal = { cx, cy, tolCells };
  }

  setPlannedPath(pointsWorld) {
    if (!Array.isArray(pointsWorld)) { this.plannedPath = []; return; }
    // WHY: Shallow-validate; renderer will ignore NaNs.
    this.plannedPath = pointsWorld;
  }

  draw() {
    const ctx = this.ctx;

    // WHY: HiDPI-safe resize: draw in CSS pixels but back the canvas with device pixels.
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

    // WHY: Fit map into canvas with padding.
    const pad = Math.max(18, Math.floor(Math.min(width, height) * 0.045));
    const mCm = Math.max(0, this.viewMarginCm || 0);
    const viewW_cm = g.mapW_cm + 2 * mCm;
    const viewH_cm = g.mapH_cm + 2 * mCm;
    const sx = (width - 2 * pad) / viewW_cm;
    const sy = (height - 2 * pad) / viewH_cm;
    // WHY: Real scale (uniform): 1 cm maps to s pixels in both axes.
    const s = Math.min(sx, sy);

    const viewWpx = viewW_cm * s;
    const viewHpx = viewH_cm * s;
    const ox = pad + (width - 2 * pad - viewWpx) * 0.5;
    const oy = pad + (height - 2 * pad - viewHpx) * 0.5;

    const toPx = (x_cm, y_cm) => {
    // WHY: Convert world bottom-left coordinates to canvas top-left coordinates.
      return { x: ox + (x_cm + mCm) * s, y: oy + viewHpx - (y_cm + mCm) * s };
    };

    // WHY: background
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "rgba(7,3,15,0.12)";
    ctx.fillRect(0, 0, width, height);

    // WHY: map border
    ctx.strokeStyle = "rgba(203,215,255,0.30)";
    ctx.lineWidth = 2;
    // WHY: Border is the true map extents within the view.
    ctx.strokeRect(ox + mCm * s, oy + mCm * s, g.mapW_cm * s, g.mapH_cm * s);

    // WHY: Major guide grid is approximately 10 cm.
    const majorEvery = Math.max(1, Math.floor(10 / g.cell_cm));
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

    // WHY: hit cells (true squares, uniform scale)
    const maxV = 35;
    for (let cy = 0; cy < g.h; cy++) {
      for (let cx = 0; cx < g.w; cx++) {
        const v = g.cellLogOdds(cx, cy);
        const st = g.cellState(cx, cy);
        if (st === "unk") continue;

        // WHY: Hard walls: always render at maximum obstacle strength.
        if (g.isLocked && g.isLocked(cx, cy)) {
          ctx.fillStyle = "rgba(88,243,255,0.95)";
          const x_cm = cx * g.cell_cm;
          const y_cm = cy * g.cell_cm;
          // CONTRACT: Cell fill uses top-left world corner in canvas projection.
          const p = toPx(x_cm, y_cm + g.cell_cm);
          const sz = g.cell_cm * s;
          const inset = Math.min(1.2, sz * 0.08);
          ctx.fillRect(p.x + inset, p.y + inset, sz - 2 * inset, sz - 2 * inset);
          continue;
        }

        // WHY: Map confidence to alpha based on magnitude of log-odds.
        const mag = Math.min(maxV, Math.abs(v));
        const t = clamp(mag / maxV, 0, 1);
        const alpha = 0.08 + 0.55 * t;

        // WHY: OCC uses cyan/teal and FREE uses green for quick semantic reading.
        if (st === "occ") ctx.fillStyle = `rgba(88,243,255,${alpha.toFixed(3)})`;
        else ctx.fillStyle = `rgba(90,255,140,${alpha.toFixed(3)})`;

        const x_cm = cx * g.cell_cm;
        const y_cm = cy * g.cell_cm;
        // CONTRACT: Cell fill uses top-left world corner in canvas projection.
        const p = toPx(x_cm, y_cm + g.cell_cm);
        const sz = g.cell_cm * s;
        // WHY: Slight inset so dense regions look cleaner.
        const inset = Math.min(1.2, sz * 0.08);
        ctx.fillRect(p.x + inset, p.y + inset, sz - 2 * inset, sz - 2 * inset);
      }
    }

    // SECTION: Virtual obstacle overlay.
    if (this.virtualBlocked && this.virtualBlocked.length === g.w * g.h) {
      // WHY: Virtual hard-blocks render as strong red for planner/debug visibility.
      ctx.fillStyle = "rgba(255, 70, 70, 0.90)";
      const sz = g.cell_cm * s;
      const inset = Math.min(1.4, sz * 0.10);
      for (let cy = 0; cy < g.h; cy++) {
        for (let cx = 0; cx < g.w; cx++) {
          const i = cy * g.w + cx;
          if (!this.virtualBlocked[i]) continue;
          const x_cm = cx * g.cell_cm;
          const y_cm = cy * g.cell_cm;
          // CONTRACT: Cell fill uses top-left world corner in canvas projection.
          const p = toPx(x_cm, y_cm + g.cell_cm);
          ctx.fillRect(p.x + inset, p.y + inset, sz - 2 * inset, sz - 2 * inset);
        }
      }
    }

    // SECTION: Planned path overlay.
    if (this.plannedPath && this.plannedPath.length >= 2) {
      ctx.strokeStyle = "rgba(255,255,255,0.85)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      let moved = false;
      for (const pt of this.plannedPath) {
        const x = Number(pt?.x);
        const y = Number(pt?.y);
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        const p = toPx(x, y);
        if (!moved) { ctx.moveTo(p.x, p.y); moved = true; }
        else ctx.lineTo(p.x, p.y);
      }
      if (moved) ctx.stroke();

      // WHY: Dots improve readability over dense backgrounds.
      ctx.fillStyle = "rgba(255,255,255,0.55)";
      for (const pt of this.plannedPath) {
        const x = Number(pt?.x);
        const y = Number(pt?.y);
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        const p = toPx(x, y);
        ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
      }
    }

    // SECTION: Goal overlay.
    if (this.goal) {
      const { cx, cy, tolCells } = this.goal;
      if (cx >= 0 && cy >= 0 && cx < g.w && cy < g.h) {
        const x_cm = (cx + 0.5) * g.cell_cm;
        const y_cm = (cy + 0.5) * g.cell_cm;
        const p = toPx(x_cm, y_cm);
        const r1 = Math.max(6, 0.42 * g.cell_cm * s);

        if (tolCells > 0) {
          const r_cm = (tolCells + 0.55) * g.cell_cm;
          const r = r_cm * s;
          ctx.strokeStyle = "rgba(255, 216, 92, 0.55)";
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
          ctx.stroke();
        }

        ctx.strokeStyle = "rgba(255, 216, 92, 0.95)";
        ctx.fillStyle = "rgba(255, 216, 92, 0.18)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, r1, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        ctx.strokeStyle = "rgba(255,255,255,0.85)";
        ctx.beginPath();
        ctx.moveTo(p.x - r1, p.y);
        ctx.lineTo(p.x + r1, p.y);
        ctx.moveTo(p.x, p.y - r1);
        ctx.lineTo(p.x, p.y + r1);
        ctx.stroke();

        ctx.font = "12px system-ui, -apple-system, Segoe UI, sans-serif";
        ctx.fillStyle = "rgba(255, 216, 92, 0.95)";
        ctx.fillText("GOAL", p.x + r1 + 6, p.y - r1 - 2);
      }
    }

    // SECTION: Scan-point overlay.
    if (this.showScan && this.scanPts.length) {
      ctx.fillStyle = "rgba(255,95,135,0.85)";
      for (const pt of this.scanPts) {
        const p = toPx(pt.x, pt.y);
        ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
      }
    }

    // SECTION: Robot pose arrow.
    const rp = toPx(this.pose.x, this.pose.y);
    ctx.save();
    ctx.translate(rp.x, rp.y);
    // WHY: Convert north-up heading convention to canvas +X angle convention.
    const ang = ((this.pose.headingDeg - 90) * Math.PI) / 180;
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
