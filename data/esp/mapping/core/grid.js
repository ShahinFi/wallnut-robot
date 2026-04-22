export class HitGrid {
  constructor(mapW_cm, mapH_cm, cell_cm) {
    this.mapW_cm = mapW_cm;
    this.mapH_cm = mapH_cm;
    this.cell_cm = cell_cm;
    this.w = Math.ceil(mapW_cm / cell_cm);
    this.h = Math.ceil(mapH_cm / cell_cm);
    // Log-odds style occupancy grid (browser-side, 3-state: UNKNOWN/FREE/OCC).
    // 0 ~= UNKNOWN, positive ~= OCC, negative ~= FREE.
    this.cells = new Int16Array(this.w * this.h);

    // Tunable thresholds (kept as properties so tests/experiments can adjust).
    this.occThreshold = 8;
    this.freeThreshold = -8;
    this.minLogOdds = -80;
    this.maxLogOdds = 80;
  }

  clear() {
    this.cells.fill(0);
  }

  _idx(cx, cy) {
    return cy * this.w + cx;
  }

  worldToCell(x_cm, y_cm) {
    if (!Number.isFinite(x_cm) || !Number.isFinite(y_cm)) return null;
    if (x_cm < 0 || y_cm < 0) return null;
    if (x_cm >= this.mapW_cm || y_cm >= this.mapH_cm) return null;
    const cx = Math.floor(x_cm / this.cell_cm);
    const cy = Math.floor(y_cm / this.cell_cm);
    if (cx < 0 || cy < 0 || cx >= this.w || cy >= this.h) return null;
    return { cx, cy };
  }

  addHitWorld(x_cm, y_cm) {
    const c = this.worldToCell(x_cm, y_cm);
    if (!c) return false;
    this.addOccupied(c.cx, c.cy, 12);
    return true;
  }

  hit(cx, cy) {
    if (cx < 0 || cy < 0 || cx >= this.w || cy >= this.h) return 0;
    const v = this.cells[this._idx(cx, cy)];
    // Renderer expects a non-negative intensity. Show only occupied strength.
    return v > 0 ? v : 0;
  }

  cellLogOdds(cx, cy) {
    if (cx < 0 || cy < 0 || cx >= this.w || cy >= this.h) return 0;
    return this.cells[this._idx(cx, cy)];
  }

  cellState(cx, cy) {
    const v = this.cellLogOdds(cx, cy);
    if (v >= this.occThreshold) return "occ";
    if (v <= this.freeThreshold) return "free";
    return "unk";
  }

  setOccupied(cx, cy, logOdds = 60) {
    if (cx < 0 || cy < 0 || cx >= this.w || cy >= this.h) return;
    const i = this._idx(cx, cy);
    this.cells[i] = Math.max(this.cells[i], Math.min(this.maxLogOdds, Math.trunc(logOdds)));
  }

  addOccupied(cx, cy, delta = 12) {
    if (cx < 0 || cy < 0 || cx >= this.w || cy >= this.h) return;
    const i = this._idx(cx, cy);
    const v = this.cells[i] + Math.trunc(delta);
    this.cells[i] = Math.max(this.minLogOdds, Math.min(this.maxLogOdds, v));
  }

  addFree(cx, cy, delta = 4) {
    if (cx < 0 || cy < 0 || cx >= this.w || cy >= this.h) return;
    const i = this._idx(cx, cy);
    const v = this.cells[i] - Math.trunc(delta);
    this.cells[i] = Math.max(this.minLogOdds, Math.min(this.maxLogOdds, v));
  }
}
