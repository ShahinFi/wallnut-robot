#include "mapping/occupancy_grid.h"

#include <math.h>

namespace mapping {

OccupancyGrid::OccupancyGrid() : cfg_{}, wCells_(0), hCells_(0), totalHits_(0) {
  setConfig(cfg_);
  clear();
}

void OccupancyGrid::setConfig(const Config& cfg) {
  cfg_ = cfg;
  if (cfg_.cell_cm == 0) cfg_.cell_cm = 1;
  // Compute cell dims (ceil).
  wCells_ = (uint16_t)((cfg_.mapW_cm + cfg_.cell_cm - 1) / cfg_.cell_cm);
  hCells_ = (uint16_t)((cfg_.mapH_cm + cfg_.cell_cm - 1) / cfg_.cell_cm);
  if (wCells_ > kMaxW) wCells_ = kMaxW;
  if (hCells_ > kMaxH) hCells_ = kMaxH;
  // Default origin center if caller didn't set it.
  if (!isfinite(cfg_.originX_cm) || !isfinite(cfg_.originY_cm)) {
    cfg_.originX_cm = 0.5f * (float)cfg_.mapW_cm;
    cfg_.originY_cm = 0.5f * (float)cfg_.mapH_cm;
  }
}

const OccupancyGrid::Config& OccupancyGrid::config() const { return cfg_; }

uint16_t OccupancyGrid::cellsW() const { return wCells_; }
uint16_t OccupancyGrid::cellsH() const { return hCells_; }

void OccupancyGrid::clear() {
  totalHits_ = 0;
  for (uint16_t y = 0; y < kMaxH; ++y) {
    for (uint16_t x = 0; x < kMaxW; ++x) {
      cells_[y][x] = 0;
    }
  }
}

bool OccupancyGrid::worldToCell(float x_cm, float y_cm, uint16_t& cx, uint16_t& cy) const {
  // Shift by origin so robot pose (0,0) maps into the configured origin position.
  const float gx = x_cm + cfg_.originX_cm;
  const float gy = y_cm + cfg_.originY_cm;
  if (!(isfinite(gx) && isfinite(gy))) return false;
  if (gx < 0.0f || gy < 0.0f) return false;
  const uint16_t ix = (uint16_t)(gx / (float)cfg_.cell_cm);
  const uint16_t iy = (uint16_t)(gy / (float)cfg_.cell_cm);
  if (ix >= wCells_ || iy >= hCells_) return false;
  cx = ix;
  cy = iy;
  return true;
}

void OccupancyGrid::addHitCell(uint16_t cx, uint16_t cy) {
  if (cx >= wCells_ || cy >= hCells_) return;
  uint8_t& v = cells_[cy][cx];
  if (v < 255) v++;
  if (totalHits_ < 0xFFFFFFFFu) totalHits_++;
}

uint8_t OccupancyGrid::hitCount(uint16_t cx, uint16_t cy) const {
  if (cx >= wCells_ || cy >= hCells_) return 0;
  return cells_[cy][cx];
}

uint32_t OccupancyGrid::totalHits() const { return totalHits_; }

void OccupancyGrid::printAscii(uint8_t threshold) const {
  Serial.print("MAP: cells=");
  Serial.print((unsigned)wCells_);
  Serial.print("x");
  Serial.print((unsigned)hCells_);
  Serial.print(" cell_cm=");
  Serial.print((unsigned)cfg_.cell_cm);
  Serial.print(" origin_cm=(");
  Serial.print(cfg_.originX_cm, 1);
  Serial.print(",");
  Serial.print(cfg_.originY_cm, 1);
  Serial.println(")");

  // Print top row first for human viewing.
  for (int y = (int)hCells_ - 1; y >= 0; --y) {
    for (uint16_t x = 0; x < wCells_; ++x) {
      const uint8_t v = cells_[y][x];
      Serial.print(v >= threshold ? '#' : '.');
    }
    Serial.println();
  }
}

}  // namespace mapping
