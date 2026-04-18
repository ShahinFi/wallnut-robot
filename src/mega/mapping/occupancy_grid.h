#pragma once

#include <Arduino.h>

namespace mapping {

// ============================================================
// NOTE (Project Direction):
// On-board mapping is NOT the primary architecture anymore.
// This grid is kept for debug/experimentation only.
//
// The browser/PC should own:
// - map representation,
// - scan matching / pose correction,
// - maze-solving decisions.
// ============================================================

// Endpoint-only occupancy grid (hit counts per cell).
//
// Map axes:
// - X: +East
// - Y: +North
//
// We keep origin (0,0) at the grid's bottom-left in map coordinates.
class OccupancyGrid {
public:
  struct Config {
    uint16_t mapW_cm = 46;
    uint16_t mapH_cm = 26;
    uint8_t  cell_cm = 2;

    // Where (0,0) of the robot pose lands in the grid, in cm from bottom-left.
    // Default: center of the map.
    float originX_cm = 23.0f;
    float originY_cm = 13.0f;
  };

  // Hard cap to keep SRAM bounded on Mega.
  // With endpoint-only mapping we don't need a huge grid; keeping this small
  // avoids SRAM exhaustion (Mega has 8KB total, shared with all tasks/globals).
  static const uint16_t kMaxW = 24;
  static const uint16_t kMaxH = 24;

  OccupancyGrid();

  void setConfig(const Config& cfg);
  const Config& config() const;

  uint16_t cellsW() const;
  uint16_t cellsH() const;

  void clear();

  // Convert map cm coords to cell indices. Returns false if out of bounds.
  bool worldToCell(float x_cm, float y_cm, uint16_t& cx, uint16_t& cy) const;

  // Adds an occupied hit to a cell (saturating counter).
  void addHitCell(uint16_t cx, uint16_t cy);

  uint8_t hitCount(uint16_t cx, uint16_t cy) const;
  uint32_t totalHits() const;

  // Debug: ASCII dump to Serial ('.' empty, '#' occupied by threshold).
  void printAscii(uint8_t threshold = 1) const;

private:
  Config cfg_;
  uint16_t wCells_;
  uint16_t hCells_;
  uint32_t totalHits_;
  uint8_t cells_[kMaxH][kMaxW];  // [y][x]
};

}  // namespace mapping
