#pragma once

#include <Arduino.h>

#include "mapping/frame_conventions.h"
#include "mapping/occupancy_grid.h"
#include "mapping/scan_buffer.h"

namespace mapping {

// ============================================================
// NOTE (Project Direction):
// This on-board correlative matcher is kept for debugging experiments only.
// Real scan matching / SLAM should be implemented in the browser/PC where we
// have CPU/RAM and can iterate fast.
// ============================================================

// Thin correlative matcher: local search around a prior using endpoint-only hits grid.
class CorrelativeMatcher {
public:
  struct SearchWindow {
    float xMin_cm = 0.0f;
    float xMax_cm = 0.0f;
    float yMin_cm = 0.0f;
    float yMax_cm = 0.0f;
    float headingMin_deg = 0.0f;
    float headingMax_deg = 0.0f;
    float stepX_cm = 2.0f;
    float stepY_cm = 2.0f;
    float stepHeading_deg = 2.0f;
  };

  struct Config {
    float searchDx_cm = 8.0f;     // +- (cm)
    float searchDy_cm = 8.0f;     // +-
    float step_cm = 2.0f;         // grid step (cm)
    float searchDHeadingDeg = 20.0f;  // +- (deg)
    float stepHeadingDeg = 2.0f;
  };

  struct Result {
    bool  ok = false;
    Pose2D bestPose = {};
    long  score = -1;
  };

  CorrelativeMatcher();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // Core operation: search over an explicit pose window.
  Result matchWindow(const SearchWindow& w, const ScanBuffer& scan,
                     const OccupancyGrid& grid, const LidarOffsetBodyCm& lidarOffset) const;

  // Convenience: local search around a prior using Config (+/- dx/dy/dheading).
  Result match(const Pose2D& prior, const ScanBuffer& scan,
               const OccupancyGrid& grid, const LidarOffsetBodyCm& lidarOffset) const;

private:
  static long scorePose_(const Pose2D& pose, const ScanBuffer& scan,
                         const OccupancyGrid& grid, const LidarOffsetBodyCm& lidarOffset);

  Config cfg_;
};

}  // namespace mapping
