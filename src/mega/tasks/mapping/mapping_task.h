#pragma once

#include <Arduino.h>

#include "mapping/correlative_matcher.h"
#include "mapping/frame_conventions.h"
#include "mapping/occupancy_grid.h"
#include "mapping/scan_buffer.h"

namespace mapping {

// ============================================================
// NOTE (Project Direction):
// The main mapping/SLAM and maze-solving logic is intended to run on the
// browser/PC side. This on-board task is kept for debugging and experiments.
//
// If you are implementing "real" mapping: keep Mega focused on scanning +
// telemetry and do map building + scan matching in JS on the browser.
// ============================================================

// MappingTask (endpoint-only): capture a turret sweep into a scan buffer, optionally
// match to the current map (local search around prior), then update the map grid.
class MappingTask {
public:
  struct Config {
    OccupancyGrid::Config grid = {};
    CorrelativeMatcher::Config matcher = {};
    LidarOffsetBodyCm lidarOffset = {};
    bool enableMatching = true;
  };

  enum class State : uint8_t { Idle, Capturing, Matching, Updated };

  MappingTask();

  void setConfig(const Config& cfg);
  const Config& config() const;

  void resetMap();

  void beginCapture(const Pose2D& priorPose);
  void onSample(float angleDeg, float distCm);
  void finishCaptureAndUpdate(const Pose2D& priorPose, Pose2D& poseInOut);
  void finishCaptureAndUpdateInWindow(const CorrelativeMatcher::SearchWindow& w,
                                      const Pose2D& fallbackPose, Pose2D& poseInOut);

  bool capturing() const;
  State state() const;

  const ScanBuffer& lastScan() const;
  const OccupancyGrid& grid() const;
  long lastMatchScore() const;
  bool lastUpdateApplied() const;

  void printSummary() const;
  void printMap(uint8_t threshold = 1) const;

private:
  void finishCaptureAndUpdateImpl_(const Pose2D& fallbackPose, Pose2D& poseInOut,
                                  const CorrelativeMatcher::SearchWindow* wOpt);

  Config cfg_;
  State state_;

  OccupancyGrid grid_;
  ScanBuffer scan_;
  CorrelativeMatcher matcher_;

  uint32_t scanSeq_;
  long lastMatchScore_;
  bool lastUpdateApplied_;
};

}  // namespace mapping
