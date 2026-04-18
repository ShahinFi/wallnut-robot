#include "tasks/mapping/mapping_task.h"

#include "mapping/frame_conventions.h"

#include <math.h>

namespace mapping {

MappingTask::MappingTask()
: cfg_{},
  state_(State::Idle),
  grid_(),
  scan_(),
  matcher_(),
  scanSeq_(0),
  lastMatchScore_(-1),
  lastUpdateApplied_(false) {
  grid_.setConfig(cfg_.grid);
  matcher_.setConfig(cfg_.matcher);
  grid_.clear();
}

void MappingTask::setConfig(const Config& cfg) {
  cfg_ = cfg;
  grid_.setConfig(cfg_.grid);
  matcher_.setConfig(cfg_.matcher);
}

const MappingTask::Config& MappingTask::config() const { return cfg_; }

void MappingTask::resetMap() {
  grid_.clear();
  scan_.reset();
  scanSeq_ = 0;
  lastMatchScore_ = -1;
  lastUpdateApplied_ = false;
  state_ = State::Idle;
}

void MappingTask::beginCapture(const Pose2D& priorPose) {
  (void)priorPose;
  scan_.reset();
  scanSeq_++;
  lastMatchScore_ = -1;
  lastUpdateApplied_ = false;
  state_ = State::Capturing;
}

void MappingTask::onSample(float angleDeg, float distCm) {
  if (state_ != State::Capturing) return;
  // Keep raw samples; filtering/downsampling is handled by the scan producer.
  scan_.push(angleDeg, distCm);
}

void MappingTask::finishCaptureAndUpdate(const Pose2D& priorPose, Pose2D& poseInOut) {
  finishCaptureAndUpdateImpl_(priorPose, poseInOut, nullptr);
}

void MappingTask::finishCaptureAndUpdateInWindow(const CorrelativeMatcher::SearchWindow& w,
                                                 const Pose2D& fallbackPose, Pose2D& poseInOut) {
  finishCaptureAndUpdateImpl_(fallbackPose, poseInOut, &w);
}

void MappingTask::finishCaptureAndUpdateImpl_(const Pose2D& fallbackPose, Pose2D& poseInOut,
                                              const CorrelativeMatcher::SearchWindow* wOpt) {
  if (state_ != State::Capturing) return;
  state_ = State::Matching;

  Pose2D usedPose = fallbackPose;
  const bool mapWasEmpty = (grid_.totalHits() == 0);
  bool matchOk = false;
  if (cfg_.enableMatching && grid_.totalHits() > 0) {
    const CorrelativeMatcher::Result r =
      wOpt ? matcher_.matchWindow(*wOpt, scan_, grid_, cfg_.lidarOffset)
           : matcher_.match(fallbackPose, scan_, grid_, cfg_.lidarOffset);
    if (r.ok) {
      usedPose = r.bestPose;
      lastMatchScore_ = r.score;
      matchOk = true;
    }
  }

  // Update in/out pose with the pose we used.
  poseInOut = usedPose;

  // IMPORTANT:
  // If the map already has content and matching was enabled but failed, do NOT
  // apply the scan update. Otherwise we would pollute a good map with endpoints
  // stamped at a wrong pose.
  //
  // We still allow updating when the map is empty (bootstrapping).
  if (!mapWasEmpty && cfg_.enableMatching && !matchOk) {
    lastUpdateApplied_ = false;
    state_ = State::Updated;
    return;
  }

  // Precompute heading rotation once (AVR sin/cos are expensive).
  const float h = degToRad(usedPose.headingDeg);
  const float c = cosf(h);
  const float s = sinf(h);

  // Map update: endpoint-only hits.
  for (uint16_t i = 0; i < scan_.size(); ++i) {
    const ScanBuffer::Point& pt = scan_.at(i);
    float xb = pt.xb_cm();
    float yb = pt.yb_cm();
    if (!(isfinite(xb) && isfinite(yb))) continue;
    if (xb == 0.0f && yb == 0.0f) continue;
    xb += cfg_.lidarOffset.x_cm;
    yb += cfg_.lidarOffset.y_cm;

    const float xw = usedPose.x_cm + (s * xb + c * yb);  // East
    const float yw = usedPose.y_cm + (c * xb - s * yb);  // North

    uint16_t cx = 0, cy = 0;
    if (!grid_.worldToCell(xw, yw, cx, cy)) continue;
    grid_.addHitCell(cx, cy);
  }

  lastUpdateApplied_ = true;
  state_ = State::Updated;
}

bool MappingTask::capturing() const { return state_ == State::Capturing; }
MappingTask::State MappingTask::state() const { return state_; }

const ScanBuffer& MappingTask::lastScan() const { return scan_; }
const OccupancyGrid& MappingTask::grid() const { return grid_; }
long MappingTask::lastMatchScore() const { return lastMatchScore_; }
bool MappingTask::lastUpdateApplied() const { return lastUpdateApplied_; }

void MappingTask::printSummary() const {
  Serial.print("MAP_TASK: scan_points=");
  Serial.print((unsigned)scan_.size());
  Serial.print(" match_score=");
  Serial.print((long)lastMatchScore_);
  Serial.print(" total_hits=");
  Serial.print((unsigned long)grid_.totalHits());
  Serial.print(" grid_cells=");
  Serial.print((unsigned)grid_.cellsW());
  Serial.print("x");
  Serial.println((unsigned)grid_.cellsH());
}

void MappingTask::printMap(uint8_t threshold) const {
  grid_.printAscii(threshold);
}

}  // namespace mapping
