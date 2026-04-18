#include "mapping/correlative_matcher.h"

#include <math.h>

namespace mapping {

CorrelativeMatcher::CorrelativeMatcher() : cfg_{} {}

void CorrelativeMatcher::setConfig(const Config& cfg) {
  cfg_ = cfg;
  if (!isfinite(cfg_.step_cm) || cfg_.step_cm <= 0.0f) cfg_.step_cm = 2.0f;
  if (!isfinite(cfg_.stepHeadingDeg) || cfg_.stepHeadingDeg <= 0.0f) cfg_.stepHeadingDeg = 2.0f;
  if (!isfinite(cfg_.searchDx_cm) || cfg_.searchDx_cm < 0.0f) cfg_.searchDx_cm = 0.0f;
  if (!isfinite(cfg_.searchDy_cm) || cfg_.searchDy_cm < 0.0f) cfg_.searchDy_cm = 0.0f;
  if (!isfinite(cfg_.searchDHeadingDeg) || cfg_.searchDHeadingDeg < 0.0f) cfg_.searchDHeadingDeg = 0.0f;
}

const CorrelativeMatcher::Config& CorrelativeMatcher::config() const { return cfg_; }

CorrelativeMatcher::Result CorrelativeMatcher::matchWindow(const SearchWindow& w, const ScanBuffer& scan,
                                                           const OccupancyGrid& grid,
                                                           const LidarOffsetBodyCm& lidarOffset) const {
  Result out;
  if (scan.size() == 0) return out;

  float xMin = w.xMin_cm, xMax = w.xMax_cm;
  float yMin = w.yMin_cm, yMax = w.yMax_cm;
  float hMin = w.headingMin_deg, hMax = w.headingMax_deg;
  if (xMax < xMin) { const float t = xMin; xMin = xMax; xMax = t; }
  if (yMax < yMin) { const float t = yMin; yMin = yMax; yMax = t; }
  if (hMax < hMin) { const float t = hMin; hMin = hMax; hMax = t; }

  float stepX = w.stepX_cm;
  float stepY = w.stepY_cm;
  float stepH = w.stepHeading_deg;
  if (!isfinite(stepX) || stepX <= 0.0f) stepX = 2.0f;
  if (!isfinite(stepY) || stepY <= 0.0f) stepY = 2.0f;
  if (!isfinite(stepH) || stepH <= 0.0f) stepH = 2.0f;

  long bestScore = -1;
  Pose2D best = {};

  // Hard safety bound so a bad window can't lock up the loop.
  // (Enough for typical map sizes / search windows.)
  const uint32_t kMaxIters = 200000UL;
  uint32_t iters = 0;

  for (float hdg = hMin; hdg <= hMax + 0.0001f; hdg += stepH) {
    for (float x = xMin; x <= xMax + 0.0001f; x += stepX) {
      for (float y = yMin; y <= yMax + 0.0001f; y += stepY) {
        Pose2D p;
        p.x_cm = x;
        p.y_cm = y;
        p.headingDeg = hdg;
        const long s = scorePose_(p, scan, grid, lidarOffset);
        if (s > bestScore) {
          bestScore = s;
          best = p;
        }
        if (++iters >= kMaxIters) {
          out.ok = (bestScore >= 0);
          out.bestPose = best;
          out.score = bestScore;
          return out;
        }
      }
    }
  }

  out.ok = (bestScore >= 0);
  out.bestPose = best;
  out.score = bestScore;
  return out;
}

CorrelativeMatcher::Result CorrelativeMatcher::match(const Pose2D& prior, const ScanBuffer& scan,
                                                     const OccupancyGrid& grid,
                                                     const LidarOffsetBodyCm& lidarOffset) const {
  SearchWindow w;
  w.xMin_cm = prior.x_cm - cfg_.searchDx_cm;
  w.xMax_cm = prior.x_cm + cfg_.searchDx_cm;
  w.yMin_cm = prior.y_cm - cfg_.searchDy_cm;
  w.yMax_cm = prior.y_cm + cfg_.searchDy_cm;
  w.headingMin_deg = prior.headingDeg - cfg_.searchDHeadingDeg;
  w.headingMax_deg = prior.headingDeg + cfg_.searchDHeadingDeg;
  w.stepX_cm = cfg_.step_cm;
  w.stepY_cm = cfg_.step_cm;
  w.stepHeading_deg = cfg_.stepHeadingDeg;
  return matchWindow(w, scan, grid, lidarOffset);
}

long CorrelativeMatcher::scorePose_(const Pose2D& pose, const ScanBuffer& scan,
                                    const OccupancyGrid& grid, const LidarOffsetBodyCm& lidarOffset) {
  long score = 0;
  // Precompute heading rotation once per pose (AVR sin/cos are expensive).
  const float h = degToRad(pose.headingDeg);
  const float c = cosf(h);
  const float s = sinf(h);
  // Endpoint-only score: sum hit counts at projected endpoints.
  for (uint16_t i = 0; i < scan.size(); ++i) {
    const ScanBuffer::Point& pt = scan.at(i);
    float xb = pt.xb_cm();
    float yb = pt.yb_cm();
    if (!(isfinite(xb) && isfinite(yb))) continue;
    if (xb == 0.0f && yb == 0.0f) continue;
    xb += lidarOffset.x_cm;
    yb += lidarOffset.y_cm;

    // bodyToMap with precomputed (s,c):
    const float xw = pose.x_cm + (s * xb + c * yb);  // East
    const float yw = pose.y_cm + (c * xb - s * yb);  // North

    uint16_t cx = 0, cy = 0;
    if (!grid.worldToCell(xw, yw, cx, cy)) continue;
    score += (long)grid.hitCount(cx, cy);
  }
  return score;
}

}  // namespace mapping
