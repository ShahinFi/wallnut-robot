#include "tasks/room_measure/collect/room_scan_collector.h"

#include <math.h>

RoomScanCollector::RoomScanCollector() : cfg_{}, binCount_(0) {
  reset();
}

void RoomScanCollector::setConfig(const Config& cfg) {
  cfg_ = cfg;
  reset();
}

const RoomScanCollector::Config& RoomScanCollector::config() const { return cfg_; }

void RoomScanCollector::reset() {
  uint16_t bs = cfg_.binSizeDeg;
  if (bs < 1) bs = 1;
  if (bs > 90) bs = 90;

  binCount_ = 360 / bs;
  if (binCount_ < 4) binCount_ = 4;
  if (binCount_ > kMaxBins) binCount_ = kMaxBins;

  for (uint16_t i = 0; i < binCount_; ++i) binMinCm_[i] = INFINITY;
}

float RoomScanCollector::wrapDeg360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

uint16_t RoomScanCollector::wrapIndex(int idx) const {
  if (binCount_ == 0) return 0;
  int m = idx % (int)binCount_;
  if (m < 0) m += binCount_;
  return (uint16_t)m;
}

float RoomScanCollector::smoothedAt(uint16_t idx) const {
  if (binCount_ == 0) return INFINITY;
  const float a = binMinCm_[wrapIndex((int)idx - 1)];
  const float b = binMinCm_[idx];
  const float c = binMinCm_[wrapIndex((int)idx + 1)];

  float sum = 0.0f;
  int count = 0;
  if (isfinite(a)) { sum += a; ++count; }
  if (isfinite(b)) { sum += b; ++count; }
  if (isfinite(c)) { sum += c; ++count; }

  if (count == 0) return INFINITY;
  return sum / (float)count;
}

bool RoomScanCollector::isLocalMin(uint16_t idx) const {
  const float v = smoothedAt(idx);
  if (!isfinite(v)) return false;

  const float vPrev = smoothedAt(wrapIndex((int)idx - 1));
  const float vNext = smoothedAt(wrapIndex((int)idx + 1));

  if (!isfinite(vPrev) && !isfinite(vNext)) return false;
  if (isfinite(vPrev) && v >= vPrev) return false;
  if (isfinite(vNext) && v >= vNext) return false;
  return true;
}

void RoomScanCollector::push(float relSweepDeg, float lidarAvgCm) {
  if (lidarAvgCm < cfg_.minValidCm || lidarAvgCm > cfg_.maxValidCm) return;

  uint16_t bs = cfg_.binSizeDeg;
  if (bs < 1) bs = 1;

  const float a = wrapDeg360(relSweepDeg);
  uint16_t idx = (uint16_t)(a / (float)bs);
  if (idx >= binCount_) idx = binCount_ - 1;

  if (lidarAvgCm < binMinCm_[idx]) binMinCm_[idx] = lidarAvgCm;
}

bool RoomScanCollector::quadrantLocalMinima(float outWallCm[4]) const {
  if (binCount_ == 0) return false;

  uint16_t bs = cfg_.binSizeDeg;
  if (bs < 1) bs = 1;

  const uint16_t binsPerQuadrant = (90 / bs);
  if (binsPerQuadrant < 1) return false;

  for (int q = 0; q < 4; ++q) {
    const uint16_t startBin = (uint16_t)(q * binsPerQuadrant);
    uint16_t endBin = (uint16_t)((q + 1) * binsPerQuadrant);
    if (endBin > binCount_) endBin = binCount_;
    if (startBin >= endBin) return false;

    int bestIdx = -1;
    float bestVal = INFINITY;
    const int center = (int)startBin + ((int)(endBin - startBin) / 2);
    int bestCenterDist = 0;

    for (uint16_t i = startBin; i < endBin; ++i) {
      if (!isLocalMin(i)) continue;
      const float v = smoothedAt(i);
      if (!isfinite(v)) continue;

      if (cfg_.pickMode == MinPickMode::ClosestToCenter) {
        const int dist = abs((int)i - center);
        if (bestIdx < 0 || dist < bestCenterDist ||
            (dist == bestCenterDist && v < bestVal)) {
          bestIdx = (int)i;
          bestVal = v;
          bestCenterDist = dist;
        }
      } else {
        if (bestIdx < 0 || v < bestVal) {
          bestIdx = (int)i;
          bestVal = v;
        }
      }
    }

    if (bestIdx < 0 || !isfinite(bestVal)) return false;
    outWallCm[q] = bestVal;
  }

  return true;
}
