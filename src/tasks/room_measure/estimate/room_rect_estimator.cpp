#include "tasks/room_measure/estimate/room_rect_estimator.h"

#include <math.h>

RoomRectEstimateOutput RoomRectEstimator::compute(const RoomRectEstimateInput& in) {
  RoomRectEstimateOutput out = {};
  out.valid = false;

  for (int i = 0; i < 4; ++i) {
    if (!isfinite(in.wallDistanceCm[i]) || in.wallDistanceCm[i] <= 0.0f) return out;
  }

  const float off = in.lidarToCenterOffsetCm;

  // Opposites: (0 with 2), (1 with 3)
  out.widthCm  = (in.wallDistanceCm[0] + off) + (in.wallDistanceCm[2] + off);
  out.lengthCm = (in.wallDistanceCm[1] + off) + (in.wallDistanceCm[3] + off);

  if (!(out.widthCm > 0.0f) || !(out.lengthCm > 0.0f)) return out;

  const float widthM  = out.widthCm  * 0.01f;
  const float lengthM = out.lengthCm * 0.01f;
  const float heightM = in.ceilingHeightCm * 0.01f;

  out.areaM2   = widthM * lengthM;
  out.volumeM3 = out.areaM2 * heightM;

  out.valid = true;
  return out;
}
