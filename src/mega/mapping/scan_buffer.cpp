#include "mapping/scan_buffer.h"


#include <math.h>

namespace mapping {

ScanBuffer::ScanBuffer() : n_(0) {}

void ScanBuffer::reset() { n_ = 0; }

bool ScanBuffer::push(float angleDeg, float distCm) {
  if (n_ >= kMaxPoints) return false;

  if (!(isfinite(angleDeg) && isfinite(distCm) && distCm > 0.0f)) {
    pts_[n_].xb_mm = 0;
    pts_[n_].yb_mm = 0;
    n_++;
    return true;
  }

  while (angleDeg < 0.0f) angleDeg += 360.0f;
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;

  const float a = angleDeg * (3.14159265358979323846f / 180.0f);
  const float xb_cm = distCm * cosf(a);
  const float yb_cm = distCm * sinf(a);

  long xb_mm = lroundf(xb_cm * 10.0f);
  long yb_mm = lroundf(yb_cm * 10.0f);
  if (xb_mm < -32768L) xb_mm = -32768L;
  if (xb_mm > 32767L) xb_mm = 32767L;
  if (yb_mm < -32768L) yb_mm = -32768L;
  if (yb_mm > 32767L) yb_mm = 32767L;

  pts_[n_].xb_mm = (int16_t)xb_mm;
  pts_[n_].yb_mm = (int16_t)yb_mm;
  n_++;
  return true;
}

uint16_t ScanBuffer::size() const { return n_; }

const ScanBuffer::Point& ScanBuffer::at(uint16_t i) const { return pts_[i]; }

}  // namespace mapping
