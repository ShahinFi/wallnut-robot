#include "esp_scan_events.h"
// (Refactor) Transport module implementation.

#include <math.h>
#include <stdlib.h>

namespace esp_scan_events {

ScanEvents::~ScanEvents() { freeBuf_(); }

void ScanEvents::freeBuf_() {
  if (kind_) { free(kind_); kind_ = nullptr; }
  if (dir_) { free(dir_); dir_ = nullptr; }
  if (angleCdeg_) { free(angleCdeg_); angleCdeg_ = nullptr; }
  if (distMm_) { free(distMm_); distMm_ = nullptr; }
  cap_ = 0;
}

bool ScanEvents::allocOnce() {
  if (cap_ > 0 && kind_ && dir_ && angleCdeg_ && distMm_) return true;

  freeBuf_();
  kind_ = (Kind*)malloc((size_t)kMaxEvents * sizeof(Kind));
  dir_ = (int8_t*)malloc((size_t)kMaxEvents * sizeof(int8_t));
  angleCdeg_ = (uint16_t*)malloc((size_t)kMaxEvents * sizeof(uint16_t));
  distMm_ = (uint16_t*)malloc((size_t)kMaxEvents * sizeof(uint16_t));
  if (!(kind_ && dir_ && angleCdeg_ && distMm_)) {
    freeBuf_();
    return false;
  }
  cap_ = kMaxEvents;
  return true;
}

void ScanEvents::reset() {
  nextSeq_ = 1;
  count_ = 0;
  hasData_ = false;
  overflow_ = false;
}

void ScanEvents::pushBegin(int dirSign) {
  if (!cap_ || !kind_ || !dir_ || !angleCdeg_ || !distMm_) { overflow_ = true; return; }
  if (count_ >= cap_) { overflow_ = true; return; }
  kind_[count_] = Kind::Begin;
  dir_[count_] = (dirSign < 0) ? -1 : 1;
  angleCdeg_[count_] = 0;
  distMm_[count_] = 0;
  count_++;
  nextSeq_++;
  hasData_ = true;
}

void ScanEvents::pushDone() {
  if (!cap_ || !kind_ || !dir_ || !angleCdeg_ || !distMm_) { overflow_ = true; return; }
  if (count_ >= cap_) { overflow_ = true; return; }
  kind_[count_] = Kind::Done;
  dir_[count_] = 0;
  angleCdeg_[count_] = 0;
  distMm_[count_] = 0;
  count_++;
  nextSeq_++;
  hasData_ = true;
}

void ScanEvents::pushCancel() {
  if (!cap_ || !kind_ || !dir_ || !angleCdeg_ || !distMm_) { overflow_ = true; return; }
  if (count_ >= cap_) { overflow_ = true; return; }
  kind_[count_] = Kind::Cancel;
  dir_[count_] = 0;
  angleCdeg_[count_] = 0;
  distMm_[count_] = 0;
  count_++;
  nextSeq_++;
  hasData_ = true;
}

void ScanEvents::pushSample(float angleDeg, float distCm) {
  if (!cap_ || !kind_ || !dir_ || !angleCdeg_ || !distMm_) { overflow_ = true; return; }
  if (count_ >= cap_) { overflow_ = true; return; }
  if (!(isfinite(angleDeg) && isfinite(distCm))) return;

  while (angleDeg < 0.0f) angleDeg += 360.0f;
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  int a = (int)lroundf(angleDeg * 100.0f);
  if (a < 0) a = 0;
  if (a > 36000) a = 36000;

  int dmm = (int)lroundf(distCm * 10.0f);
  if (dmm < 0) dmm = 0;
  if (dmm > 65535) dmm = 65535;

  kind_[count_] = Kind::Sample;
  dir_[count_] = 0;
  angleCdeg_[count_] = (uint16_t)a;
  distMm_[count_] = (uint16_t)dmm;
  count_++;
  nextSeq_++;
  hasData_ = true;
}

void ScanEvents::serveHttp(ESP8266WebServer& server, uint32_t from) const {
  // External wire format preserved: lines of "seq|TSCAN:...".
  String out;
  out.reserve(2400);

  if (overflow_) {
    out += String(from + 1);
    out += "|TSCAN:OVERFLOW\n";
    server.send(200, "text/plain", out);
    return;
  }

  if (!hasData_ || count_ == 0) {
    server.send(200, "text/plain", "");
    return;
  }

  if (from >= (uint32_t)count_) {
    server.send(200, "text/plain", "");
    return;
  }

  const uint16_t startIdx = (from > 0) ? (uint16_t)from : 0u;  // from=N => start at seq N+1 => index N
  for (uint16_t i = startIdx; i < count_; ++i) {
    const uint32_t seq = (uint32_t)i + 1u;

    out += String(seq);
    out += "|";

    const Kind k = kind_[i];
    if (k == Kind::Begin) {
      out += "TSCAN:BEGIN,";
      out += (dir_[i] < 0) ? "-" : "+";
      out += "\n";
    } else if (k == Kind::Done) {
      out += "TSCAN:DONE\n";
    } else if (k == Kind::Cancel) {
      out += "TSCAN:CANCEL\n";
    } else if (k == Kind::Sample) {
      const float a = ((float)angleCdeg_[i]) / 100.0f;
      const float d = ((float)distMm_[i]) / 10.0f;
      out += "TSCAN:";
      out += String(a, 2);
      out += ",";
      out += String(d, 1);
      out += "\n";
    } else {
      out += "\n";
    }

    if (out.length() >= 1800) break;
    if ((i & 0x3Fu) == 0) yield();
  }

  server.send(200, "text/plain", out);
}

}  // namespace esp_scan_events
