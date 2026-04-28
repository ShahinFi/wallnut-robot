#include "esp_alert_ring.h"
// SECTION: Alert ring buffer transport implementation.

#include <math.h>

namespace esp_alert_ring {

void AlertRing::reset() {
  nextSeq_ = 1;
  head_ = 0;
  count_ = 0;
}

void AlertRing::pushFromLine(const String& line) {
  // CONTRACT: Input format is "EVT:TAG,x,y,h[,extra]".
  const String s = line;
  if (!s.startsWith("EVT:")) return;

  // WHY: Payload starts immediately after "EVT:" prefix.
  const int p0 = 4;
  const int c1 = s.indexOf(',', p0);
  if (c1 < 0) return;
  const int c2 = s.indexOf(',', c1 + 1);
  const int c3 = (c2 >= 0) ? s.indexOf(',', c2 + 1) : -1;
  const int c4 = (c3 >= 0) ? s.indexOf(',', c3 + 1) : -1;
  if (c2 < 0 || c3 < 0) return;

  AlertEvt& e = ring_[head_];
  e.seq = nextSeq_++;

  const String tag = s.substring(p0, c1);
  tag.toCharArray(e.tag, sizeof(e.tag));
  e.tag[sizeof(e.tag) - 1] = '\0';

  e.x_cm = s.substring(c1 + 1, c2).toFloat();
  e.y_cm = s.substring(c2 + 1, c3).toFloat();
  if (c4 < 0) {
    e.heading_deg = s.substring(c3 + 1).toFloat();
    e.hasExtra = false;
    e.extra = NAN;
  } else {
    e.heading_deg = s.substring(c3 + 1, c4).toFloat();
    e.extra = s.substring(c4 + 1).toFloat();
    e.hasExtra = true;
  }

  head_ = (uint16_t)((head_ + 1) % kRingSize);
  if (count_ < kRingSize) count_++;
}

void AlertRing::serveHttp(ESP8266WebServer& server, uint32_t from) const {
  String out;
  out.reserve(1400);

  if (count_ == 0) {
    server.send(200, "text/plain", "");
    return;
  }

  const uint16_t oldestIdx = (uint16_t)((head_ + kRingSize - count_) % kRingSize);
  const uint16_t newestIdx = (uint16_t)((head_ + kRingSize - 1) % kRingSize);
  const uint32_t newestSeq = ring_[newestIdx].seq;
  if (from >= newestSeq) {
    server.send(200, "text/plain", "");
    return;
  }

  uint16_t emitted = 0;
  for (uint16_t i = 0; i < count_; ++i) {
    const uint16_t idx = (uint16_t)((oldestIdx + i) % kRingSize);
    const AlertEvt& e = ring_[idx];
    if (e.seq == 0) continue;
    if (e.seq <= from) continue;

    out += String(e.seq);
    out += "|";
    out += "EVT:";
    out += e.tag;
    out += ",";
    out += String(e.x_cm, 2);
    out += ",";
    out += String(e.y_cm, 2);
    out += ",";
    out += String(e.heading_deg, 1);
    if (e.hasExtra && isfinite(e.extra)) {
      out += ",";
      out += String(e.extra, 1);
    }
    out += "\n";

    emitted++;
    if (out.length() >= 1200) break;
    if ((emitted & 0x1Fu) == 0) yield();
  }

  server.send(200, "text/plain", out);
}

uint32_t AlertRing::newestSeq() const {
  if (count_ == 0) return 0;
  const uint16_t newestIdx = (uint16_t)((head_ + kRingSize - 1) % kRingSize);
  return ring_[newestIdx].seq;
}

}
