#pragma once
// SECTION: Alert ring buffer transport state.

#include <Arduino.h>
#include <ESP8266WebServer.h>

namespace esp_alert_ring {

class AlertRing {
 public:
  void reset();
  void pushFromLine(const String& line);
  void serveHttp(ESP8266WebServer& server, uint32_t from) const;
  uint32_t newestSeq() const;

 private:
  static constexpr uint16_t kRingSize = 256;

  struct AlertEvt {
    uint32_t seq;
    char tag[12];
    float x_cm;
    float y_cm;
    float heading_deg;
    float extra;
    bool hasExtra;
  };

  AlertEvt ring_[kRingSize]{};
  uint32_t nextSeq_ = 1;
  // CONTRACT: `head_` always points to the next write index in the ring.
  uint16_t head_ = 0;
  // CONTRACT: `count_` is the number of valid entries currently stored.
  uint16_t count_ = 0;
};

}
