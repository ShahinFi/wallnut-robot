#pragma once
// SECTION: Scan-event transport buffer.

#include <Arduino.h>
#include <ESP8266WebServer.h>

namespace esp_scan_events {

class ScanEvents {
 public:
  ScanEvents() = default;
  ~ScanEvents();

  bool allocOnce();
  void reset();

  void pushBegin(int dirSign);
  void pushDone();
  void pushCancel();
  void pushSample(float angleDeg, float distCm);

  bool overflowed() const { return overflow_; }
  bool hasData() const { return hasData_ && count_ > 0; }
  uint16_t count() const { return count_; }

  void serveHttp(ESP8266WebServer& server, uint32_t from) const;

 private:
  enum class Kind : uint8_t { Begin = 1, Sample = 2, Done = 3, Cancel = 4 };
  static constexpr uint16_t kMaxEvents = 4096;

  void freeBuf_();

  Kind* kind_ = nullptr;
  // CONTRACT: `dir_` stores +1 / -1 for BEGIN events, otherwise 0.
  int8_t* dir_ = nullptr;
  // CONTRACT: `angleCdeg_` stores centi-degrees in [0..36000].
  uint16_t* angleCdeg_ = nullptr;
  // CONTRACT: `distMm_` stores distance in millimeters.
  uint16_t* distMm_ = nullptr;
  uint16_t cap_ = 0;

  uint32_t nextSeq_ = 1;
  uint16_t count_ = 0;
  bool hasData_ = false;
  bool overflow_ = false;
};

}
