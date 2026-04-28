#include "color_cal_button.h"

#include "motor/motor.h"

#include "motion_control.h"

static const int kButtonPin = 19;
static const uint32_t kButtonDebounceMs = 250;

extern RuntimeState gRt;

void onButtonIsr() {
  gRt.buttonEdge = true;
}

bool handleColorCalButton(RuntimeState& rt, uint32_t nowMs) {
  if (!rt.buttonEdge) return false;
  if (nowMs - rt.lastButtonMs < kButtonDebounceMs) return false;
  rt.buttonEdge = false;
  if (digitalRead(kButtonPin) != LOW) return false;
  rt.lastButtonMs = nowMs;
  if (!rt.colorCalTask.active()) {
    cancelActiveMotion(rt);
    motorDrive(0.0f, 0.0f);
    rt.colorCalTask.begin();
  } else {
    rt.colorCalTask.onButtonPress(&rt.lastRgb, rt.lastRgbValid);
  }
  return true;
}

bool tickColorCalibrationTask(RuntimeState& rt) {
  if (!rt.colorCalTask.active()) return false;
  rt.colorCalTask.update(&rt.lastRgb, rt.lastRgbValid);
  if (rt.colorCalTask.consumeJustSaved()) {
    const ColorRgb* refs = rt.colorCalTask.refs();
    if (refs) {
      Serial2.print("RGBREF:");
      for (uint8_t i = 0; i < ColorCalibrationTask::kColorCount; i++) {
        if (i) Serial2.print(";");
        Serial2.print((int)refs[i].r);
        Serial2.print(",");
        Serial2.print((int)refs[i].g);
        Serial2.print(",");
        Serial2.print((int)refs[i].b);
      }
      Serial2.println();
    }
  }
  return true;
}

