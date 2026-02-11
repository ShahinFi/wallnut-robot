#include "tasks/encoder_calibration/ui/encoder_calibration_ui.h"

#include <math.h>
#include <stdio.h>
#include "display/lcd.h"

void EncoderCalibrationUI::begin() {
  lcdInit();
}

void EncoderCalibrationUI::showIdle() {
  lcdClear();
  lcdWrite(0, 0, "Enc Cal");
  lcdWrite(1, 0, "Send 'k' start");
  lcdWrite(2, 0, "Drive 20cm");
}

void EncoderCalibrationUI::showRunning(float distanceCm, float cmPerPulse, float pulses,
                                       Phase phase) {
  lcdWrite(0, 0, "Enc Cal");

  char line1[21];
  snprintf(line1, sizeof(line1), "Dist:%4dcm", (int)lroundf(distanceCm));
  lcdWrite(1, 0, line1);

  const char* p = "IDLE";
  if (phase == Phase::CheckStart) p = "CHECK";
  if (phase == Phase::Driving)    p = "DRIVE";
  if (phase == Phase::Compute)    p = "COMP";
  if (phase == Phase::Done)       p = "DONE";
  if (phase == Phase::Failed)     p = "FAIL";

  char line2[21];
  snprintf(line2, sizeof(line2), "Phase:%s", p);
  lcdWrite(2, 0, line2);

  if (phase == Phase::Compute || phase == Phase::Done) {
    lcdWrite(3, 0, "cm/p:");
    lcdWriteFloat(3, 5, cmPerPulse, 4, false);
  } else {
    char line3[21];
    snprintf(line3, sizeof(line3), "Pulses:%5d", (int)lroundf(pulses));
    lcdWrite(3, 0, line3);
  }
}

void EncoderCalibrationUI::showFailed(const char* msg) {
  lcdClear();
  lcdWrite(0, 0, "Enc Cal Fail");
  lcdWrite(1, 0, msg ? msg : "");
}

void EncoderCalibrationUI::showSaved(float cmPerPulse) {
  lcdClear();
  lcdWrite(0, 0, "Enc Cal Saved");
  lcdWrite(1, 0, "cm/p:");
  lcdWriteFloat(1, 5, cmPerPulse, 4, false);
}
