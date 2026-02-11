#include "tasks/wall_sequence/ui/wall_sequence_ui.h"

#include <math.h>
#include <stdio.h>
#include "display/lcd.h"

void WallSequenceUI::begin() {
  lcdInit();
}

void WallSequenceUI::showIdle() {
  lcdClear();
  lcdWrite(0, 0, "Wall Seq");
  lcdWrite(1, 0, "Send 's' start");
  lcdWrite(2, 0, "Targets:30/15");
}

void WallSequenceUI::showRunning(float distanceCm, float totalCm, Phase phase) {
  lcdWrite(0, 0, "Wall Seq");

  char line1[21];
  snprintf(line1, sizeof(line1), "Dist:%4dcm",
           (int)lroundf(distanceCm));
  lcdWrite(1, 0, line1);

  const char* p = "IDLE";
  if (phase == Phase::Approach30_1) p = "A30-1";
  if (phase == Phase::TurnLeft_1)  p = "TL-1";
  if (phase == Phase::Approach15_2) p = "A15-2";
  if (phase == Phase::TurnLeft_2)  p = "TL-2";
  if (phase == Phase::Approach30_3) p = "A30-3";
  if (phase == Phase::TurnLeft_3)  p = "TL-3";
  if (phase == Phase::Approach15_4) p = "A15-4";
  if (phase == Phase::Done)        p = "DONE";
  if (phase == Phase::Failed)      p = "FAIL";

  char line2[21];
  snprintf(line2, sizeof(line2), "Phase:%s", p);
  lcdWrite(2, 0, line2);

  char line3[21];
  snprintf(line3, sizeof(line3), "Total:%5dcm",
           (int)lroundf(totalCm));
  lcdWrite(3, 0, line3);
}

void WallSequenceUI::showFailed(const char* msg) {
  lcdClear();
  lcdWrite(0, 0, "Wall Seq Fail");
  lcdWrite(1, 0, msg ? msg : "");
}
