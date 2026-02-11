#include "tasks/wall_align_90/ui/wall_align_90_ui.h"

#include <math.h>
#include <stdio.h>
#include "display/lcd.h"

void WallAlign90UI::begin() {
  lcdInit();
}

void WallAlign90UI::showIdle() {
  lcdClear();
  lcdWrite(0, 0, "Wall Align 90");
  lcdWrite(1, 0, "Send 'w' start");
  lcdWrite(2, 0, "Target: 15cm");
}

void WallAlign90UI::showRunning(float distanceCm, Phase phase) {
  char line0[21];
  snprintf(line0, sizeof(line0), "Wall Align 90");

  char line1[21];
  snprintf(line1, sizeof(line1), "Dist:%4dcm", (int)lroundf(distanceCm));

  const char* p = "IDLE";
  if (phase == Phase::Check)      p = "CHECK";
  if (phase == Phase::Approach1)  p = "APP1";
  if (phase == Phase::Turn)       p = "TURN";
  if (phase == Phase::Approach2)  p = "APP2";
  if (phase == Phase::Done)       p = "DONE";
  if (phase == Phase::Failed)     p = "FAIL";

  char line2[21];
  snprintf(line2, sizeof(line2), "Phase:%s", p);

  lcdWrite(0, 0, line0);
  lcdWrite(1, 0, line1);
  lcdWrite(2, 0, line2);
}

void WallAlign90UI::showFailed(const char* msg) {
  lcdClear();
  lcdWrite(0, 0, "Wall Align Fail");
  lcdWrite(1, 0, msg ? msg : "");
}
