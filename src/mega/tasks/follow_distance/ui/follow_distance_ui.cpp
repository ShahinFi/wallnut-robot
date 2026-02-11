#include "tasks/follow_distance/ui/follow_distance_ui.h"

#include <math.h>
#include <stdio.h>
#include "display/lcd.h"

void FollowDistanceUI::begin() {
  lcdInit();
}

void FollowDistanceUI::showIdle(float targetCm) {
  lcdClear();
  lcdWrite(0, 0, "Follow Distance");
  char line1[21];
  snprintf(line1, sizeof(line1), "Target:%3dcm", (int)lroundf(targetCm));
  lcdWrite(1, 0, line1);
  lcdWrite(2, 0, "Send 'f' to start");
  lcdWrite(3, 0, "Send 'c' cancel");
}

void FollowDistanceUI::showRunning(float targetCm, float currentCm, float errorCm,
                                   float headingHoldDeg, float headingNowDeg,
                                   FollowStatus status) {
  char line0[21];
  snprintf(line0, sizeof(line0), "Follow %3dcm", (int)lroundf(targetCm));

  char line1[21];
  snprintf(line1, sizeof(line1), "Now:%4dcm", (int)lroundf(currentCm));

  char line2[21];
  snprintf(line2, sizeof(line2), "H:%3d C:%3d",
           (int)lroundf(headingHoldDeg),
           (int)lroundf(headingNowDeg));

  const char* state = "HOLD";
  if (status == FollowStatus::Forward)  state = "FWD";
  if (status == FollowStatus::Backward) state = "BWD";
  if (status == FollowStatus::Invalid)  state = "INV";

  char line3[21];
  snprintf(line3, sizeof(line3), "State:%s", state);

  lcdWrite(0, 0, line0);
  lcdWrite(1, 0, line1);
  lcdWrite(2, 0, line2);
  lcdWrite(3, 0, line3);
}
