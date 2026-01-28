#include "tasks/room_measure/ui/room_measure_ui.h"

#include <math.h>
#include <stdio.h>
#include "display/lcd.h"

void RoomMeasureUI::begin() {
  lcdInit();
  showIdle();
}

void RoomMeasureUI::showIdle() {
  lcdClear();
  lcdWrite(0, 0, "Room Measure");
  lcdWrite(1, 0, "Send 'm' to start");
  lcdWrite(2, 0, "Send 'c' cancel");
}

void RoomMeasureUI::showRunning(float sweepDeg, float lidarCm) {
  char line1[21];
  snprintf(line1, sizeof(line1), "Sweeping:%3ddeg", (int)lroundf(sweepDeg));
  char line2[21];
  snprintf(line2, sizeof(line2), "Dist:%4dcm", (int)lroundf(lidarCm));
  lcdWrite(0, 0, "Scanning 360...");
  lcdWrite(1, 0, line1);
  lcdWrite(2, 0, line2);
}

void RoomMeasureUI::showSucceeded(const float wallCm[4], float widthCm, float lengthCm,
                                  float areaM2, float volumeM3) {
  lcdClear();

  // Line 0: labels
  lcdWrite(0, 0, "W0 W90 W180 W270");

  // Line 1: 4 distances (ints)
  char line1[21];
  snprintf(line1, sizeof(line1), "%3d %3d %3d %3dcm",
           (int)lroundf(wallCm[0]),
           (int)lroundf(wallCm[1]),
           (int)lroundf(wallCm[2]),
           (int)lroundf(wallCm[3]));
  lcdWrite(1, 0, line1);

  // Line 2: Width / Length (cm)
  char line2[21];
  snprintf(line2, sizeof(line2), "W:%3dcm L:%3dcm",
           (int)lroundf(widthCm),
           (int)lroundf(lengthCm));
  lcdWrite(2, 0, line2);

  // Line 3: Area and Volume
  lcdWrite(3, 0, "A:");
  lcdWriteFloat(3, 2, areaM2, 2, false);
  lcdWrite(3, 2 + 6, "m2 ");
  lcdWrite(3, 2 + 9, "V:");
  lcdWriteFloat(3, 2 + 11, volumeM3, 2, false);
  lcdWrite(3, 2 + 17, "m3");
}

void RoomMeasureUI::showFailed(const char* msg) {
  lcdClear();
  lcdWrite(0, 0, "Scan failed");
  lcdWrite(1, 0, msg ? msg : "");
}
