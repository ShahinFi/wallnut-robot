#include "tasks/color_maze/ui/color_maze_ui.h"

#include <stdio.h>
#include "display/lcd.h"

void ColorMazeUI::begin() {
  lcdInit();
}

void ColorMazeUI::showIdle() {
  lcdClear();
  lcdWrite(0, 0, "Color Maze");
  lcdWrite(1, 0, "Press web start");
}

void ColorMazeUI::showRunning(const char* label, int speedPct) {
  char line2[21];
  char line3[21];
  snprintf(line2, sizeof(line2), "Color:%s", label ? label : "UNK");
  snprintf(line3, sizeof(line3), "Speed:%3d%%", speedPct);

  lcdWrite(0, 0, "Maze RUN");
  lcdWrite(1, 0, line2);
  lcdWrite(2, 0, line3);
}

void ColorMazeUI::showBackoff(float cm) {
  char line2[21];
  snprintf(line2, sizeof(line2), "Back %.1fcm", cm);
  lcdWrite(0, 0, "Maze CORR");
  lcdWrite(1, 0, line2);
  lcdWrite(2, 0, "Backing up");
}

void ColorMazeUI::showTurn(float deg) {
  char line2[21];
  snprintf(line2, sizeof(line2), "Turn %.0fdeg", deg);
  lcdWrite(0, 0, "Maze CORR");
  lcdWrite(1, 0, line2);
  lcdWrite(2, 0, "Turning");
}

void ColorMazeUI::showDone() {
  lcdClear();
  lcdWrite(0, 0, "Maze DONE");
  lcdWrite(1, 0, "End color hit");
}

void ColorMazeUI::showFailed(const char* msg) {
  lcdClear();
  lcdWrite(0, 0, "Maze FAIL");
  lcdWrite(1, 0, msg ? msg : "");
}
