#include "tasks/sequence_executor/ui/sequence_executor_ui.h"

#include <math.h>
#include <stdio.h>
#include "display/lcd.h"

void SequenceExecutorUI::begin() {
  lcdInit();
}

void SequenceExecutorUI::showIdle() {
  lcdClear();
  lcdWrite(0, 0, "Seq Exec");
  lcdWrite(1, 0, "Send 'q' start");
}

void SequenceExecutorUI::showRunning(float distanceCm, float totalCm, uint16_t stepIndex,
                                     uint16_t totalSteps, StepLabel label) {
  lcdWrite(0, 0, "Seq Exec");

  char line1[21];
  if (totalSteps == 0) {
    snprintf(line1, sizeof(line1), "Step:%2d/--", (int)stepIndex);
  } else {
    snprintf(line1, sizeof(line1), "Step:%2d/%2d", (int)stepIndex, (int)totalSteps);
  }
  lcdWrite(1, 0, line1);

  const char* l = "MOVE";
  if (label == StepLabel::Turn) l = "TURN";
  if (label == StepLabel::End)  l = "END";

  char line2[21];
  snprintf(line2, sizeof(line2), "Cmd:%s D:%3d",
           l, (int)lroundf(distanceCm));
  lcdWrite(2, 0, line2);

  char line3[21];
  snprintf(line3, sizeof(line3), "Total:%5dcm", (int)lroundf(totalCm));
  lcdWrite(3, 0, line3);
}

void SequenceExecutorUI::showFailed(const char* msg) {
  lcdClear();
  lcdWrite(0, 0, "Seq Exec Fail");
  lcdWrite(1, 0, msg ? msg : "");
}
