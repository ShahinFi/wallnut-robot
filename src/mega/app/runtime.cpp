#include "runtime.h"

RuntimeState gRt;

void runtimeInit(RuntimeState& rt) {
  // SECTION: Sensor and telemetry cache defaults.
  rt.colorSensorOk = false;
  rt.lastRgbMs = 0;
  rt.lastRgb = {0, 0, 0};
  rt.lastRgbValid = false;

  // SECTION: Link and heading baselines.
  strncpy(rt.espIpStr, "0.0.0.0", sizeof(rt.espIpStr));
  rt.espIpStr[sizeof(rt.espIpStr) - 1] = '\0';
  rt.lastIpReqMs = 0;

  rt.headingValid = false;
  rt.lastHeading = {};

  // SECTION: Auth defaults.
  rt.espArmed = false;
  rt.espFailCount = 0;
  rt.espLocked = false;

  // SECTION: Command template defaults.
  rt.espSteps[0] = {SequenceStepType::End, 0.0f};
  rt.espSteps[1] = {SequenceStepType::End, 0.0f};

  rt.seqSteps[0] = {SequenceStepType::MoveToDistance, 34.5f};
  rt.seqSteps[1] = {SequenceStepType::TurnDeg, 90.0f};
  rt.seqSteps[2] = {SequenceStepType::MoveToDistance, 27.2f};
  rt.seqSteps[3] = {SequenceStepType::End, 0.0f};

  // SECTION: Button and reflex latches.
  rt.buttonEdge = false;
  rt.lastButtonMs = 0;

  rt.reflexLatchedRed = false;
  rt.reflexLatchedFront = false;
  rt.seqWasActive = false;

  // SECTION: Motion behavior defaults.
  rt.forwardSpeedScale = 0.50f;
  rt.lastRgbClassSent = -1;
}

