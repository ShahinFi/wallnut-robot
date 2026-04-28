#include "reflex.h"

#include "motor/motor.h"
#include "missions/maze/maze_lcd.h"
#include "odometry/world_odometry.h"
#include "odometry/odometry_manager.h"
#include "color/color_classifier.h"

static bool isVirtualRed_(RuntimeState& rt, const ColorRgb& live) {
  if (!rt.colorCalTask.hasCalibration()) return false;
  const ColorRgb* refs = rt.colorCalTask.refs();
  if (!refs) return false;

  color::ClassifyConfig cfg;
  cfg.absMaxDist = 0.18f;
  cfg.bestOverSecondMax = 0.80f;
  const color::ClassifyResult r = color::classifyNearest(live, refs, ColorCalibrationTask::kColorCount, cfg);
  return r.idx == 0;
}

static void sendEvtPose_(RuntimeState& rt, const char* tag, float extra) {
  const WorldOdomData w = odomWorldRead();
  Serial2.print("EVT:");
  Serial2.print(tag);
  Serial2.print(",");
  Serial2.print(w.eastCm, 2);
  Serial2.print(",");
  Serial2.print(w.northCm, 2);
  Serial2.print(",");
  Serial2.print(rt.lastHeading.headingDegWrapped, 1);
  if (isfinite(extra)) {
    Serial2.print(",");
    Serial2.print(extra, 1);
  }
  Serial2.println();
}

void resetReflexLatchesForNewCommand(RuntimeState& rt) {
  rt.reflexLatchedRed = false;
  rt.reflexLatchedFront = false;
}

void applyReflexSafety(RuntimeState& rt, float lidarFilteredCm, const CompassData&) {
  if (!rt.seqExecTask.active()) return;

  const float kFrontStopCm = 10.0f;
  if (!rt.reflexLatchedFront && isfinite(lidarFilteredCm) && lidarFilteredCm > 0.0f && lidarFilteredCm < kFrontStopCm) {
    rt.reflexLatchedFront = true;
    rt.seqExecTask.cancel();
    motorDrive(0.0f, 0.0f);
    sendEvtPose_(rt, "FRONTSTOP", lidarFilteredCm);
    maze_lcd::notifyStopFront();
  }

  if (!rt.reflexLatchedRed && !rt.seqExecTask.reverseMoveActive() && rt.lastRgbValid && isVirtualRed_(rt, rt.lastRgb)) {
    rt.reflexLatchedRed = true;
    rt.seqExecTask.cancel();
    motorDrive(0.0f, 0.0f);
    sendEvtPose_(rt, "RED", NAN);
    maze_lcd::notifyStopRed();
  }
}

