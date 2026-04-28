#include "motion_control.h"

#include "motor/motor.h"
#include "odometry/odometry_manager.h"

#include "command_router.h"

// SECTION: Motion completion event helpers.
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

void cancelActiveMotion(RuntimeState& rt) {
  // CONTRACT: Shared cancel path must stop active motion owners only.
  if (rt.encCalTask.active()) rt.encCalTask.cancel();
  if (rt.seqExecTask.active()) rt.seqExecTask.cancel();
}

void tickActiveControllers(RuntimeState& rt, const CompassData& heading, float lidarFilteredCm) {
  // SECTION: Active controller updates.
  if (rt.seqExecTask.active()) {
    OdometryData od = odomRaw();
    rt.seqExecTask.update(heading.headingDegContinuous, od.avgCmSigned, od.avgCmAbs, lidarFilteredCm);
  } else if (rt.encCalTask.active()) {
    OdometryData od = odomRaw();
    const EncoderCalibrationTask::State prev = rt.encCalTask.state();
    rt.encCalTask.update(heading.headingDegContinuous, od.avgCmSigned, lidarFilteredCm);
    if (prev != EncoderCalibrationTask::State::Succeeded && rt.encCalTask.state() == EncoderCalibrationTask::State::Succeeded) {
      const float mmPerPulse = rt.encCalTask.calibratedCmPerPulse() * 10.0f;
      if (isfinite(mmPerPulse) && mmPerPulse > 0.0f) {
        Serial2.print("ENC_CAL:");
        Serial2.println(mmPerPulse, 2);
      }
    }
  }
}

void emitCommandCompletionEdge(RuntimeState& rt) {
  // CONTRACT: Emit command completion exactly on Running->terminal edge.
  const bool seqActiveNow = rt.seqExecTask.active();
  if (rt.seqWasActive && !seqActiveNow) {
    const SequenceExecutorTask::State st = rt.seqExecTask.state();
    if (st == SequenceExecutorTask::State::Succeeded) {
      sendEvtPose_(rt, "CMDOK", NAN);
    } else if (st == SequenceExecutorTask::State::Failed) {
      sendEvtPose_(rt, "CMDFAIL", NAN);
    } else if (st == SequenceExecutorTask::State::Cancelled) {
      sendEvtPose_(rt, "CMDCANCEL", NAN);
    }
  }
  rt.seqWasActive = seqActiveNow;
}

