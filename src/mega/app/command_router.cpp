#include "command_router.h"

#include "esp/esp_uart.h"
#include "missions/maze/maze_lcd.h"
#include "motor/motor.h"
#include "odometry/odometry_manager.h"
#include "odometry/world_odometry.h"
#include "color/color_classifier.h"

#include "motion_control.h"
#include "protocol_helpers.h"
#include "reflex.h"

// SECTION: Optional local secrets override (gitignored).
#if defined(__has_include)
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#endif

#ifndef ESP_PASSCODE_INT
#define ESP_PASSCODE_INT 1234
#endif

static const int kEspPasscodeInt = ESP_PASSCODE_INT;
static const uint8_t kEspMaxFails = 3;

// SECTION: Telemetry serialization helpers.
static void sendRgbRefsToEsp_(RuntimeState& rt) {
  if (!rt.colorCalTask.hasCalibration()) return;
  const ColorRgb* refs = rt.colorCalTask.refs();
  if (!refs) return;
  Serial2.print("RGBREF:");
  for (uint8_t i = 0; i < ColorCalibrationTask::kColorCount; i++) {
    if (i) Serial2.print(";");
    Serial2.print((int)refs[i].r);
    Serial2.print(",");
    Serial2.print((int)refs[i].g);
    Serial2.print(",");
    Serial2.print((int)refs[i].b);
  }
  Serial2.println();
}

static int8_t classifyColorIdx1BasedOrNone_(RuntimeState& rt, const ColorRgb& live) {
  if (!rt.colorCalTask.hasCalibration()) return 0;
  const ColorRgb* refs = rt.colorCalTask.refs();
  if (!refs) return 0;
  color::ClassifyConfig cfg;
  cfg.absMaxDist = 0.18f;
  cfg.bestOverSecondMax = 0.80f;
  const color::ClassifyResult r = color::classifyNearest(live, refs, ColorCalibrationTask::kColorCount, cfg);
  if (r.idx < 0) return 0;
  return (int8_t)(r.idx + 1);
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

void sendTelemetrySnapshotToEsp(RuntimeState& rt) {
  sendRgbRefsToEsp_(rt);

  const int8_t cls = rt.lastRgbValid ? classifyColorIdx1BasedOrNone_(rt, rt.lastRgb) : (int8_t)0;
  rt.lastRgbClassSent = cls;
  Serial2.print("RGBCLS:");
  Serial2.println((int)cls);

  const float mmPerPulse = rt.encCalTask.calibratedCmPerPulse() * 10.0f;
  if (isfinite(mmPerPulse) && mmPerPulse > 0.0f) {
    Serial2.print("ENC_CAL:");
    Serial2.println(mmPerPulse, 2);
  }

  if (rt.turretEncCal.hasCalibration()) {
    Serial2.print("TURCAL:");
    Serial2.print((unsigned long)rt.turretEncCal.ticksPerRevPos());
    Serial2.print(",");
    Serial2.println((unsigned long)rt.turretEncCal.ticksPerRevNeg());
  }
}

// SECTION: Command safety helpers.
static void stopAndCancelMotionOwners_(RuntimeState& rt) {
  // CONTRACT: Any auth/lockout stop path must stop motors immediately.
  cancelActiveMotion(rt);
  motorDrive(0.0f, 0.0f);
}

static void startMotionSequence_(RuntimeState& rt, const CompassData& heading, const OdometryData& od) {
  // CONTRACT: New motion command resets reflex latches for a fresh safety window.
  rt.seqExecTask.clearAlignHeading();
  rt.seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
  rt.seqWasActive = true;
  resetReflexLatchesForNewCommand(rt);
}

bool handleEspCommand(RuntimeState& rt, const CompassData& heading, float) {
  // SECTION: ESP command dispatch.
  EspCommand espCmd;
  if (!espPoll(espCmd)) return false;

  if (espCmd.type == EspCommand::Type::EspIp) {
    if (espCmd.text.length() > 0) {
      espCmd.text.toCharArray(rt.espIpStr, sizeof(rt.espIpStr));
      rt.espIpStr[sizeof(rt.espIpStr) - 1] = '\0';
      maze_lcd::setIp(rt.espIpStr);
    }
    return true;
  }
  if (espCmd.type == EspCommand::Type::Passcode) {
    if (rt.espLocked) {
      Serial2.println("AUTH:LOCKED");
    } else if (espCmd.value == kEspPasscodeInt) {
      const bool wasArmed = rt.espArmed;
      rt.espArmed = true;
      rt.espFailCount = 0;
      Serial2.println("AUTH:OK");
      if (!wasArmed) sendTelemetrySnapshotToEsp(rt);
    } else {
      rt.espArmed = false;
      if (espCmd.value >= 0) {
        if (rt.espFailCount < 255) rt.espFailCount++;
      }
      const uint8_t triesLeft = (rt.espFailCount >= kEspMaxFails) ? 0 : (uint8_t)(kEspMaxFails - rt.espFailCount);
      if (rt.espFailCount >= kEspMaxFails) {
        rt.espLocked = true;
        Serial2.println("AUTH:LOCKED");
      } else {
        Serial2.print("AUTH:FAIL:");
        Serial2.println(triesLeft);
      }
    }
    motorDrive(0.0f, 0.0f);
    return true;
  }

  if (espCmd.type == EspCommand::Type::Disarm) {
    if (rt.espLocked) {
      Serial2.println("AUTH:LOCKED");
    } else {
      rt.espArmed = false;
      Serial2.println("AUTH:OFF");
    }
    stopAndCancelMotionOwners_(rt);
    return true;
  }

  if (rt.espLocked) {
    Serial2.println("AUTH:LOCKED");
    stopAndCancelMotionOwners_(rt);
    return true;
  }

  if (!rt.espArmed) {
    Serial2.println("AUTH:REQUIRED");
    stopAndCancelMotionOwners_(rt);
    return true;
  }

  if (espCmd.type == EspCommand::Type::MapPose) {
    float east = 0.0f, north = 0.0f, hdgMatch = 0.0f;
    bool hasH = false;
    if (parseCommaFloats3Opt(espCmd.text, east, north, hdgMatch, hasH)) {
      odomWorldSetPos(east, north, heading.headingDegContinuous);
      if (hasH) {
        const float matchWrapped = wrapDeg360(hdgMatch);
        const float compassWrapped = wrapDeg360(heading.headingDegWrapped);
        const float err = wrapDegDiff180(matchWrapped, compassWrapped);

        const float kAlpha = 0.25f;
        float off = rt.compass.headingOffsetDeg();
        off += kAlpha * err;
        off = clampf_(off, -180.0f, 180.0f);
        rt.compass.setHeadingOffsetDeg(off);
        rt.compass.resetHeadingContinuous();
        odomWorldRebase(matchWrapped);
      }
    }
    return true;
  }

  if (espCmd.type == EspCommand::Type::TurretZero) {
    rt.turretEncCal.setZeroTicks(rt.turretMotor.ticksAbs());
    rt.turretAngle.setZero();
    Serial2.println("TURZERO:OK");
    return true;
  } else if (espCmd.type == EspCommand::Type::TurretTpr) {
    const uint32_t pos = (uint32_t)espCmd.value;
    const uint32_t neg = (uint32_t)espCmd.value2;
    if (pos < 10u || pos > 200000u || neg < 10u || neg > 200000u) {
      Serial2.println("TURCAL:FAIL");
      if (rt.turretEncCal.hasCalibration()) {
        Serial2.print("TURCAL:");
        Serial2.print((unsigned long)rt.turretEncCal.ticksPerRevPos());
        Serial2.print(",");
        Serial2.println((unsigned long)rt.turretEncCal.ticksPerRevNeg());
      }
      return true;
    }

    const bool okPos = rt.turretEncCal.setTicksPerRevPos(pos);
    const bool okNeg = rt.turretEncCal.setTicksPerRevNeg(neg);
    if (okPos && okNeg && rt.turretEncCal.hasCalibration()) {
      rt.turretAngle.setTicksPerRevPosNeg(rt.turretEncCal.ticksPerRevPos(), rt.turretEncCal.ticksPerRevNeg());
      Serial2.print("TURCAL:");
      Serial2.print((unsigned long)rt.turretEncCal.ticksPerRevPos());
      Serial2.print(",");
      Serial2.println((unsigned long)rt.turretEncCal.ticksPerRevNeg());
    } else {
      Serial2.println("TURCAL:FAIL");
      if (rt.turretEncCal.hasCalibration()) {
        Serial2.print("TURCAL:");
        Serial2.print((unsigned long)rt.turretEncCal.ticksPerRevPos());
        Serial2.print(",");
        Serial2.println((unsigned long)rt.turretEncCal.ticksPerRevNeg());
      }
    }
    return true;
  }

  const bool isMotionCmd = (espCmd.type == EspCommand::Type::Move || espCmd.type == EspCommand::Type::Turn ||
                            espCmd.type == EspCommand::Type::TurnShortest || espCmd.type == EspCommand::Type::TurnAbs ||
                            espCmd.type == EspCommand::Type::EncCal);
  // CONTRACT: Movement controllers are single-owner; reject new motion while busy.
  const bool motorBusy = (rt.seqExecTask.active() || rt.encCalTask.active() || rt.turretSweep.active());
  if (isMotionCmd && motorBusy) {
    sendEvtPose_(rt, "CMDFAIL", NAN);
    return true;
  }

  cancelActiveMotion(rt);

  if (espCmd.type == EspCommand::Type::TurretScanCancel) {
    if (rt.turretSweep.active()) {
      rt.turretSweep.cancel();
      Serial2.println("TSCAN:CANCEL");
    }
    motorDrive(0.0f, 0.0f);
    return true;
  }
  if (espCmd.type == EspCommand::Type::TurretScanPlus || espCmd.type == EspCommand::Type::TurretScanMinus) {
    if (rt.turretSweep.active()) return true;
    motorDrive(0.0f, 0.0f);
    const int dir = (espCmd.type == EspCommand::Type::TurretScanMinus) ? -1 : +1;
    Serial2.print("TSCAN:BEGIN,");
    Serial2.println(dir < 0 ? "-" : "+");
    rt.turretSweep.begin(&rt.turretMotor, &rt.turretAngle, &rt.turretCompass, &rt.lidar, dir, rt.turretMotor.ticksAbs(), millis());
    return true;
  }

  odomHardResetKeepWorld(heading.headingDegContinuous);
  // CONTRACT: New motion starts from a fresh local baseline while preserving world continuity.
  const OdometryData od = odomRaw();

  if (espCmd.type == EspCommand::Type::Move) {
    rt.espSteps[0] = {SequenceStepType::MoveByDistance, (float)espCmd.value};
    rt.espSteps[1] = {SequenceStepType::End, 0.0f};
    rt.seqExecTask.setSequence(rt.espSteps);
    startMotionSequence_(rt, heading, od);
  } else if (espCmd.type == EspCommand::Type::Turn) {
    rt.espSteps[0] = {SequenceStepType::TurnDeg, (float)espCmd.value};
    rt.espSteps[1] = {SequenceStepType::End, 0.0f};
    rt.seqExecTask.setSequence(rt.espSteps);
    startMotionSequence_(rt, heading, od);
  } else if (espCmd.type == EspCommand::Type::TurnShortest) {
    rt.espSteps[0] = {SequenceStepType::TurnDegShortest, (float)espCmd.value};
    rt.espSteps[1] = {SequenceStepType::End, 0.0f};
    rt.seqExecTask.setSequence(rt.espSteps);
    startMotionSequence_(rt, heading, od);
  } else if (espCmd.type == EspCommand::Type::TurnAbs) {
    const float targetWrapped = wrapDeg360((float)espCmd.value);
    const float curWrapped = wrapDeg360(heading.headingDegWrapped);
    const float delta = wrapDegDiff180(targetWrapped, curWrapped);
    rt.espSteps[0] = {SequenceStepType::TurnDegShortest, delta};
    rt.espSteps[1] = {SequenceStepType::End, 0.0f};
    rt.seqExecTask.setSequence(rt.espSteps);
    startMotionSequence_(rt, heading, od);
  } else if (espCmd.type == EspCommand::Type::EncCal) {
    rt.encCalTask.begin(heading.headingDegContinuous, od.avgCmSigned);
  }

  return false;
}

