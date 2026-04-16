#include <Arduino.h>

#include "compass/compass.h"
#include "lidar/lidar.h"
#include "lidar/utils/moving_average.h"
#include "lidar/turret/turret_motor.h"
#include "lidar/turret/turret_encoder_cal.h"
#include "motor/motor.h"
#include "display/lcd.h"
#include "encoder/encoder.h"
#include "odometry/odometry.h"
#include "odometry/odometry_manager.h"
#include "actions/drive_by_distance.h"
#include "tasks/wall_align_90/wall_align_90_task.h"
#include "tasks/wall_sequence/wall_sequence_task.h"
#include "tasks/encoder_calibration/encoder_calibration_task.h"
#include "tasks/sequence_executor/sequence_executor_task.h"
#include "esp/esp_uart.h"
#include "telemetry/telemetry.h"
#include "telemetry/telemetry_test.h"
#include "color/color_sensor.h"
#include "tasks/color_calibration/color_calibration_task.h"

#include "tasks/room_measure/room_measure_task.h"
#include "tasks/follow_distance/follow_distance_task.h"
#include "tasks/color_maze/color_maze_task.h"

static Compass         compass;
static Lidar           lidar;
static MovingAverage   lidarAvg;
static TurretMotor     turretMotor;
static TurretEncoderCal turretEncCal;
static RoomMeasureTask roomTask;
static FollowDistanceTask followTask;
static uint32_t        lastCompassUiMs = 0;
static Odometry        odom(0.0f);
static DriveByDistance driveByDistance;
static WallAlign90Task wallAlignTask;
static WallSequenceTask wallSeqTask;
static EncoderCalibrationTask encCalTask;
static SequenceExecutorTask seqExecTask;
static ColorSensor     colorSensor;
static bool            colorSensorOk = false;
static uint32_t        lastRgbMs = 0;
static ColorRgb        lastRgb = {0, 0, 0};
static bool            lastRgbValid = false;
static ColorMazeTask   colorMazeTask;
static uint32_t        lastEncDbgMs = 0;
static bool            headingValid = false;
static CompassData     lastHeading = {};
static bool            seqHeadingSet = false;
static float           seqHeadingHoldDeg = 0.0f;

// ===== Option 1 security (UART passcode arming) =====
static const char kEspPasscode[] = "1234";
static bool gEspArmed = false;
static uint8_t gEspFailCount = 0;
static bool gEspLocked = false;
static const uint8_t kEspMaxFails = 3;
static SequenceStep espSteps[] = {
  {SequenceStepType::End, 0.0f},
  {SequenceStepType::End, 0.0f}
};

static ColorCalibrationTask colorCalTask;

static const int kButtonPin = 19;
static const uint32_t kButtonDebounceMs = 250;
static volatile bool gButtonEdge = false;
static uint32_t gLastButtonMs = 0;

static void onButtonIsr() {
  gButtonEdge = true;
}

static float wrapDegDiff180(float targetDeg, float currentDeg) {
  float d = targetDeg - currentDeg;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static SequenceStep seqSteps[] = {
  {SequenceStepType::MoveToDistance, 34.5f},
  {SequenceStepType::TurnDeg, 90.0f},
  {SequenceStepType::MoveToDistance, 27.2f},
  {SequenceStepType::End, 0.0f}
};

void setup() {
  Serial.begin(115200);
  lcdInit();

  // --- Hardware ---
  if (!compass.begin()) {
    Serial.println("Compass failed");
    while (1) {}
  }

  if (!lidar.begin()) {
    Serial.println("LiDAR failed");
    while (1) {}
  }

  turretMotor.begin();
  turretEncCal.loadFromEeprom();
  colorSensorOk = colorSensor.begin();
  if (!colorSensorOk) {
    Serial.println("Color sensor failed (continuing without RGB telemetry)");
  }

  motorInit();
  pinMode(kButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kButtonPin), onButtonIsr, FALLING);

  colorCalTask.loadFromEeprom();

  encoderInit();
  espSetup();
  telemetryInit(200);
  Serial2.println("AUTH:OFF");
  // Send last saved encoder calibration (if any) so the web UI can show it.
  const float mmPerPulse = encCalTask.calibratedCmPerPulse() * 10.0f;
  if (isfinite(mmPerPulse) && mmPerPulse > 0.0f) {
    Serial2.print("ENC_CAL:");
    Serial2.println(mmPerPulse, 2);
  }
  if (turretEncCal.hasCalibration()) {
    Serial2.print("TURCAL:");
    Serial2.println((unsigned long)turretEncCal.ticksPerRev());
  }

  // --- Odometry (set your pulses/meter) ---
  odom.setPulsesPerMeter(787.0f);
  odomManagerInit(&odom);

  // --- DriveByDistance test config ---
  DriveByDistance::Config dcfg = driveByDistance.config();
  dcfg.maxSpeed = 0.6f;
  dcfg.minSpeed = 0.2f;
  dcfg.timeoutMs = 8000;
  driveByDistance.setConfig(dcfg);

  // --- Room measurement physical config ONLY ---
  RoomMeasureTask::Config cfg;
  cfg.lidarToCenterOffsetCm = 18.0f;   // measured once
  cfg.ceilingHeightCm       = 100.0f;  // task requirement

  roomTask.setConfig(cfg);
  roomTask.reset();

  // --- Follow distance (target only) ---
  followTask.setTargetDistanceCm(30.0f);
  followTask.reset();

  wallAlignTask.reset();
  wallSeqTask.reset();
  encCalTask.reset();
  seqExecTask.setSequence(seqSteps);
  seqExecTask.reset();

  Serial.println("Ready. Send 'm' room, 'f' follow, 'd' drive, 'w' wall, 's' seq, 'k' cal, 'h' set heading, 'q' exec.");
}

void loop() {
  // ---- Compass (continuous heading) ----
  CompassData heading;
  if (!compass.read(heading)) {
    if (!headingValid) return;
    heading = lastHeading;
  } else {
    lastHeading = heading;
    headingValid = true;
  }

  // ---- LiDAR (moving averaged) ----
  float lidarCm = 0.0f;
  if (lidar.update(lidarCm)) {
    lidarAvg.push(lidarCm);
  }

  const float lidarFilteredCm = lidarAvg.average();
  telemetryUpdate(lidarFilteredCm, (int)lroundf(heading.headingDegContinuous), heading.headingDirLabel);
  telemetryTestUpdate(lidarFilteredCm);

  // ---- Odometry manager (local + world-frame) ----
  // Integrates world-frame East/North continuously.
  odomManagerUpdate(heading.headingDegContinuous);

  const uint32_t nowMs = millis();
  if (colorSensorOk && nowMs - lastRgbMs >= 200U) {
    lastRgbMs = nowMs;
    ColorRgb rgb;
    if (colorSensor.read(rgb)) {
      lastRgb = rgb;
      lastRgbValid = true;
      telemetryRgbUpdate(rgb.r, rgb.g, rgb.b);
    } else {
      lastRgbValid = false;
    }
  }

  if (gButtonEdge) {
    if (nowMs - gLastButtonMs >= kButtonDebounceMs) {
      gButtonEdge = false;
      if (digitalRead(kButtonPin) == LOW) {
        gLastButtonMs = nowMs;
        if (!colorCalTask.active()) {
          if (roomTask.active()) roomTask.cancel();
          if (followTask.active()) followTask.cancel();
          if (driveByDistance.active()) driveByDistance.cancel();
          if (wallAlignTask.active()) wallAlignTask.cancel();
          if (wallSeqTask.active()) wallSeqTask.cancel();
          if (encCalTask.active()) encCalTask.cancel();
          if (seqExecTask.active()) seqExecTask.cancel();
          motorDrive(0.0f, 0.0f);
          colorCalTask.begin();
        } else {
          colorCalTask.onButtonPress(&lastRgb, lastRgbValid);
        }
      }
    }
  }

  EspCommand espCmd;
  if (espPoll(espCmd)) {
    if (espCmd.type == EspCommand::Type::Passcode) {
      if (gEspLocked) {
        Serial2.println("AUTH:LOCKED");
      } else if (espCmd.text.equals(kEspPasscode)) {
        gEspArmed = true;
        gEspFailCount = 0;
        Serial2.println("AUTH:OK");
      } else {
        gEspArmed = false;
        if (gEspFailCount < 255) gEspFailCount++;
        const uint8_t triesLeft =
            (gEspFailCount >= kEspMaxFails) ? 0 : (uint8_t)(kEspMaxFails - gEspFailCount);
        if (gEspFailCount >= kEspMaxFails) {
          gEspLocked = true;
          Serial2.println("AUTH:LOCKED");
        } else {
          Serial2.print("AUTH:FAIL:");
          Serial2.println(triesLeft);
        }
      }
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (espCmd.type == EspCommand::Type::Disarm) {
      // IMPORTANT: once locked, do not allow disarm/unlock until reset.
      if (gEspLocked) {
        Serial2.println("AUTH:LOCKED");
      } else {
        gEspArmed = false;
        Serial2.println("AUTH:OFF");
      }
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (gEspLocked) {
      Serial2.println("AUTH:LOCKED");
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (!gEspArmed) {
      Serial2.println("AUTH:REQUIRED");
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (espCmd.type == EspCommand::Type::TurretCalStart) {
      turretEncCal.start(turretMotor.ticksAbs());
      Serial.println("Turret cal start");
      Serial2.println("TURCAL:CALIBRATING");
      return;
    } else if (espCmd.type == EspCommand::Type::TurretCalDone) {
      const bool ok = turretEncCal.finish(turretMotor.ticksAbs());
      Serial.println(ok ? "Turret cal done" : "Turret cal failed");
      if (ok) {
        Serial2.print("TURCAL:");
        Serial2.println((unsigned long)turretEncCal.ticksPerRev());
      } else {
        Serial2.println("TURCAL:FAIL");
      }
      return;
    } else if (espCmd.type == EspCommand::Type::TurretZero) {
      turretEncCal.setZeroTicks(turretMotor.ticksAbs());
      Serial.println("Turret zero set");
      Serial2.println("TURCAL:ZEROED");
      return;
    }

    // Commands below this point are motion/task-affecting.
    if (roomTask.active()) roomTask.cancel();
    if (followTask.active()) followTask.cancel();
    if (driveByDistance.active()) driveByDistance.cancel();
    if (wallAlignTask.active()) wallAlignTask.cancel();
    if (wallSeqTask.active()) wallSeqTask.cancel();
    if (encCalTask.active()) encCalTask.cancel();
    if (seqExecTask.active()) seqExecTask.cancel();
    if (colorMazeTask.active()) colorMazeTask.cancel();

    odomHardResetKeepWorld(heading.headingDegContinuous);
    OdometryData od = odomRaw();

    if (espCmd.type == EspCommand::Type::Move) {
      espSteps[0] = {SequenceStepType::MoveByDistance, (float)espCmd.value};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
    } else if (espCmd.type == EspCommand::Type::Turn) {
      espSteps[0] = {SequenceStepType::TurnDeg, (float)espCmd.value};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
    } else if (espCmd.type == EspCommand::Type::North) {
      if (!seqHeadingSet) {
        Serial.println("Seq heading not set. Press 'h' first.");
      } else {
        const float delta = wrapDegDiff180(seqHeadingHoldDeg, heading.headingDegContinuous);
        espSteps[0] = {SequenceStepType::TurnDeg, delta};
        espSteps[1] = {SequenceStepType::End, 0.0f};
        seqExecTask.setSequence(espSteps);
        seqExecTask.clearAlignHeading();
        seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      }
    } else if (espCmd.type == EspCommand::Type::SetNorth) {
      seqHeadingHoldDeg = heading.headingDegContinuous;
      seqHeadingSet = true;
      Serial.print("Seq heading set (ESP): ");
      Serial.println(seqHeadingHoldDeg, 2);
    } else if (espCmd.type == EspCommand::Type::Maze) {
      if (colorCalTask.hasCalibration()) {
        ColorMazeTask::Config cfg;
        cfg.driveSpeed = 0.30f;
        cfg.backoffCm = 5.0f;
        cfg.turnDeg = 30.0f;
        cfg.cooldownMs = 400;
        cfg.leftThreshold = 0.18f;
        cfg.rightThreshold = 0.18f;
        cfg.endThreshold = 0.14f;
        cfg.useNormalized = true;
        colorMazeTask.setConfig(cfg);
        colorMazeTask.setCalibration(colorCalTask.refs(), true);
        colorMazeTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      } else {
        colorMazeTask.setCalibration(nullptr, false);
        colorMazeTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      }
    } else if (espCmd.type == EspCommand::Type::EncCal) {
      encCalTask.begin(heading.headingDegContinuous, od.avgCmSigned);
    }
  }

  // Allow ESP Disarm/Passcode handling above to work even while these are active.
  if (colorCalTask.active()) {
    colorCalTask.update(&lastRgb, lastRgbValid);
    return;
  }

  if (colorMazeTask.active()) {
    OdometryData od = odomRaw();
    colorMazeTask.update(heading.headingDegContinuous, od.avgCmSigned, &lastRgb, lastRgbValid);
    return;
  }

  if (Serial.available()) {
    const char c = Serial.read();
    if ((c == 'm' || c == 'M') && !roomTask.active()) {
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      roomTask.begin(heading.headingDegContinuous);
    }
    if (c == 'f' || c == 'F') {
      if (roomTask.active()) roomTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      if (!followTask.active()) followTask.begin(heading.headingDegContinuous, 0.0f);
    }
    if (c == 'd' || c == 'D') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      odomHardResetKeepWorld(heading.headingDegContinuous);
      OdometryData od = odomRaw();
      driveByDistance.beginByDistance(heading.headingDegContinuous, od.avgCmSigned, 20.0f, 0.5f);
    }
    if (c == 'w' || c == 'W') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      odomHardResetKeepWorld(heading.headingDegContinuous);
      OdometryData od = odomRaw();
      wallAlignTask.begin(heading.headingDegContinuous, od.avgCmSigned);
    }
    if (c == 's' || c == 'S') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      odomHardResetKeepWorld(heading.headingDegContinuous);
      OdometryData od = odomRaw();
      wallSeqTask.begin(heading.headingDegContinuous, od.avgCmSigned);
    }
    if (c == 'k' || c == 'K') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      odomHardResetKeepWorld(heading.headingDegContinuous);
      OdometryData od = odomRaw();
      encCalTask.begin(heading.headingDegContinuous, od.avgCmSigned);
    }
    if (c == 'h' || c == 'H') {
      seqHeadingHoldDeg = heading.headingDegContinuous;
      seqHeadingSet = true;
      Serial.print("Seq heading set: ");
      Serial.println(seqHeadingHoldDeg, 2);
    }
    if (c == 'q' || c == 'Q') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      odomHardResetKeepWorld(heading.headingDegContinuous);
      OdometryData od = odomRaw();
      if (!seqHeadingSet) {
        Serial.println("Seq heading not set. Press 'h' first.");
      } else {
        seqExecTask.setAlignHeading(seqHeadingHoldDeg);
        seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      }
    }
    if (c == 'c' || c == 'C') {
      if (followTask.active()) followTask.cancel();
      if (driveByDistance.active()) driveByDistance.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
    }
    if (c == 't' || c == 'T') {
      telemetryTestStart();
    }
  }

  // ---- Run tasks/actions ----
  if (seqExecTask.active()) {
    OdometryData od = odomRaw();
    seqExecTask.update(heading.headingDegContinuous, od.avgCmSigned, od.avgCmAbs, lidarFilteredCm);
  } else if (encCalTask.active()) {
    OdometryData od = odomRaw();
    const EncoderCalibrationTask::State prev = encCalTask.state();
    encCalTask.update(heading.headingDegContinuous, od.avgCmSigned, lidarFilteredCm);
    if (prev != EncoderCalibrationTask::State::Succeeded &&
        encCalTask.state() == EncoderCalibrationTask::State::Succeeded) {
      const float mmPerPulse = encCalTask.calibratedCmPerPulse() * 10.0f;
      if (isfinite(mmPerPulse) && mmPerPulse > 0.0f) {
        Serial2.print("ENC_CAL:");
        Serial2.println(mmPerPulse, 2);
      }
    }
  } else if (wallSeqTask.active()) {
    OdometryData od = odomRaw();
    wallSeqTask.update(heading.headingDegContinuous, od.avgCmSigned, od.avgCmAbs, lidarFilteredCm);
  } else if (wallAlignTask.active()) {
    OdometryData od = odomRaw();
    wallAlignTask.update(heading.headingDegContinuous, od.avgCmSigned, lidarFilteredCm);
  } else if (driveByDistance.active()) {
    OdometryData od = odomRaw();
    driveByDistance.update(heading.headingDegContinuous, od.avgCmSigned);
  } else if (followTask.active()) {
    followTask.update(heading.headingDegContinuous, 0.0f, lidarFilteredCm);
  } else {
    roomTask.update(heading.headingDegContinuous, lidarFilteredCm);
  }
}
