#include <Arduino.h>

#include "compass/compass.h"
#include "lidar/lidar.h"
#include "lidar/utils/moving_average.h"
#include "motor/motor.h"
#include "display/lcd.h"
#include "encoder/encoder.h"
#include "odometry/odometry.h"
#include "actions/drive_straight.h"
#include "tasks/wall_align_90/wall_align_90_task.h"
#include "tasks/wall_sequence/wall_sequence_task.h"
#include "tasks/encoder_calibration/encoder_calibration_task.h"
#include "tasks/sequence_executor/sequence_executor_task.h"
#include "esp/esp_uart.h"
#include "telemetry/telemetry.h"
#include "telemetry/telemetry_test.h"
#include "color/color_sensor.h"

#include "tasks/room_measure/room_measure_task.h"
#include "tasks/follow_distance/follow_distance_task.h"

static Compass         compass;
static Lidar           lidar;
static MovingAverage   lidarAvg;
static RoomMeasureTask roomTask;
static FollowDistanceTask followTask;
static uint32_t        lastCompassUiMs = 0;
static Odometry        odom(0.0f);
static DriveStraight   driveStraight;
static WallAlign90Task wallAlignTask;
static WallSequenceTask wallSeqTask;
static EncoderCalibrationTask encCalTask;
static SequenceExecutorTask seqExecTask;
static ColorSensor     colorSensor;
static uint32_t        lastRgbMs = 0;
static uint32_t        lastEncDbgMs = 0;
static bool            seqHeadingSet = false;
static float           seqHeadingHoldDeg = 0.0f;
static SequenceStep espSteps[] = {
  {SequenceStepType::End, 0.0f},
  {SequenceStepType::End, 0.0f}
};

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

  // --- Hardware ---
  if (!compass.begin()) {
    Serial.println("Compass failed");
    while (1) {}
  }

  if (!lidar.begin()) {
    Serial.println("LiDAR failed");
    while (1) {}
  }
  if (!colorSensor.begin()) {
    Serial.println("Color sensor failed");
    while (1) {}
  }

  motorInit();
  lcdInit();
  encoderInit();
  espSetup();
  telemetryInit(200);

  // --- Odometry (set your pulses/meter) ---
  odom.setPulsesPerMeter(787.0f);

  // --- DriveStraight test config ---
  DriveStraight::Config dcfg = driveStraight.config();
  dcfg.maxSpeed = 0.6f;
  dcfg.minSpeed = 0.2f;
  dcfg.timeoutMs = 8000;
  driveStraight.setConfig(dcfg);

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
  if (!compass.read(heading)) return;

  // ---- LiDAR (moving averaged) ----
  float lidarCm = 0.0f;
  if (lidar.update(lidarCm)) {
    lidarAvg.push(lidarCm);
  }

  const float lidarFilteredCm = lidarAvg.average();
  telemetryUpdate(lidarFilteredCm, (int)lroundf(heading.headingDegContinuous), heading.headingDirLabel);
  telemetryTestUpdate(lidarFilteredCm);

  const uint32_t nowMs = millis();
  if (nowMs - lastRgbMs >= 200U) {
    lastRgbMs = nowMs;
    ColorRgb rgb;
    if (colorSensor.read(rgb)) {
      telemetryRgbUpdate(rgb.r, rgb.g, rgb.b);
    }
  }

  EspCommand espCmd;
  if (espPoll(espCmd)) {
    if (roomTask.active()) roomTask.cancel();
    if (followTask.active()) followTask.cancel();
    if (driveStraight.active()) driveStraight.cancel();
    if (wallAlignTask.active()) wallAlignTask.cancel();
    if (wallSeqTask.active()) wallSeqTask.cancel();
    if (encCalTask.active()) encCalTask.cancel();
    if (seqExecTask.active()) seqExecTask.cancel();

    encoderReset();
    OdometryData od = odom.read();

    if (espCmd.type == EspCommand::Type::Move) {
      espSteps[0] = {SequenceStepType::MoveByDistance, (float)espCmd.value};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCm);
    } else if (espCmd.type == EspCommand::Type::Turn) {
      espSteps[0] = {SequenceStepType::TurnDeg, (float)espCmd.value};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCm);
    } else if (espCmd.type == EspCommand::Type::North) {
      if (!seqHeadingSet) {
        Serial.println("Seq heading not set. Press 'h' first.");
      } else {
        const float delta = wrapDegDiff180(seqHeadingHoldDeg, heading.headingDegContinuous);
        espSteps[0] = {SequenceStepType::TurnDeg, delta};
        espSteps[1] = {SequenceStepType::End, 0.0f};
        seqExecTask.setSequence(espSteps);
        seqExecTask.clearAlignHeading();
        seqExecTask.begin(heading.headingDegContinuous, od.avgCm);
      }
    }
  }

  if (Serial.available()) {
    const char c = Serial.read();
    if ((c == 'm' || c == 'M') && !roomTask.active()) {
      if (followTask.active()) followTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      roomTask.begin(heading.headingDegContinuous);
    }
    if (c == 'f' || c == 'F') {
      if (roomTask.active()) roomTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (!followTask.active()) followTask.begin(heading.headingDegContinuous, 0.0f);
    }
    if (c == 'd' || c == 'D') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      encoderReset();
      OdometryData od = odom.read();
      driveStraight.begin(heading.headingDegContinuous, od.avgCm, 20.0f, 0.5f);
    }
    if (c == 'w' || c == 'W') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      encoderReset();
      OdometryData od = odom.read();
      wallAlignTask.begin(heading.headingDegContinuous, od.avgCm);
    }
    if (c == 's' || c == 'S') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      encoderReset();
      OdometryData od = odom.read();
      wallSeqTask.begin(heading.headingDegContinuous, od.avgCm);
    }
    if (c == 'k' || c == 'K') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      encoderReset();
      OdometryData od = odom.read();
      encCalTask.begin(heading.headingDegContinuous, od.avgCm);
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
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      encoderReset();
      OdometryData od = odom.read();
      if (!seqHeadingSet) {
        Serial.println("Seq heading not set. Press 'h' first.");
      } else {
        seqExecTask.setAlignHeading(seqHeadingHoldDeg);
        seqExecTask.begin(heading.headingDegContinuous, od.avgCm);
      }
    }
    if (c == 'c' || c == 'C') {
      if (followTask.active()) followTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
    }
    if (c == 't' || c == 'T') {
      telemetryTestStart();
    }
  }

  // ---- Run tasks/actions ----
  if (seqExecTask.active()) {
    OdometryData od = odom.read();
    seqExecTask.update(heading.headingDegContinuous, od.avgCm, lidarFilteredCm);
  } else if (encCalTask.active()) {
    OdometryData od = odom.read();
    encCalTask.update(heading.headingDegContinuous, od.avgCm, lidarFilteredCm);
  } else if (wallSeqTask.active()) {
    OdometryData od = odom.read();
    wallSeqTask.update(heading.headingDegContinuous, od.avgCm, lidarFilteredCm);
  } else if (wallAlignTask.active()) {
    OdometryData od = odom.read();
    wallAlignTask.update(heading.headingDegContinuous, od.avgCm, lidarFilteredCm);
  } else if (driveStraight.active()) {
    OdometryData od = odom.read();
    driveStraight.update(heading.headingDegContinuous, od.avgCm);
  } else if (followTask.active()) {
    followTask.update(heading.headingDegContinuous, 0.0f, lidarFilteredCm);
  } else {
    roomTask.update(heading.headingDegContinuous, lidarFilteredCm);
  }
}
