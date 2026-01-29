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
static uint32_t        lastEncDbgMs = 0;

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

  motorInit();
  lcdInit();
  encoderInit();

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

  Serial.println("Ready. Send 'm' room, 'f' follow, 'd' drive, 'w' wall, 's' seq, 'k' cal.");
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

  if (Serial.available()) {
    const char c = Serial.read();
    if ((c == 'm' || c == 'M') && !roomTask.active()) {
      if (followTask.active()) followTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      roomTask.begin(heading.headingDegContinuous);
    }
    if (c == 'f' || c == 'F') {
      if (roomTask.active()) roomTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (!followTask.active()) followTask.begin(heading.headingDegContinuous, 0.0f);
    }
    if (c == 'd' || c == 'D') {
      if (roomTask.active()) roomTask.cancel();
      if (followTask.active()) followTask.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
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
      encoderReset();
      OdometryData od = odom.read();
      encCalTask.begin(heading.headingDegContinuous, od.avgCm);
    }
    if (c == 'c' || c == 'C') {
      if (followTask.active()) followTask.cancel();
      if (driveStraight.active()) driveStraight.cancel();
      if (wallAlignTask.active()) wallAlignTask.cancel();
      if (wallSeqTask.active()) wallSeqTask.cancel();
      if (encCalTask.active()) encCalTask.cancel();
    }
  }

  // ---- Run tasks/actions ----
  if (encCalTask.active()) {
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
