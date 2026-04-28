#pragma once

#include <Arduino.h>

#include "compass/compass.h"
#include "lidar/lidar.h"
#include "lidar/utils/moving_average.h"
#include "lidar/turret/turret_motor.h"
#include "lidar/turret/turret_encoder_cal.h"
#include "lidar/turret/turret_angle_tracker.h"
#include "lidar/turret/actions/turret_sweep_scan_360.h"
#include "odometry/odometry.h"
#include "tasks/encoder_calibration/encoder_calibration_task.h"
#include "tasks/sequence_executor/sequence_executor_task.h"
#include "color/color_sensor.h"
#include "tasks/color_calibration/color_calibration_task.h"

struct RuntimeState {
  Compass compass;
  Lidar lidar;
  MovingAverage lidarAvg;
  TurretMotor turretMotor;
  TurretEncoderCal turretEncCal;
  TurretAngleTracker turretAngle;
  TurretSweepScan360 turretSweep;
  Odometry odom{0.0f};
  EncoderCalibrationTask encCalTask;
  SequenceExecutorTask seqExecTask;
  ColorSensor colorSensor;
  ColorCalibrationTask colorCalTask;

  bool colorSensorOk;
  uint32_t lastRgbMs;
  ColorRgb lastRgb;
  bool lastRgbValid;

  char espIpStr[16];
  uint32_t lastIpReqMs;

  bool headingValid;
  CompassData lastHeading;

  bool espArmed;
  uint8_t espFailCount;
  bool espLocked;

  SequenceStep espSteps[2];
  SequenceStep seqSteps[4];

  volatile bool buttonEdge;
  uint32_t lastButtonMs;

  bool reflexLatchedRed;
  bool reflexLatchedFront;
  bool seqWasActive;

  float forwardSpeedScale;
  int8_t lastRgbClassSent;
};

extern RuntimeState gRt;

void runtimeInit(RuntimeState& rt);

