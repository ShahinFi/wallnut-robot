#include <Arduino.h>

#include "compass/compass.h"
#include "lidar/lidar.h"
#include "lidar/utils/moving_average.h"
#include "motor/motor.h"

#include "tasks/room_measure/room_measure_task.h"

static Compass         compass;
static Lidar           lidar;
static MovingAverage   lidarAvg;
static RoomMeasureTask roomTask;

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

  // --- Room measurement physical config ONLY ---
  RoomMeasureTask::Config cfg;
  cfg.lidarToCenterOffsetCm = 18.0f;   // measured once
  cfg.ceilingHeightCm       = 100.0f;  // task requirement

  roomTask.setConfig(cfg);
  roomTask.reset();

  Serial.println("Room measurement ready. Send 'm' to start.");
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

  // ---- Start measurement ----
  if (Serial.available()) {
    const char c = Serial.read();
    if ((c == 'm' || c == 'M') && !roomTask.active()) {
      roomTask.begin(heading.headingDegContinuous);
    }
  }

  // ---- Run task ----
  roomTask.update(heading.headingDegContinuous, lidarFilteredCm);
}
