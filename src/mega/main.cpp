#include <Arduino.h>

#include "app/runtime.h"
#include "app/init.h"
#include "app/sensor_pipeline.h"
#include "app/reflex.h"
#include "app/color_cal_button.h"
#include "app/command_router.h"
#include "app/motion_control.h"
#include "app/display_sync.h"

void setup() {
  runtimeInit(gRt);
  initHardware(gRt);
  initCommsAndTelemetry(gRt);
  initMissionDefaults(gRt);
}

void loop() {
  const uint32_t nowMs = millis();

  updateTurretTracking(gRt);
  if (handleTurretSweepEarlyReturn(gRt, nowMs)) return;

  CompassData heading;
  if (!updateHeading(gRt, heading)) return;

  const float lidarFilteredCm = updateLidarOdomTelemetry(gRt, heading);
  updateColorAndSpeedLatch(gRt, nowMs);

  applyReflexSafety(gRt, lidarFilteredCm, heading);
  handleColorCalButton(gRt, nowMs);
  if (handleEspCommand(gRt, heading, lidarFilteredCm)) return;
  if (tickColorCalibrationTask(gRt)) return;

  tickActiveControllers(gRt, heading, lidarFilteredCm);
  emitCommandCompletionEdge(gRt);

  maybeRequestEspIp(gRt, nowMs);
  updateMazeLcd(gRt, heading, lidarFilteredCm);
}

