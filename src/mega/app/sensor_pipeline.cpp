#include "sensor_pipeline.h"

#include "odometry/odometry_manager.h"
#include "telemetry/telemetry.h"
#include "color/color_classifier.h"

// SECTION: Color classification helpers.
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

static void maybeSendRgbClassToEsp_(RuntimeState& rt, const ColorRgb& live, bool liveValid) {
  // CONTRACT: Emit RGBCLS only on transitions to reduce UART noise.
  const int8_t cls = liveValid ? classifyColorIdx1BasedOrNone_(rt, live) : (int8_t)0;
  if (cls == rt.lastRgbClassSent) return;
  rt.lastRgbClassSent = cls;
  Serial2.print("RGBCLS:");
  Serial2.println((int)cls);
}

static void maybeLatchForwardSpeedFromColor_(RuntimeState& rt, const ColorRgb& live) {
  // CONTRACT: refs[1] maps to speed mode; other classes do not alter speed.
  if (!rt.colorCalTask.hasCalibration()) return;
  const ColorRgb* refs = rt.colorCalTask.refs();
  if (!refs) return;

  constexpr float kSpeed75 = 0.75f;

  color::ClassifyConfig cfg;
  cfg.absMaxDist = 0.18f;
  cfg.bestOverSecondMax = 0.80f;
  const color::ClassifyResult r = color::classifyNearest(live, refs, ColorCalibrationTask::kColorCount, cfg);
  if (r.idx != 1) return;

  const float desired = kSpeed75;
  if (!isfinite(desired)) return;
  if (fabsf(desired - rt.forwardSpeedScale) < 0.001f) return;
  rt.forwardSpeedScale = desired;
  rt.seqExecTask.setForwardSpeedScale(rt.forwardSpeedScale);
}

bool updateHeading(RuntimeState& rt, CompassData& headingOut) {
  // CONTRACT: Fall back to last valid heading after transient read failures.
  if (!rt.compass.read(headingOut)) {
    if (!rt.headingValid) return false;
    headingOut = rt.lastHeading;
  } else {
    rt.lastHeading = headingOut;
    rt.headingValid = true;
  }
  return true;
}

void updateTurretTracking(RuntimeState& rt) {
  const long turretTicksNow = rt.turretMotor.ticksAbs();
  rt.turretAngle.update(turretTicksNow, rt.turretMotor.lastCmd());
}

bool handleTurretSweepEarlyReturn(RuntimeState& rt, uint32_t nowMs) {
  // CONTRACT: Turret sweep owns LiDAR timing while active.
  if (!rt.turretSweep.active()) return false;
  const bool done = rt.turretSweep.update(rt.turretMotor.ticksAbs(), nowMs);
  if (done) Serial2.println("TSCAN:DONE");
  return true;
}

float updateLidarOdomTelemetry(RuntimeState& rt, const CompassData& heading) {
  float lidarCm = 0.0f;
  if (rt.lidar.update(lidarCm)) rt.lidarAvg.push(lidarCm);

  const float lidarFilteredCm = rt.lidarAvg.average();
  odomManagerUpdate(heading.headingDegContinuous);
  const WorldOdomData wOdom = odomWorldRead();
  telemetryUpdate(lidarFilteredCm, (int)lroundf(heading.headingDegContinuous), heading.headingDirLabel, wOdom.eastCm, wOdom.northCm);
  return lidarFilteredCm;
}

void updateColorAndSpeedLatch(RuntimeState& rt, uint32_t nowMs) {
  // WHY: Rate-limit color polling to reduce I2C load and serial churn.
  if (!rt.colorSensorOk || nowMs - rt.lastRgbMs < 200U) return;
  rt.lastRgbMs = nowMs;
  ColorRgb rgb;
  if (rt.colorSensor.read(rgb)) {
    rt.lastRgb = rgb;
    rt.lastRgbValid = true;
    telemetryRgbUpdate(rgb.r, rgb.g, rgb.b);
    maybeSendRgbClassToEsp_(rt, rgb, true);
    maybeLatchForwardSpeedFromColor_(rt, rgb);
  } else {
    rt.lastRgbValid = false;
    maybeSendRgbClassToEsp_(rt, rt.lastRgb, false);
  }
}

