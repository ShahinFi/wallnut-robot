#include <Arduino.h>

#include "compass/compass.h"
#include "lidar/lidar.h"
#include "lidar/utils/moving_average.h"
#include "lidar/turret/turret_motor.h"
#include "lidar/turret/turret_encoder_cal.h"
#include "lidar/turret/turret_angle_tracker.h"
#include "lidar/turret/actions/turret_sweep_scan_360.h"
#include "motor/motor.h"
#include "display/lcd.h"
#include "missions/maze/maze_lcd.h"
#include "encoder/encoder.h"
#include "odometry/odometry.h"
#include "odometry/odometry_manager.h"
#include "actions/drive_by_distance.h"
#include "tasks/encoder_calibration/encoder_calibration_task.h"
#include "tasks/sequence_executor/sequence_executor_task.h"
#include "esp/esp_uart.h"
#include "telemetry/telemetry.h"
#include "color/color_sensor.h"
#include "color/color_classifier.h"
#include "tasks/color_calibration/color_calibration_task.h"

// Optional local secrets file (gitignored): src/mega/secrets.h
#if defined(__has_include)
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#endif

// Default passcode fallback (used if secrets.h is absent).
#ifndef ESP_PASSCODE_STR
#define ESP_PASSCODE_STR "1234"
#endif
#ifndef ESP_PASSCODE_INT
#define ESP_PASSCODE_INT 1234
#endif

static Compass         compass;
static Lidar           lidar;
static MovingAverage   lidarAvg;
static TurretMotor     turretMotor;
static TurretEncoderCal turretEncCal;
static TurretAngleTracker turretAngle;
static TurretSweepScan360 turretSweep;
static uint32_t        lastCompassUiMs = 0;
static Odometry        odom(0.0f);
static DriveByDistance driveByDistance;
static EncoderCalibrationTask encCalTask;
static SequenceExecutorTask seqExecTask;
static ColorSensor     colorSensor;
static bool            colorSensorOk = false;
static uint32_t        lastRgbMs = 0;
static ColorRgb        lastRgb = {0, 0, 0};
static bool            lastRgbValid = false;
static char            gEspIpStr[16] = "0.0.0.0";
static uint32_t        gLastIpReqMs = 0;
static uint32_t        lastEncDbgMs = 0;
static bool            headingValid = false;
static CompassData     lastHeading = {};
static bool            seqHeadingSet = false;
static float           seqHeadingHoldDeg = 0.0f;

// ===== Option 1 security (UART passcode arming) =====
static const char kEspPasscode[] = ESP_PASSCODE_STR;
static const int  kEspPasscodeInt = ESP_PASSCODE_INT;
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

// ===== Browser-autonomy reflex events (UART -> ESP -> browser polls) =====
static bool gReflexLatchedRed = false;
static bool gReflexLatchedFront = false;
static bool gSeqWasActive = false;

// Latched forward speed modes driven by stored colors:
// - ref[1] => 75% forward PWM
// - ref[2] => 35% forward PWM
static float gForwardSpeedScale = 0.50f;
static int8_t gLastRgbClassSent = -1;  // 0=NONE, 1..kColorCount

static void onButtonIsr() {
  gButtonEdge = true;
}

static float wrapDegDiff180(float targetDeg, float currentDeg) {
  float d = targetDeg - currentDeg;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static bool parseCommaFloats2_(const String& s, float& aOut, float& bOut) {
  const int comma = s.indexOf(',');
  if (comma <= 0) return false;
  const float a = s.substring(0, comma).toFloat();
  const float b = s.substring(comma + 1).toFloat();
  if (!(isfinite(a) && isfinite(b))) return false;
  aOut = a;
  bOut = b;
  return true;
}

static bool parseCommaFloats3Opt_(const String& s, float& aOut, float& bOut, float& cOut, bool& hasCOut) {
  hasCOut = false;
  const int c1 = s.indexOf(',');
  if (c1 <= 0) return false;
  const int c2 = s.indexOf(',', c1 + 1);
  if (c2 < 0) {
    float a = 0.0f, b = 0.0f;
    if (!parseCommaFloats2_(s, a, b)) return false;
    aOut = a;
    bOut = b;
    return true;
  }

  const float a = s.substring(0, c1).toFloat();
  const float b = s.substring(c1 + 1, c2).toFloat();
  const float c = s.substring(c2 + 1).toFloat();
  if (!(isfinite(a) && isfinite(b) && isfinite(c))) return false;
  aOut = a;
  bOut = b;
  cOut = c;
  hasCOut = true;
  return true;
}

static float wrapDeg360_(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static float clamp_(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static bool isVirtualRed_(const ColorRgb& live) {
  // Project convention: first saved calibration color is the virtual obstacle (red).
  if (!colorCalTask.hasCalibration()) return false;
  const ColorRgb* refs = colorCalTask.refs();
  if (!refs) return false;

  color::ClassifyConfig cfg;
  cfg.absMaxDist = 0.18f;
  cfg.bestOverSecondMax = 0.80f;
  const color::ClassifyResult r = color::classifyNearest(live, refs, ColorCalibrationTask::kColorCount, cfg);
  return r.idx == 0;
}

static void sendRgbRefsToEsp_() {
  if (!colorCalTask.hasCalibration()) return;
  const ColorRgb* refs = colorCalTask.refs();
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

static int8_t classifyColorIdx1BasedOrNone_(const ColorRgb& live);

static void sendTelemetrySnapshotToEsp_() {
  // Refresh cached UI values on the ESP (useful after an ESP reset or page refresh).
  // This does not change any protocol; it only re-sends the same telemetry tags.
  sendRgbRefsToEsp_();

  // Color class (0=NONE, 1..4 = stored ref index).
  const int8_t cls = lastRgbValid ? classifyColorIdx1BasedOrNone_(lastRgb) : (int8_t)0;
  gLastRgbClassSent = cls;
  Serial2.print("RGBCLS:");
  Serial2.println((int)cls);

  // Encoder calibration (mm/pulse) if available.
  const float mmPerPulse = encCalTask.calibratedCmPerPulse() * 10.0f;
  if (isfinite(mmPerPulse) && mmPerPulse > 0.0f) {
    Serial2.print("ENC_CAL:");
    Serial2.println(mmPerPulse, 2);
  }

  // Turret ticks/rev if available.
  if (turretEncCal.hasCalibration()) {
    Serial2.print("TURCAL:");
    Serial2.print((unsigned long)turretEncCal.ticksPerRevPos());
    Serial2.print(",");
    Serial2.println((unsigned long)turretEncCal.ticksPerRevNeg());
  }
}

static int8_t classifyColorIdx1BasedOrNone_(const ColorRgb& live) {
  if (!colorCalTask.hasCalibration()) return 0;
  const ColorRgb* refs = colorCalTask.refs();
  if (!refs) return 0;
  color::ClassifyConfig cfg;
  cfg.absMaxDist = 0.18f;
  cfg.bestOverSecondMax = 0.80f;
  const color::ClassifyResult r = color::classifyNearest(live, refs, ColorCalibrationTask::kColorCount, cfg);
  if (r.idx < 0) return 0;
  return (int8_t)(r.idx + 1);
}

static void maybeSendRgbClassToEsp_(const ColorRgb& live, bool liveValid) {
  const int8_t cls = liveValid ? classifyColorIdx1BasedOrNone_(live) : (int8_t)0;
  if (cls == gLastRgbClassSent) return;
  gLastRgbClassSent = cls;
  Serial2.print("RGBCLS:");
  Serial2.println((int)cls);
}

static void maybeLatchForwardSpeedFromColor_(const ColorRgb& live) {
  // Stored colors meaning:
  // - refs[0] => virtual red obstacle (handled elsewhere)
  // - refs[1] => speed mode 75%
  // - refs[2] => speed mode 35%
  if (!colorCalTask.hasCalibration()) return;
  const ColorRgb* refs = colorCalTask.refs();
  if (!refs) return;

  constexpr float kSpeed75 = 0.75f;
  constexpr float kSpeed35 = 0.35f;

  color::ClassifyConfig cfg;
  cfg.absMaxDist = 0.18f;
  cfg.bestOverSecondMax = 0.80f;
  const color::ClassifyResult r = color::classifyNearest(live, refs, ColorCalibrationTask::kColorCount, cfg);
  if (!(r.idx == 1 || r.idx == 2)) return;

  const float desired = (r.idx == 1) ? kSpeed75 : kSpeed35;
  if (!isfinite(desired)) return;
  if (fabsf(desired - gForwardSpeedScale) < 0.001f) return;
  gForwardSpeedScale = desired;
  seqExecTask.setForwardSpeedScale(gForwardSpeedScale);
}

static void sendEvtPose_(const char* tag, float extra) {
  const WorldOdomData w = odomWorldRead();
  Serial2.print("EVT:");
  Serial2.print(tag);
  Serial2.print(",");
  Serial2.print(w.eastCm, 2);
  Serial2.print(",");
  Serial2.print(w.northCm, 2);
  Serial2.print(",");
  Serial2.print(lastHeading.headingDegWrapped, 1);
  if (isfinite(extra)) {
    Serial2.print(",");
    Serial2.print(extra, 1);
  }
  Serial2.println();
}

static void onTurretSweepSample(const TurretSweepScan360::Sample& s, void* user) {
  (void)user;
  // Thin scan stream (bandwidth-friendly).
  // TSCAN:angleDeg,distanceCm
  // Do not stream per-sample scan points anywhere except the ESP link.

  // Mirror scan stream to ESP (Serial2) so the browser can do mapping/SLAM.
  Serial2.print("TSCAN:");
  Serial2.print(s.angleDeg, 2);
  Serial2.print(",");
  Serial2.print(s.distanceCm, 1);
  Serial2.println();
}

static SequenceStep seqSteps[] = {
  {SequenceStepType::MoveToDistance, 34.5f},
  {SequenceStepType::TurnDeg, 90.0f},
  {SequenceStepType::MoveToDistance, 27.2f},
  {SequenceStepType::End, 0.0f}
};

static void mazeLcdTick_(float headingDegWrapped, float lidarFilteredCm) {
  maze_lcd::setIp(gEspIpStr);
  const maze_lcd::AuthState auth =
      gEspLocked ? maze_lcd::AuthState::Locked : (gEspArmed ? maze_lcd::AuthState::Armed : maze_lcd::AuthState::Disarmed);

  maze_lcd::AutoState astate = maze_lcd::AutoState::Idle;
  if (turretSweep.active()) astate = maze_lcd::AutoState::Scan;
  else if (seqExecTask.active()) astate = maze_lcd::AutoState::Run;

  const WorldOdomData w = odomWorldRead();
  const int16_t x = (int16_t)lroundf(w.eastCm);
  const int16_t y = (int16_t)lroundf(w.northCm);
  const uint16_t h = (uint16_t)lroundf(wrapDeg360_(headingDegWrapped));
  const uint16_t ahead = isfinite(lidarFilteredCm) ? (uint16_t)lroundf(lidarFilteredCm) : 0u;
  const uint8_t cls = (uint8_t)((gLastRgbClassSent < 0) ? 0 : gLastRgbClassSent);

  maze_lcd::update(auth, astate, x, y, h, ahead, cls);
}

static bool hasValidEspIp_() {
  // Treat anything other than the default placeholder as valid.
  return strncmp(gEspIpStr, "0.0.0.0", 7) != 0;
}

static void maybeRequestEspIp_() {
  if (hasValidEspIp_()) return;
  const uint32_t now = millis();
  if (gLastIpReqMs != 0 && (now - gLastIpReqMs) < 1500) return;
  gLastIpReqMs = now;
  Serial2.println("IPREQ");
}

void setup() {
  maze_lcd::init();

  // --- Hardware ---
  if (!compass.begin()) {
    maze_lcd::showFatal("Compass failed", "Check I2C/wiring");
    while (1) {}
  }

  if (!lidar.begin()) {
    maze_lcd::showFatal("LiDAR failed", "Check I2C/wiring");
    while (1) {}
  }

  turretMotor.begin();
  turretEncCal.loadFromEeprom();
  if (turretEncCal.hasCalibration()) {
    turretAngle.setTicksPerRevPosNeg(turretEncCal.ticksPerRevPos(), turretEncCal.ticksPerRevNeg());
  }
  turretSweep.setSampleCallback(onTurretSweepSample, nullptr);
  {
    // Default to a slow sweep so LiDAR has time to produce enough samples per revolution.
    TurretSweepScan360::Config cfg = turretSweep.config();
    cfg.cmdAbs = 0.20f;
    // Default: no extra downsampling (emit as often as LiDAR results are ready).
    cfg.sampleEveryTicks = 1;
    turretSweep.setConfig(cfg);
  }
  colorSensorOk = colorSensor.begin();

  motorInit();
  pinMode(kButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kButtonPin), onButtonIsr, FALLING);

  colorCalTask.loadFromEeprom();

  encoderInit();
  espSetup();
  // Deterministic IP delivery for LCD: request until we receive ESPIP:<ip>.
  maybeRequestEspIp_();
  telemetryInit(200);
  seqExecTask.setForwardSpeedScale(gForwardSpeedScale);
  Serial2.println("AUTH:OFF");
  // Provide reference swatches to the browser at boot if calibration exists.
  sendRgbRefsToEsp_();
  // Send last saved encoder calibration (if any) so the web UI can show it.
  const float mmPerPulse = encCalTask.calibratedCmPerPulse() * 10.0f;
  if (isfinite(mmPerPulse) && mmPerPulse > 0.0f) {
    Serial2.print("ENC_CAL:");
    Serial2.println(mmPerPulse, 2);
  }
  if (turretEncCal.hasCalibration()) {
    Serial2.print("TURCAL:");
    Serial2.print((unsigned long)turretEncCal.ticksPerRevPos());
    Serial2.print(",");
    Serial2.println((unsigned long)turretEncCal.ticksPerRevNeg());
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

  seqExecTask.setSequence(seqSteps);
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

  const uint32_t nowMs = millis();

  // ---- Turret angle tracker (single source of truth) ----
  // Updated continuously while powered. Becomes 0 only via explicit zeroing.
  const long turretTicksNow = turretMotor.ticksAbs();
  turretAngle.update(turretTicksNow, turretMotor.lastCmd());

  // ---- Turret sweep debug action (exclusive) ----
  // Owns the LiDAR timing internally (for latency compensation), so we must return
  // before running the global LiDAR update/moving-average logic.
  if (turretSweep.active()) {
    const bool done = turretSweep.update(turretTicksNow, nowMs);
    if (done) {
      Serial2.println("TSCAN:DONE");
    }
    return;
  }

  // ---- LiDAR (moving averaged) ----
  float lidarCm = 0.0f;
  if (lidar.update(lidarCm)) {
    lidarAvg.push(lidarCm);
  }

  const float lidarFilteredCm = lidarAvg.average();
  // ---- Odometry manager (local + world-frame) ----
  // Integrates world-frame East/North continuously.
  odomManagerUpdate(heading.headingDegContinuous);
  const WorldOdomData wOdom = odomWorldRead();

  telemetryUpdate(lidarFilteredCm, (int)lroundf(heading.headingDegContinuous), heading.headingDirLabel,
                  wOdom.eastCm, wOdom.northCm);

  if (colorSensorOk && nowMs - lastRgbMs >= 200U) {
    lastRgbMs = nowMs;
    ColorRgb rgb;
    if (colorSensor.read(rgb)) {
      lastRgb = rgb;
      lastRgbValid = true;
      telemetryRgbUpdate(rgb.r, rgb.g, rgb.b);
      maybeSendRgbClassToEsp_(rgb, true);
      maybeLatchForwardSpeedFromColor_(rgb);
    } else {
      lastRgbValid = false;
      maybeSendRgbClassToEsp_(lastRgb, false);
    }
  }

  // Reflex safety during browser-driven motion (low latency, no Wi-Fi dependency).
  // Latch once per commanded motion so we don't spam events.
  if (seqExecTask.active()) {
    // Safety invariant: stop immediately when an obstacle is closer than 10 cm.
    const float kFrontStopCm = 10.0f;
    if (!gReflexLatchedFront && isfinite(lidarFilteredCm) && lidarFilteredCm > 0.0f && lidarFilteredCm < kFrontStopCm) {
      gReflexLatchedFront = true;
      seqExecTask.cancel();
      motorDrive(0.0f, 0.0f);
      sendEvtPose_("FRONTSTOP", lidarFilteredCm);
      maze_lcd::notifyStopFront();
    }

    // Do not cancel a commanded backoff just because the color sensor is still
    // over the red tile at the start of the reverse move.
    if (!gReflexLatchedRed && !seqExecTask.reverseMoveActive() && lastRgbValid && isVirtualRed_(lastRgb)) {
      gReflexLatchedRed = true;
      seqExecTask.cancel();
      motorDrive(0.0f, 0.0f);
      sendEvtPose_("RED", NAN);
      maze_lcd::notifyStopRed();
    }
  }

  if (gButtonEdge) {
    if (nowMs - gLastButtonMs >= kButtonDebounceMs) {
      gButtonEdge = false;
      if (digitalRead(kButtonPin) == LOW) {
        gLastButtonMs = nowMs;
        if (!colorCalTask.active()) {
          if (driveByDistance.active()) driveByDistance.cancel();
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
    if (espCmd.type == EspCommand::Type::EspIp) {
      if (espCmd.text.length() > 0) {
        espCmd.text.toCharArray(gEspIpStr, sizeof(gEspIpStr));
        gEspIpStr[sizeof(gEspIpStr) - 1] = '\0';
        maze_lcd::setIp(gEspIpStr);
      }
      return;
    }
    if (espCmd.type == EspCommand::Type::Passcode) {
      if (gEspLocked) {
        Serial2.println("AUTH:LOCKED");
      } else if (espCmd.value == kEspPasscodeInt) {
        const bool wasArmed = gEspArmed;
        gEspArmed = true;
        gEspFailCount = 0;
        Serial2.println("AUTH:OK");
        if (!wasArmed) sendTelemetrySnapshotToEsp_();
      } else {
        gEspArmed = false;
        // If the payload was malformed (no digits parsed), do not count it as a try.
        if (espCmd.value >= 0) {
          if (gEspFailCount < 255) gEspFailCount++;
        }
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
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (gEspLocked) {
      Serial2.println("AUTH:LOCKED");
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (!gEspArmed) {
      Serial2.println("AUTH:REQUIRED");
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (espCmd.type == EspCommand::Type::MapPose) {
      float east = 0.0f, north = 0.0f, hdgMatch = 0.0f;
      bool hasH = false;
      if (parseCommaFloats3Opt_(espCmd.text, east, north, hdgMatch, hasH)) {
        // Position alignment: set world odom position in map coordinates.
        odomWorldSetPos(east, north, heading.headingDegContinuous);

        // Optional compass alignment: adjust heading offset so compass agrees with map-matched heading.
        // Keep compass as heading source of truth, and slowly steer offset toward the match.
        if (hasH) {
          const float matchWrapped = wrapDeg360_(hdgMatch);
          const float compassWrapped = wrapDeg360_(heading.headingDegWrapped);
          const float err = wrapDegDiff180(matchWrapped, compassWrapped);

          // Low-pass correction to avoid overreacting to scan noise.
          const float kAlpha = 0.25f;
          float off = compass.headingOffsetDeg();
          off += kAlpha * err;
          // Keep offset in a reasonable range for readability (functionally any range works).
          off = clamp_(off, -180.0f, 180.0f);
          compass.setHeadingOffsetDeg(off);
          compass.resetHeadingContinuous();

          // Rebase world odometry with the matched heading to avoid a fake "turn jump" in the integrator.
          odomWorldRebase(matchWrapped);
        }
      }
      return;
    }

    if (espCmd.type == EspCommand::Type::TurretZero) {
      turretEncCal.setZeroTicks(turretMotor.ticksAbs());
      // Also zero the continuous turret-to-body angle state.
      turretAngle.setZero();
      // NOTE: Keep TURCAL:* reserved for ticks/rev calibration values only.
      // Turret zero is a scan-control action, so it uses its own channel.
      Serial2.println("TURZERO:OK");
      return;
    } else if (espCmd.type == EspCommand::Type::TurretTpr) {
      // Manual override: set CW/CCW ticks/rev from UI (persist to EEPROM).
      const uint32_t pos = (uint32_t)espCmd.value;
      const uint32_t neg = (uint32_t)espCmd.value2;
      // Keep range aligned with TurretEncoderCal EEPROM sanity checks.
      if (pos < 10u || pos > 200000u || neg < 10u || neg > 200000u) {
        Serial2.println("TURCAL:FAIL");
        if (turretEncCal.hasCalibration()) {
          Serial2.print("TURCAL:");
          Serial2.print((unsigned long)turretEncCal.ticksPerRevPos());
          Serial2.print(",");
          Serial2.println((unsigned long)turretEncCal.ticksPerRevNeg());
        }
        return;
      }

      const bool okPos = turretEncCal.setTicksPerRevPos(pos);
      const bool okNeg = turretEncCal.setTicksPerRevNeg(neg);
      if (okPos && okNeg && turretEncCal.hasCalibration()) {
        turretAngle.setTicksPerRevPosNeg(turretEncCal.ticksPerRevPos(), turretEncCal.ticksPerRevNeg());
        Serial2.print("TURCAL:");
        Serial2.print((unsigned long)turretEncCal.ticksPerRevPos());
        Serial2.print(",");
        Serial2.println((unsigned long)turretEncCal.ticksPerRevNeg());
      } else {
        Serial2.println("TURCAL:FAIL");
        if (turretEncCal.hasCalibration()) {
          Serial2.print("TURCAL:");
          Serial2.print((unsigned long)turretEncCal.ticksPerRevPos());
          Serial2.print(",");
          Serial2.println((unsigned long)turretEncCal.ticksPerRevNeg());
        }
      }
      return;
    }

    // Motion exclusivity (fundamental safety invariant):
    // Never start a new motion command while another motion owner is active.
    //
    // Do NOT cancel the in-flight motion here, because the browser has no command IDs.
    // Preempt/cancel would emit completion events that can be mis-attributed to the
    // new command. Instead: reject the new request immediately and keep the current
    // motion running.
    const bool isMotionCmd =
        (espCmd.type == EspCommand::Type::Move || espCmd.type == EspCommand::Type::Turn ||
         espCmd.type == EspCommand::Type::TurnShortest || espCmd.type == EspCommand::Type::TurnAbs ||
         espCmd.type == EspCommand::Type::North || espCmd.type == EspCommand::Type::EncCal);
    const bool motorBusy =
        (seqExecTask.active() || driveByDistance.active() || encCalTask.active() || turretSweep.active());
    if (isMotionCmd && motorBusy) {
      sendEvtPose_("CMDFAIL", NAN);
      return;
    }

    // Commands below this point are motion/task-affecting.
    if (driveByDistance.active()) driveByDistance.cancel();
    if (encCalTask.active()) encCalTask.cancel();
    if (seqExecTask.active()) seqExecTask.cancel();

    // Turret scan commands (from browser). These must not be blocked by odom resets.
    if (espCmd.type == EspCommand::Type::TurretScanCancel) {
      if (turretSweep.active()) {
        turretSweep.cancel();
        Serial2.println("TSCAN:CANCEL");
      }
      motorDrive(0.0f, 0.0f);
      return;
    }
    if (espCmd.type == EspCommand::Type::TurretScanPlus ||
        espCmd.type == EspCommand::Type::TurretScanMinus) {
      if (turretSweep.active()) return;  // ignore if already scanning
      motorDrive(0.0f, 0.0f);
      const int dir = (espCmd.type == EspCommand::Type::TurretScanMinus) ? -1 : +1;
      Serial2.print("TSCAN:BEGIN,");
      Serial2.println(dir < 0 ? "-" : "+");
      turretSweep.begin(&turretMotor, &turretAngle, &lidar, dir, turretMotor.ticksAbs(), millis());
      return;
    }

    odomHardResetKeepWorld(heading.headingDegContinuous);
    OdometryData od = odomRaw();

    if (espCmd.type == EspCommand::Type::Move) {
      espSteps[0] = {SequenceStepType::MoveByDistance, (float)espCmd.value};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      // Ensure CMD completion events are never missed even if the step finishes
      // within the same loop iteration (edge detector needs "was active" true).
      gSeqWasActive = true;
      gReflexLatchedRed = false;
      gReflexLatchedFront = false;
    } else if (espCmd.type == EspCommand::Type::Turn) {
      espSteps[0] = {SequenceStepType::TurnDeg, (float)espCmd.value};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      gSeqWasActive = true;
      gReflexLatchedRed = false;
      gReflexLatchedFront = false;
    } else if (espCmd.type == EspCommand::Type::TurnShortest) {
      espSteps[0] = {SequenceStepType::TurnDegShortest, (float)espCmd.value};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      gSeqWasActive = true;
      gReflexLatchedRed = false;
      gReflexLatchedFront = false;
    } else if (espCmd.type == EspCommand::Type::TurnAbs) {
      const float targetWrapped = wrapDeg360_((float)espCmd.value);
      const float curWrapped = wrapDeg360_(heading.headingDegWrapped);
      const float delta = wrapDegDiff180(targetWrapped, curWrapped);
      espSteps[0] = {SequenceStepType::TurnDegShortest, delta};
      espSteps[1] = {SequenceStepType::End, 0.0f};
      seqExecTask.setSequence(espSteps);
      seqExecTask.clearAlignHeading();
      seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
      gSeqWasActive = true;
      gReflexLatchedRed = false;
      gReflexLatchedFront = false;
    } else if (espCmd.type == EspCommand::Type::North) {
      if (!seqHeadingSet) {
      } else {
        const float delta = wrapDegDiff180(seqHeadingHoldDeg, heading.headingDegContinuous);
        espSteps[0] = {SequenceStepType::TurnDeg, delta};
        espSteps[1] = {SequenceStepType::End, 0.0f};
        seqExecTask.setSequence(espSteps);
        seqExecTask.clearAlignHeading();
        seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
        gSeqWasActive = true;
        gReflexLatchedRed = false;
        gReflexLatchedFront = false;
      }
    } else if (espCmd.type == EspCommand::Type::SetNorth) {
      seqHeadingHoldDeg = heading.headingDegContinuous;
      seqHeadingSet = true;
    } else if (espCmd.type == EspCommand::Type::EncCal) {
      encCalTask.begin(heading.headingDegContinuous, od.avgCmSigned);
    }
  }

  // Allow ESP Disarm/Passcode handling above to work even while these are active.
  if (colorCalTask.active()) {
    colorCalTask.update(&lastRgb, lastRgbValid);
    // Publish updated ref swatches immediately after a successful save (even if
    // we already had a previous calibration).
    if (colorCalTask.consumeJustSaved()) sendRgbRefsToEsp_();
    // While color calibration is active, keep the LCD dedicated to the color
    // calibration UI (do not overwrite it with the maze mission LCD).
    return;
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
  } else if (driveByDistance.active()) {
    OdometryData od = odomRaw();
    driveByDistance.update(heading.headingDegContinuous, od.avgCmSigned);
  }

  // Command completion event (lets browser know a step finished).
  const bool seqActiveNow = seqExecTask.active();
  if (gSeqWasActive && !seqActiveNow) {
    const SequenceExecutorTask::State st = seqExecTask.state();
    if (st == SequenceExecutorTask::State::Succeeded) {
      sendEvtPose_("CMDOK", NAN);
    } else if (st == SequenceExecutorTask::State::Failed) {
      sendEvtPose_("CMDFAIL", NAN);
    } else if (st == SequenceExecutorTask::State::Cancelled) {
      sendEvtPose_("CMDCANCEL", NAN);
    }
  }
  gSeqWasActive = seqActiveNow;

  maybeRequestEspIp_();
  mazeLcdTick_(heading.headingDegWrapped, lidarFilteredCm);
}
