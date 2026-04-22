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
#include "telemetry/telemetry_test.h"
#include "color/color_sensor.h"
#include "color/color_classifier.h"
#include "tasks/color_calibration/color_calibration_task.h"
#include "tasks/mapping/mapping_task.h"
#include "tasks/color_maze/color_maze_task.h"

// AVR-only free SRAM estimator (helps diagnose "stuck at boot" after adding features).
#if defined(ARDUINO_ARCH_AVR)
extern int __heap_start, *__brkval;
static int freeRamBytes() {
  int v;
  return (int)&v - (__brkval ? (int)__brkval : (int)&__heap_start);
}
#else
static int freeRamBytes() { return -1; }
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
static ColorMazeTask   colorMazeTask;
static uint32_t        lastEncDbgMs = 0;
static bool            headingValid = false;
static CompassData     lastHeading = {};
static bool            seqHeadingSet = false;
static float           seqHeadingHoldDeg = 0.0f;
static mapping::MappingTask mappingTask;
static mapping::Pose2D mappingPose = {};       // last corrected pose (map X east, Y north)
static mapping::Pose2D mappingPriorPose = {};  // prior pose for the in-flight capture
static WorldOdomData   mappingOdomAnchor = {0.0f, 0.0f};  // odom world pose at mappingPose time
static bool            mappingPoseValid = false;          // becomes true once we successfully match at least once
static bool            mappingPoseInitialized = false;    // set once we have a reasonable initial guess

// ===== Option 1 security (UART passcode arming) =====
static const char kEspPasscode[] = "1234";
static const int  kEspPasscodeInt = 1234;
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

// Quick hardware sanity test for turret direction/sign.
// Press:
// - 'u' => cmd +0.25 for 1s
// - 'j' => cmd -0.25 for 1s
static void turretDebugPulse(float cmd, uint32_t ms) {
  const long t0 = turretMotor.ticksAbs();
  const float a0 = turretAngle.angleDegSigned();

  turretMotor.setCmd(cmd);
  const uint32_t endMs = millis() + ms;
  while ((int32_t)(millis() - endMs) < 0) {
    turretAngle.update(turretMotor.ticksAbs(), turretMotor.lastCmd());
    delay(5);
  }
  turretMotor.stop();
  turretAngle.update(turretMotor.ticksAbs(), 0.0f);

  const long t1 = turretMotor.ticksAbs();
  const long dt = t1 - t0;
  const long dir = (cmd > 0.0f) ? 1L : -1L;
  const long dsignedTicks = (long)turretAngle.config().angleSign * dir * dt;
  const float a1 = turretAngle.angleDegSigned();

  Serial.print("Turret pulse cmd=");
  Serial.print(cmd, 2);
  Serial.print(" ms=");
  Serial.print((unsigned long)ms);
  Serial.print(" dticks=");
  Serial.print(dt);
  Serial.print(" dsigned_ticks=");
  Serial.print(dsignedTicks);
  Serial.print(" ddeg=");
  Serial.print(a1 - a0, 2);

  Serial.println();
}

static void turretDebugOneRev(float cmd) {
  const int dirSign = (cmd < 0.0f) ? -1 : 1;
  const uint32_t tpr = turretAngle.ticksPerRevForDirSign(dirSign);
  if (tpr == 0) {
    Serial.println("Turret 1rev: no ticksPerRev (calibrate first).");
    return;
  }

  const uint32_t kTimeoutMs = 8000;
  const long t0 = turretMotor.ticksAbs();
  const float a0 = turretAngle.angleDegSigned();

  turretMotor.setCmd(cmd);
  const uint32_t startMs = millis();
  bool timedOut = false;
  while ((uint32_t)(millis() - startMs) < kTimeoutMs) {
    const long tNow = turretMotor.ticksAbs();
    turretAngle.update(tNow, turretMotor.lastCmd());
    if ((tNow - t0) >= (long)tpr) break;
    delay(5);
  }
  if ((uint32_t)(millis() - startMs) >= kTimeoutMs) timedOut = true;

  turretMotor.stop();
  turretAngle.update(turretMotor.ticksAbs(), 0.0f);

  const long t1 = turretMotor.ticksAbs();
  const long dt = t1 - t0;
  const long dir = (cmd > 0.0f) ? 1L : -1L;
  const long dsignedTicks = (long)turretAngle.config().angleSign * dir * dt;
  const float a1 = turretAngle.angleDegSigned();

  Serial.print("Turret 1rev cmd=");
  Serial.print(cmd, 2);
  Serial.print(" targetTicks=");
  Serial.print((unsigned long)tpr);
  Serial.print(" dticks=");
  Serial.print(dt);
  Serial.print(" dsigned_ticks=");
  Serial.print(dsignedTicks);
  Serial.print(" ddeg=");
  Serial.print(a1 - a0, 2);
  if (timedOut) Serial.print(" (TIMEOUT)");
  Serial.println();
}

static void handleAtCommand(const String& cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  if (cmd.length() == 0) return;

  // @tpr 1234   => set BOTH + and - ticks/rev (persist)
  // @tpr=1234   => same
  // @tpr+ 1234  => set + only (persist)
  // @tpr- 1234  => set - only (persist)
  if (cmd.startsWith("tpr")) {
    const char ch = (cmd.length() > 3) ? cmd.charAt(3) : '\0';
    const bool setPosOnly = (ch == '+');
    const bool setNegOnly = (ch == '-');

    int prefixLen = 3;
    if (setPosOnly || setNegOnly) prefixLen = 4;

    String rest = cmd.substring(prefixLen);
    rest.trim();
    if (rest.startsWith("=")) rest = rest.substring(1);
    rest.trim();

    const long v = rest.toInt();
    if (v <= 0) {
      Serial.println("Usage: @tpr 1234 | @tpr+ 1234 | @tpr- 1234");
      return;
    }

    const uint32_t tpr = (uint32_t)v;
    bool ok = false;
    if (setPosOnly) ok = turretEncCal.setTicksPerRevPos(tpr);
    else if (setNegOnly) ok = turretEncCal.setTicksPerRevNeg(tpr);
    else ok = turretEncCal.setTicksPerRev(tpr);

    Serial.print("Turret set ticksPerRev");
    if (setPosOnly) Serial.print("+");
    else if (setNegOnly) Serial.print("-");
    Serial.print("=");
    Serial.print((unsigned long)tpr);
    Serial.println(ok ? " OK" : " FAIL");

    if (ok) {
      turretAngle.setTicksPerRevPosNeg(turretEncCal.ticksPerRevPos(), turretEncCal.ticksPerRevNeg());
      Serial2.print("TURCAL:");
      Serial2.print((unsigned long)turretEncCal.ticksPerRevPos());
      Serial2.print(",");
      Serial2.println((unsigned long)turretEncCal.ticksPerRevNeg());
    }
    return;
  }

  Serial.print("Unknown @cmd: ");
  Serial.println(cmd);
}

static void onTurretSweepSample(const TurretSweepScan360::Sample& s, void* user) {
  (void)user;
  // Thin scan stream (bandwidth-friendly).
  // TSCAN:angleDeg,distanceCm
  // Do not stream per-sample scan points to USB Serial (too noisy for debugging).
  // Keep streaming to Serial2 for browser-side mapping.

  // Mirror scan stream to ESP (Serial2) so the browser can do mapping/SLAM.
  Serial2.print("TSCAN:");
  Serial2.print(s.angleDeg, 2);
  Serial2.print(",");
  Serial2.print(s.distanceCm, 1);
  Serial2.println();

  // Mapping capture hook (thin): store the raw scan points for later matching/replay.
  if (mappingTask.capturing()) {
    mappingTask.onSample(s.angleDeg, s.distanceCm);
  }
}

static SequenceStep seqSteps[] = {
  {SequenceStepType::MoveToDistance, 34.5f},
  {SequenceStepType::TurnDeg, 90.0f},
  {SequenceStepType::MoveToDistance, 27.2f},
  {SequenceStepType::End, 0.0f}
};

static void mappingInitPoseDefaults_() {
  const auto gc = mappingTask.config().grid;
  // Default initial guess inside the map:
  // - X: centered
  // - Y: within the [0..0.4*H] placement band (we choose 0.2*H as a neutral midpoint)
  mappingPose.x_cm = 0.5f * (float)gc.mapW_cm;
  mappingPose.y_cm = 0.2f * (float)gc.mapH_cm;
  mappingPose.headingDeg = 0.0f;
  mappingPriorPose = mappingPose;
  mappingPoseValid = false;
  mappingPoseInitialized = true;
}

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
  Serial.begin(115200);
  maze_lcd::init();
  {
    Serial.print("Free RAM at boot: ");
    Serial.println(freeRamBytes());
  }

  // --- Hardware ---
  if (!compass.begin()) {
    Serial.println("Compass failed");
    maze_lcd::showFatal("Compass failed", "Check I2C/wiring");
    while (1) {}
  }

  if (!lidar.begin()) {
    Serial.println("LiDAR failed");
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
  // Mapping defaults (tunable).
  {
    mapping::MappingTask::Config cfg;
    cfg.grid.mapW_cm = 46;
    cfg.grid.mapH_cm = 26;
    cfg.grid.cell_cm = 2;
    // Map/world coordinates are in absolute cm from the map's bottom-left corner:
    // X in [0..mapW], Y in [0..mapH].
    cfg.grid.originX_cm = 0.0f;
    cfg.grid.originY_cm = 0.0f;
    cfg.enableMatching = true;
    cfg.lidarOffset.x_cm = 0.0f;
    cfg.lidarOffset.y_cm = 0.0f;
    mappingTask.setConfig(cfg);
  }
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

  // Initialize mapping pose defaults + anchor now that odom state exists.
  mappingInitPoseDefaults_();
  mappingOdomAnchor = odomWorldRead();

  // --- DriveByDistance test config ---
  DriveByDistance::Config dcfg = driveByDistance.config();
  dcfg.maxSpeed = 0.6f;
  dcfg.minSpeed = 0.2f;
  dcfg.timeoutMs = 8000;
  driveByDistance.setConfig(dcfg);

  seqExecTask.setSequence(seqSteps);

  Serial.println("Ready. Send 'd' drive, 'w' wall, 's' seq, 'k' cal, 'h' set heading, 'q' exec.");
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
      Serial.println("TSCAN:DONE");
      Serial2.println("TSCAN:DONE");
      if (mappingTask.capturing()) {
        // If we don't yet have a validated map pose, use a larger initial search:
        // - X: full map width
        // - Y: [0..0.4*mapH] as the initial placement band (user-defined)
        // - Heading: still local around compass (kept thin)
        if (!mappingPoseValid && mappingTask.grid().totalHits() > 0) {
          const auto gc = mappingTask.config().grid;
          mapping::CorrelativeMatcher::SearchWindow w;
          w.xMin_cm = 0.0f;
          w.xMax_cm = (float)gc.mapW_cm;
          w.yMin_cm = 0.0f;
          w.yMax_cm = 0.4f * (float)gc.mapH_cm;
          const auto mc = mappingTask.config().matcher;
          w.headingMin_deg = mappingPriorPose.headingDeg - mc.searchDHeadingDeg;
          w.headingMax_deg = mappingPriorPose.headingDeg + mc.searchDHeadingDeg;
          w.stepX_cm = mc.step_cm;
          w.stepY_cm = mc.step_cm;
          w.stepHeading_deg = mc.stepHeadingDeg;
          mappingTask.finishCaptureAndUpdateInWindow(w, mappingPriorPose, mappingPose);
        } else {
          mappingTask.finishCaptureAndUpdate(mappingPriorPose, mappingPose);
        }
        mappingTask.printSummary();
        // Keep "valid" sticky once we have a match; don't drop it on a single failure.
        if (mappingTask.lastMatchScore() >= 0) mappingPoseValid = true;
        // Only advance the anchor if we actually applied the update (or seeded the map).
        if (mappingTask.lastUpdateApplied()) {
          mappingOdomAnchor = odomWorldRead();
        }
      }
    }
    if (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == 'x' || c == 'X') {
        turretSweep.cancel();
        Serial.println("TSCAN:CANCEL");
        Serial2.println("TSCAN:CANCEL");
      }
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
  telemetryTestUpdate(lidarFilteredCm);

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
    const float kFrontStopCm = 9.0f;
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
        gEspArmed = true;
        gEspFailCount = 0;
        Serial2.println("AUTH:OK");
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
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (gEspLocked) {
      Serial2.println("AUTH:LOCKED");
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);
      return;
    }

    if (!gEspArmed) {
      Serial2.println("AUTH:REQUIRED");
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
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

          Serial.print("Compass offset adjusted by ");
          Serial.print(kAlpha * err, 2);
          Serial.print(" deg (err=");
          Serial.print(err, 2);
          Serial.print(") newOff=");
          Serial.println(off, 2);
        }

        Serial.print("Map pose set (odom world): east=");
        Serial.print(east, 2);
        Serial.print(" north=");
        Serial.println(north, 2);
      } else {
        Serial.print("Bad MapPose payload: ");
        Serial.println(espCmd.text);
      }
      return;
    }

    if (espCmd.type == EspCommand::Type::TurretZero) {
      turretEncCal.setZeroTicks(turretMotor.ticksAbs());
      // Also zero the continuous turret-to-body angle state.
      turretAngle.setZero();
      Serial.println("Turret zero set");
      Serial2.println("TURCAL:ZEROED");
      return;
    } else if (espCmd.type == EspCommand::Type::TurretTpr) {
      // Manual override: set CW/CCW ticks/rev from UI (persist to EEPROM).
      const uint32_t pos = (uint32_t)espCmd.value;
      const uint32_t neg = (uint32_t)espCmd.value2;
      // Keep range aligned with TurretEncoderCal EEPROM sanity checks.
      if (pos < 10u || pos > 200000u || neg < 10u || neg > 200000u) {
        Serial.println("Turret TPR out of range");
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
        Serial.println("Turret TPR set");
        Serial2.print("TURCAL:");
        Serial2.print((unsigned long)turretEncCal.ticksPerRevPos());
        Serial2.print(",");
        Serial2.println((unsigned long)turretEncCal.ticksPerRevNeg());
      } else {
        Serial.println("Turret TPR set failed");
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

    // Commands below this point are motion/task-affecting.
    if (driveByDistance.active()) driveByDistance.cancel();
    if (encCalTask.active()) encCalTask.cancel();
    if (seqExecTask.active()) seqExecTask.cancel();
    if (colorMazeTask.active()) colorMazeTask.cancel();

    // Turret scan commands (from browser). These must not be blocked by odom resets.
    if (espCmd.type == EspCommand::Type::TurretScanCancel) {
      if (turretSweep.active()) {
        turretSweep.cancel();
        Serial.println("TSCAN:CANCEL");
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
      Serial.print("TSCAN:BEGIN,");
      Serial.println(dir < 0 ? "-" : "+");
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
      Serial.print("DBG:CMD BEGIN MOVE cm=");
      Serial.println(espCmd.value);
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
      Serial.print("DBG:CMD BEGIN TURN deg=");
      Serial.println(espCmd.value);
      gSeqWasActive = true;
      gReflexLatchedRed = false;
      gReflexLatchedFront = false;
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
        gSeqWasActive = true;
        gReflexLatchedRed = false;
        gReflexLatchedFront = false;
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

  if (Serial.available()) {
    const char peek = (char)Serial.peek();
    if (peek == '@') {
      Serial.read();  // consume '@'
      const String line = Serial.readStringUntil('\n');
      handleAtCommand(line);
      return;
    }

    const char c = Serial.read();

    // Serial control should always be able to take over. The color calibration
    // task is initiated by a physical button, but it must not lock out console
    // hotkeys (including plot_tscan.py sending mapping commands).
    if (colorCalTask.active()) colorCalTask.cancel();
    if (colorMazeTask.active()) colorMazeTask.cancel();
    if (c == 'd' || c == 'D') {
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      odomHardResetKeepWorld(heading.headingDegContinuous);
      OdometryData od = odomRaw();
      driveByDistance.beginByDistance(heading.headingDegContinuous, od.avgCmSigned, 20.0f, 0.5f);
    }
    if (c == 'k' || c == 'K') {
      if (driveByDistance.active()) driveByDistance.cancel();
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
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      odomHardResetKeepWorld(heading.headingDegContinuous);
      OdometryData od = odomRaw();
      if (!seqHeadingSet) {
        Serial.println("Seq heading not set. Press 'h' first.");
      } else {
        seqExecTask.setAlignHeading(seqHeadingHoldDeg);
        seqExecTask.begin(heading.headingDegContinuous, od.avgCmSigned);
        gSeqWasActive = true;
      }
    }
    if (c == 'c' || c == 'C') {
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
    }
    if (c == 't' || c == 'T') {
      telemetryTestStart();
    }
    if (c == 'u' || c == 'U') {
      Serial.print("Turret ticksPerRev+= ");
      Serial.print((unsigned long)turretAngle.ticksPerRevPos());
      Serial.print(" ticksPerRev-= ");
      Serial.println((unsigned long)turretAngle.ticksPerRevNeg());
      turretDebugPulse(+0.25f, 1000);
    }
    if (c == 'j' || c == 'J') {
      Serial.print("Turret ticksPerRev+= ");
      Serial.print((unsigned long)turretAngle.ticksPerRevPos());
      Serial.print(" ticksPerRev-= ");
      Serial.println((unsigned long)turretAngle.ticksPerRevNeg());
      turretDebugPulse(-0.25f, 1000);
    }
    if (c == 'o' || c == 'O') {
      Serial.print("Turret ticksPerRev+= ");
      Serial.print((unsigned long)turretAngle.ticksPerRevPos());
      Serial.print(" ticksPerRev-= ");
      Serial.println((unsigned long)turretAngle.ticksPerRevNeg());
      turretDebugOneRev(+0.25f);
    }
    if (c == 'l' || c == 'L') {
      Serial.print("Turret ticksPerRev+= ");
      Serial.print((unsigned long)turretAngle.ticksPerRevPos());
      Serial.print(" ticksPerRev-= ");
      Serial.println((unsigned long)turretAngle.ticksPerRevNeg());
      turretDebugOneRev(-0.25f);
    }
    if (c == 'p' || c == 'P') {
      // Turret 1-rev scan (positive direction).
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);

      Serial.println("TSCAN:BEGIN,+");
      Serial2.println("TSCAN:BEGIN,+");
      turretSweep.begin(&turretMotor, &turretAngle, &lidar, +1, turretMotor.ticksAbs(), millis());
    }
    if (c == 'i' || c == 'I') {
      // Turret 1-rev scan (negative direction).
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);

      Serial.println("TSCAN:BEGIN,-");
      Serial2.println("TSCAN:BEGIN,-");
      turretSweep.begin(&turretMotor, &turretAngle, &lidar, -1, turretMotor.ticksAbs(), millis());
    }
    if (c == 'b' || c == 'B') {
      // Mapping: capture + scan, then match + update map on DONE.
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);

      if (!mappingPoseInitialized) {
        mappingInitPoseDefaults_();
        mappingOdomAnchor = odomWorldRead();
      }

      const WorldOdomData w = odomWorldRead();
      const float dx = w.eastCm - mappingOdomAnchor.eastCm;
      const float dy = w.northCm - mappingOdomAnchor.northCm;
      mappingPriorPose.x_cm = mappingPose.x_cm + dx;
      mappingPriorPose.y_cm = mappingPose.y_cm + dy;
      mappingPriorPose.headingDeg = heading.headingDegContinuous;
      mappingPose = mappingPriorPose;
      mappingTask.beginCapture(mappingPriorPose);

      Serial.println("MAP:CAPTURE_BEGIN");
      Serial.println("TSCAN:BEGIN,+");
      Serial2.println("TSCAN:BEGIN,+");
      turretSweep.begin(&turretMotor, &turretAngle, &lidar, +1, turretMotor.ticksAbs(), millis());
    }
    if (c == 'n' || c == 'N') {
      // Mapping: capture - scan, then match + update map on DONE.
      if (driveByDistance.active()) driveByDistance.cancel();
      if (encCalTask.active()) encCalTask.cancel();
      if (seqExecTask.active()) seqExecTask.cancel();
      if (colorMazeTask.active()) colorMazeTask.cancel();
      motorDrive(0.0f, 0.0f);

      if (!mappingPoseInitialized) {
        mappingInitPoseDefaults_();
        mappingOdomAnchor = odomWorldRead();
      }

      const WorldOdomData w = odomWorldRead();
      const float dx = w.eastCm - mappingOdomAnchor.eastCm;
      const float dy = w.northCm - mappingOdomAnchor.northCm;
      mappingPriorPose.x_cm = mappingPose.x_cm + dx;
      mappingPriorPose.y_cm = mappingPose.y_cm + dy;
      mappingPriorPose.headingDeg = heading.headingDegContinuous;
      mappingPose = mappingPriorPose;
      mappingTask.beginCapture(mappingPriorPose);

      Serial.println("MAP:CAPTURE_BEGIN");
      Serial.println("TSCAN:BEGIN,-");
      Serial2.println("TSCAN:BEGIN,-");
      turretSweep.begin(&turretMotor, &turretAngle, &lidar, -1, turretMotor.ticksAbs(), millis());
    }
    if (c == 'v' || c == 'V') {
      mappingTask.printMap(1);
    }
    if (c == 'r' || c == 'R') {
      mappingTask.resetMap();
      mappingInitPoseDefaults_();
      odomWorldReset(heading.headingDegContinuous);
      odomLocalReset();
      mappingOdomAnchor = odomWorldRead();
      Serial.println("MAP:RESET");
    }
  }

  // Allow ESP Disarm/Passcode handling above to work even while these are active.
  if (colorCalTask.active()) {
    const bool hadCal = colorCalTask.hasCalibration();
    colorCalTask.update(&lastRgb, lastRgbValid);
    // When calibration finishes and becomes available, publish the refs once so
    // the web UI can show stable swatches for CLASS.
    if (!hadCal && colorCalTask.hasCalibration()) sendRgbRefsToEsp_();
    mazeLcdTick_(heading.headingDegWrapped, lidarFilteredCm);
    return;
  }

  if (colorMazeTask.active()) {
    OdometryData od = odomRaw();
    colorMazeTask.update(heading.headingDegContinuous, od.avgCmSigned, &lastRgb, lastRgbValid);
    mazeLcdTick_(heading.headingDegWrapped, lidarFilteredCm);
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
      Serial.println("DBG:CMD DONE -> CMDOK");
      sendEvtPose_("CMDOK", NAN);
    } else if (st == SequenceExecutorTask::State::Failed) {
      Serial.println("DBG:CMD DONE -> CMDFAIL");
      sendEvtPose_("CMDFAIL", NAN);
    } else if (st == SequenceExecutorTask::State::Cancelled) {
      Serial.println("DBG:CMD DONE -> CMDCANCEL");
      sendEvtPose_("CMDCANCEL", NAN);
    } else {
      Serial.println("DBG:CMD DONE -> (unknown state)");
    }
  }
  gSeqWasActive = seqActiveNow;

  maybeRequestEspIp_();
  mazeLcdTick_(heading.headingDegWrapped, lidarFilteredCm);
}
