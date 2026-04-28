#include "init.h"

#include "encoder/encoder.h"
#include "esp/esp_uart.h"
#include "missions/maze/maze_lcd.h"
#include "motor/motor.h"
#include "odometry/odometry_manager.h"
#include "color_cal_button.h"
#include "display_sync.h"
#include "command_router.h"
#include "telemetry/telemetry.h"

// SECTION: Turret scan sample forwarding.
void onTurretSweepSample(const TurretSweepScan360::Sample& s, void*) {
  Serial2.print("TSCAN:");
  Serial2.print(s.angleDeg, 2);
  Serial2.print(",");
  Serial2.print(s.distanceCm, 1);
  Serial2.println();
}

void initHardware(RuntimeState& rt) {
  // SECTION: Hardware and safety-critical sensor initialization.
  maze_lcd::init();

  if (!rt.compass.begin()) {
    maze_lcd::showFatal("Compass failed", "Check I2C/wiring");
    while (1) {}
  }
  if (!rt.lidar.begin()) {
    maze_lcd::showFatal("LiDAR failed", "Check I2C/wiring");
    while (1) {}
  }

  rt.turretMotor.begin();
  rt.turretEncCal.loadFromEeprom();
  if (rt.turretEncCal.hasCalibration()) {
    rt.turretAngle.setTicksPerRevPosNeg(rt.turretEncCal.ticksPerRevPos(), rt.turretEncCal.ticksPerRevNeg());
  }
  rt.turretSweep.setSampleCallback(onTurretSweepSample, nullptr);
  {
    TurretSweepScan360::Config cfg = rt.turretSweep.config();
    cfg.cmdAbs = 0.20f;
    cfg.sampleEveryTicks = 1;
    rt.turretSweep.setConfig(cfg);
  }

  rt.colorSensorOk = rt.colorSensor.begin();
  motorInit();
  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), onButtonIsr, FALLING);

  rt.colorCalTask.loadFromEeprom();
  encoderInit();
}

void initCommsAndTelemetry(RuntimeState& rt) {
  // SECTION: ESP link and telemetry initialization.
  espSetup();
  maybeRequestEspIp(rt, millis());
  telemetryInit(200);
  rt.seqExecTask.setForwardSpeedScale(rt.forwardSpeedScale);
  Serial2.println("AUTH:OFF");
  sendTelemetrySnapshotToEsp(rt);
}

void initMissionDefaults(RuntimeState& rt) {
  // SECTION: Mission defaults and odometry wiring.
  rt.odom.setPulsesPerMeter(787.0f);
  odomManagerInit(&rt.odom);
  rt.seqExecTask.setSequence(rt.seqSteps);
}

