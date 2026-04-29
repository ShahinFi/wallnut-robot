#pragma once

#include <Arduino.h>

class TurretMotor;
class TurretAngleTracker;
class TurretCompass;
class Lidar;

// WHY: Runs a one-revolution turret sweep and emits angle-distance samples.
// CONTRACT: Sweep completion is tick-based with timeout safety fallback.
class TurretSweepScan360 {
public:
  struct Config {
    enum class DoneMode : uint8_t { EncoderTicks, TurretCompass };

    float cmdAbs = 0.25f;
    uint32_t timeoutMs = 12000;
    DoneMode doneMode = DoneMode::EncoderTicks;
    // CONTRACT: sampleEveryTicks==0 enables auto cadence from targetSamplesPerRev.
    uint16_t sampleEveryTicks = 1;
    uint16_t targetSamplesPerRev = 360;
  };

  struct Sample {
    uint32_t seq = 0;
    long ticksAbs = 0;
    float angleDeg = 0.0f;
    float distanceCm = 0.0f;
    uint32_t ms = 0;
  };

  using SampleCallback = void (*)(const Sample& s, void* user);

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Cancelled, NoCalibration };

  TurretSweepScan360();

  void setConfig(const Config& cfg);
  const Config& config() const;

  void setSampleCallback(SampleCallback cb, void* user);

  // CONTRACT: Starts one sweep using calibration from the provided angle tracker.
  void begin(TurretMotor* motor, TurretAngleTracker* angleTracker,
             TurretCompass* turretCompass, Lidar* lidar, int dirSign, long ticksAbsNow, uint32_t nowMs);

  // CONTRACT: Returns true when the sweep reaches any terminal state.
  bool update(long ticksAbsNow, uint32_t nowMs);

  void cancel();
  void reset();

  bool active() const;
  State state() const;

private:
  void stopMotor_();
  void emitSample_(long ticksAbsNow, float lidarDistanceCm, uint32_t nowMs, float forcedAngleDeg);

  Config cfg_;
  State state_;

  TurretMotor* motor_;
  TurretAngleTracker* angle_;
  TurretCompass* turretCompass_;
  Lidar* lidar_;
  int dirSign_;

  long startTicksAbs_;
  long lastSampleTicksAbs_;
  float startAngleDegWrapped_;
  uint32_t ticksPerRev_;
  uint32_t sampleEveryTicks_;
  uint32_t seq_;
  float lastCmd_;

  uint32_t startMs_;
  float startCompassContinuousDeg_;
  float targetCompassContinuousDeg_;
  bool compassTargetValid_;
  float prevCompassWrappedDeg_;
  bool prevCompassValid_;

  // WHY: LiDAR measurement stamping (for latency compensation).
  bool lidarInFlight_;
  long lidarTicksStart_;
  float lidarCompassStartContinuousDeg_;
  bool lidarCompassStartValid_;
  uint32_t finishGraceStartMs_;

  SampleCallback cb_;
  void* cbUser_;
};
