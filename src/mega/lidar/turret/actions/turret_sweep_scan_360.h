#pragma once

#include <Arduino.h>

class TurretMotor;
class TurretAngleTracker;

// TurretSweepScan360: rotate turret by one revolution (based on calibrated ticks/rev)
// while emitting (angle, distance) samples.
//
// Notes:
// - Uses encoder ticks as the stop condition (not time).
// - Keeps motor command constant (open-loop), with a safety timeout.
// - Sampling cadence is tick-based (auto: ~targetSamplesPerRev samples per rev).
class TurretSweepScan360 {
public:
  struct Config {
    float cmdAbs = 0.25f;               // motor command magnitude (0..1)
    uint32_t timeoutMs = 12000;         // safety stop
    uint16_t targetSamplesPerRev = 360; // auto sampling density
  };

  struct Sample {
    uint32_t seq = 0;
    long ticksAbs = 0;
    float angleDeg = 0.0f;   // turret-to-body angle (wrapped 0..360)
    float distanceCm = 0.0f; // caller-provided
    uint32_t ms = 0;
  };

  using SampleCallback = void (*)(const Sample& s, void* user);

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Cancelled, NoCalibration };

  TurretSweepScan360();

  void setConfig(const Config& cfg);
  const Config& config() const;

  void setSampleCallback(SampleCallback cb, void* user);

  // Start a 1-rev sweep. dirSign selects direction (+1 or -1).
  // ticksPerRev is taken from the provided angle tracker.
  void begin(TurretMotor* motor, TurretAngleTracker* angleTracker,
             int dirSign, long ticksAbsNow, uint32_t nowMs);

  // Tick with latest sensor values. Returns true when finished (any terminal state).
  bool update(long ticksAbsNow, float lidarDistanceCm, uint32_t nowMs);

  void cancel();
  void reset();

  bool active() const;
  State state() const;

private:
  void stopMotor_();
  void emitSample_(long ticksAbsNow, float lidarDistanceCm, uint32_t nowMs);

  Config cfg_;
  State state_;

  TurretMotor* motor_;
  TurretAngleTracker* angle_;
  int dirSign_;

  long startTicksAbs_;
  long lastSampleTicksAbs_;
  uint32_t ticksPerRev_;
  uint32_t sampleEveryTicks_;
  uint32_t seq_;
  float lastCmd_;

  uint32_t startMs_;

  SampleCallback cb_;
  void* cbUser_;
};
