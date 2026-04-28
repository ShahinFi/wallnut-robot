#pragma once

#include <Arduino.h>

#include "actions/drive_by_distance.h"

// SECTION: LiDAR-targeted distance primitive with heading hold.
// WHY: Uses LiDAR for stop condition and delegated steering for heading stability.
class DriveToDistance {
public:
  struct Config {
    float    toleranceCm = 2.0f;
    float    minValidCm  = 5.0f;
    float    maxValidCm  = 800.0f;
    uint32_t timeoutMs   = 20000;
  };

  enum class State : uint8_t { Idle, Running, Succeeded, TimedOut, Failed, Cancelled };

  DriveToDistance();

  void setConfig(const Config& cfg);
  const Config& config() const;

  // WHY: Exposes steering-loop tuning without duplicating those controls.
  void setDriveConfig(const DriveByDistance::Config& cfg);
  const DriveByDistance::Config& driveConfig() const;

  void begin(float headingDegContinuous, float avgTravelCm,
             float targetLidarCm, float speedAbs);
  void setSpeedAbs(float speedAbs);
  bool update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  bool succeeded() const;
  bool timedOut() const;
  bool failed() const;
  State state() const;

  float targetDistanceCm() const;
  float errorCm() const;

private:
  void setState_(State s);

  Config cfg_;
  State state_;

  DriveByDistance drive_;

  float targetLidarCm_;
  float speedAbs_;
  float errorCm_;

  uint32_t startMs_;
};
