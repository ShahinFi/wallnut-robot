#pragma once

#include <Arduino.h>

#include "actions/drive_by_distance.h"
#include "tasks/encoder_calibration/ui/encoder_calibration_ui.h"

// WHY: EncoderCalibrationTask: calibrate distance per encoder pulse using LiDAR.
class EncoderCalibrationTask {
public:
  enum class State : uint8_t { Idle, CheckStart, Driving, Compute, Save, Succeeded, Failed, Cancelled };

  EncoderCalibrationTask();

  void begin(float headingDegContinuous, float avgTravelCm);
  bool update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm);

  void cancel();
  void reset();

  bool active() const;
  State state() const;

  // WHY: Read last calibrated value (cm per pulse)
  float calibratedCmPerPulse() const;

private:
  void setState_(State s);
  bool saveToEeprom_(float cmPerPulse);
  bool loadFromEeprom_(float& cmPerPulseOut) const;

  State state_;
  DriveByDistance drive_;
  EncoderCalibrationUI ui_;

  float startDistanceCm_;
  float targetDistanceCm_;
  float startAvgTravelCm_;
  float lastCmPerPulse_;
};
