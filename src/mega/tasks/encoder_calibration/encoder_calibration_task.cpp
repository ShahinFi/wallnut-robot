#include "tasks/encoder_calibration/encoder_calibration_task.h"

#include <math.h>
#include <EEPROM.h>
#include "encoder/encoder.h"
#include "odometry/odometry_manager.h"

namespace {
const float    kTargetDeltaCm     = 20.0f;
const float    kToleranceCm       = 1.0f;
const float    kMinValidCm        = 5.0f;
const float    kMaxValidCm        = 800.0f;
const float    kDriveSpeed        = 0.4f;
const uint32_t kDriveTimeoutMs    = 20000;
const uint16_t kEepromMagic       = 0xCA1B;
const int      kEepromAddr        = 0;
}

EncoderCalibrationTask::EncoderCalibrationTask()
: state_(State::Idle),
  drive_(),
  ui_(),
  startDistanceCm_(0.0f),
  targetDistanceCm_(0.0f),
  startAvgTravelCm_(0.0f),
  lastCmPerPulse_(0.0f) {
  DriveByDistance::Config dcfg = drive_.config();
  dcfg.slowDownCm          = 0.0f;
  dcfg.minSpeed            = kDriveSpeed;
  dcfg.maxSpeed            = kDriveSpeed;
  dcfg.timeoutMs           = kDriveTimeoutMs;
  drive_.setConfig(dcfg);

  float loaded = 0.0f;
  if (loadFromEeprom_(loaded)) {
    lastCmPerPulse_ = loaded;
  }
}

void EncoderCalibrationTask::begin(float headingDegContinuous, float avgTravelCm) {
  ui_.begin();
  ui_.showIdle();
  drive_.reset();
  // WHY: Hard reset is intentional here because we measure raw pulses over a known delta distance.
  // WHY: Use the odometry manager wrapper to keep world-frame odometry consistent.
  odomHardResetKeepWorld(headingDegContinuous);
  startAvgTravelCm_ = avgTravelCm;
  (void)headingDegContinuous;
  setState_(State::CheckStart);
}

bool EncoderCalibrationTask::update(float headingDegContinuous, float avgTravelCm, float lidarAvgCm) {
  if (state_ == State::Idle) return true;
  if (state_ == State::Succeeded || state_ == State::Failed || state_ == State::Cancelled) return true;

  const bool valid = isfinite(lidarAvgCm) && lidarAvgCm >= kMinValidCm && lidarAvgCm <= kMaxValidCm;

  switch (state_) {
    case State::CheckStart: {
      ui_.showRunning(lidarAvgCm, 0.0f, 0.0f, EncoderCalibrationUI::Phase::CheckStart);
      if (!valid) {
        ui_.showFailed("Invalid LiDAR");
        setState_(State::Failed);
        return true;
      }
      startDistanceCm_ = lidarAvgCm;
      targetDistanceCm_ = startDistanceCm_ - kTargetDeltaCm;
      if (targetDistanceCm_ < kMinValidCm) {
        ui_.showFailed("Too close");
        setState_(State::Failed);
        return true;
      }
      drive_.beginContinuous(headingDegContinuous, avgTravelCm, kDriveSpeed);
      drive_.setHeadingHoldDeg(headingDegContinuous);
      setState_(State::Driving);
      return false;
    }

    case State::Driving: {
      ui_.showRunning(lidarAvgCm, 0.0f, 0.0f, EncoderCalibrationUI::Phase::Driving);
      if (!valid) {
        drive_.cancel();
        ui_.showFailed("Invalid LiDAR");
        setState_(State::Failed);
        return true;
      }
      if (lidarAvgCm <= targetDistanceCm_ + kToleranceCm) {
        drive_.cancel();
        setState_(State::Compute);
        return false;
      }
      const bool done = drive_.update(headingDegContinuous, avgTravelCm);
      if (done && drive_.timedOut()) {
        ui_.showFailed("Drive timeout");
        setState_(State::Failed);
        return true;
      }
      return false;
    }

    case State::Compute: {
      const long l = encoderGetLeft();
      const long r = encoderGetRight();
      const long avgPulses = (labs(l) + labs(r)) / 2;
      if (avgPulses <= 0) {
        ui_.showFailed("No pulses");
        setState_(State::Failed);
        return true;
      }
      const float cmPerPulse = kTargetDeltaCm / (float)avgPulses;
      lastCmPerPulse_ = cmPerPulse;
      ui_.showRunning(lidarAvgCm, cmPerPulse, (float)avgPulses, EncoderCalibrationUI::Phase::Compute);
      setState_(State::Save);
      return false;
    }

    case State::Save: {
      const bool ok = saveToEeprom_(lastCmPerPulse_);
      if (!ok) {
        ui_.showFailed("EEPROM fail");
        setState_(State::Failed);
        return true;
      }
      ui_.showSaved(lastCmPerPulse_);
      setState_(State::Succeeded);
      return true;
    }

    default:
      return true;
  }
}

void EncoderCalibrationTask::cancel() {
  drive_.cancel();
  ui_.showFailed("Cancelled");
  setState_(State::Cancelled);
}

void EncoderCalibrationTask::reset() {
  drive_.reset();
  ui_.begin();
  ui_.showIdle();
  setState_(State::Idle);
}

bool EncoderCalibrationTask::active() const {
  return state_ == State::CheckStart || state_ == State::Driving ||
         state_ == State::Compute || state_ == State::Save;
}

EncoderCalibrationTask::State EncoderCalibrationTask::state() const { return state_; }

float EncoderCalibrationTask::calibratedCmPerPulse() const { return lastCmPerPulse_; }

void EncoderCalibrationTask::setState_(State s) {
  state_ = s;
}

bool EncoderCalibrationTask::saveToEeprom_(float cmPerPulse) {
  EEPROM.put(kEepromAddr, kEepromMagic);
  EEPROM.put(kEepromAddr + sizeof(kEepromMagic), cmPerPulse);
  return true;
}

bool EncoderCalibrationTask::loadFromEeprom_(float& cmPerPulseOut) const {
  uint16_t magic = 0;
  EEPROM.get(kEepromAddr, magic);
  if (magic != kEepromMagic) return false;
  EEPROM.get(kEepromAddr + sizeof(kEepromMagic), cmPerPulseOut);
  return isfinite(cmPerPulseOut) && cmPerPulseOut > 0.0f;
}
