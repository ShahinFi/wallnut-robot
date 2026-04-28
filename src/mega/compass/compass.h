#pragma once

#include <Arduino.h>
#include <Wire.h>

struct CompassData {
  // WHY: Raw register bearing value used for diagnostics.
  uint8_t bearing8;

  // WHY: Wrapped heading fields are for UI and cardinal labeling.
  float headingDegWrapped;
  int headingDegRounded;
  const char* headingDirLabel;

  // WHY: Continuous heading fields are for control loops and delta tracking.
  float headingDegContinuous;
  float deltaHeadingDeg;
};

struct CompassContinuousState {
  bool hasPrev;
  float prevHeadingDegWrapped;
  float headingDegContinuous;
  float deltaHeadingDeg;
  long wrapCount;
};

class Compass {
public:
  Compass(uint8_t i2cAddress = 0x60, uint8_t bearingReg = 0x01);

  // CONTRACT: Initializes I2C and zeros heading to current orientation; returns false on read failure.
  bool begin(TwoWire& wire = Wire);

  // CONTRACT: Reads wrapped heading and updates continuous heading in one call.
  bool read(CompassData& out);

  // SECTION: Configuration
  void setHeadingOffsetDeg(float headingOffsetDeg);
  float headingOffsetDeg() const;

  // CONTRACT: Wrapped read mutates only wrapped-output fields in CompassData.
  bool readHeadingDegWrapped(CompassData& out);
  // CONTRACT: Continuous update mutates only continuous-output fields in CompassData.
  void updateHeadingDegContinuous(CompassData& io);
  // CONTRACT: Sets heading offset so current wrapped heading becomes near 0 degrees.
  bool zeroHeadingAtCurrent();

  // SECTION: State Access
  void resetHeadingContinuous();
  const CompassContinuousState& continuousState() const;

private:
  // SECTION: Hardware Read
  bool readReg8(uint8_t reg, uint8_t& valOut);
  bool readHeadingDegRaw(float& headingDegRawOut, uint8_t& bearing8Out);

  // SECTION: Math Helpers
  static float wrapDeg360(float headingDeg);
  static const char* dirLabelFromDeg(int headingDeg);

  // SECTION: Wiring and Config
  TwoWire* wire_;
  uint8_t i2cAddress_;
  uint8_t bearingReg_;

  // SECTION: Parameters
  float headingOffsetDeg_;

  // SECTION: Continuous Tracking State
  CompassContinuousState state_;
};
