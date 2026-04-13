#pragma once

#include <Arduino.h>
#include <Wire.h>

struct CompassData {
  // Sensor register
  uint8_t bearing8;               // 0..255

  // Wrapped heading (UI-friendly)
  float       headingDegWrapped;  // 0..360 after offset + wrap
  int         headingDegRounded;  // rounded 0..359
  const char* headingDirLabel;    // N/NE/E/...

  // Continuous heading (control-friendly)
  float headingDegContinuous;     // unwrapped (can grow +/-)
  float deltaHeadingDeg;          // last step (signed)
};

struct CompassContinuousState {
  bool  hasPrev;
  float prevHeadingDegWrapped;
  float headingDegContinuous;
  float deltaHeadingDeg;
  long  wrapCount;
};

class Compass {
public:
  Compass(uint8_t i2cAddress = 0x60, uint8_t bearingReg = 0x01);

  // Initializes I2C + zeros heading at current + resets continuous.
  // Returns false if the compass cannot be read.
  bool begin(TwoWire& wire = Wire);

  // Main operation (single call in loop):
  // - reads wrapped heading into out
  // - updates continuous heading into out
  bool read(CompassData& out);

  // Configuration
  void  setHeadingOffsetDeg(float headingOffsetDeg);
  float headingOffsetDeg() const;

  // Modular building blocks (kept public for testing/advanced use)
  bool readHeadingDegWrapped(CompassData& out);        // fills ONLY wrapped fields
  void updateHeadingDegContinuous(CompassData& io);    // fills ONLY continuous fields
  bool zeroHeadingAtCurrent();                         // sets offset so current becomes ~0°

  // State access
  void resetHeadingContinuous();
  const CompassContinuousState& continuousState() const;

private:
  // Hardware read
  bool readReg8(uint8_t reg, uint8_t& valOut);
  bool readHeadingDegRaw(float& headingDegRawOut, uint8_t& bearing8Out);

  // Math helpers
  static float wrapDeg360(float headingDeg);
  static const char* dirLabelFromDeg(int headingDeg);

  // Wiring/config
  TwoWire* wire_;
  uint8_t  i2cAddress_;
  uint8_t  bearingReg_;

  // Parameters
  float headingOffsetDeg_;

  // Continuous tracking state (history)
  CompassContinuousState state_;
};
