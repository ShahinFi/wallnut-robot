#pragma once

#include <Arduino.h>

#include "actions/drive_straight.h"
#include "actions/turn_to_angle.h"
#include "color/color_sensor.h"
#include "tasks/color_maze/ui/color_maze_ui.h"

class ColorMazeTask {
public:
  struct Config {
    float driveSpeed = 0.30f;
    float backoffCm = 5.0f;
    float turnDeg = 30.0f;
    uint32_t cooldownMs = 400;
    float leftThreshold = 0.18f;
    float rightThreshold = 0.18f;
    float endThreshold = 0.14f;
    bool useNormalized = true;
  };

  enum class State : uint8_t { Idle, Running, Backoff, Turn, Done, Failed };

  ColorMazeTask();

  void setConfig(const Config& cfg);
  void setCalibration(const ColorRgb refs[3], bool valid);
  void begin(float headingDegContinuous, float avgTravelCm);
  bool update(float headingDegContinuous, float avgTravelCm, const ColorRgb* live, bool liveValid);
  void cancel();
  void reset();

  bool active() const;
  State state() const;

private:
  enum class ColorClass : uint8_t { Floor, Left, Right, End, Unknown };
  enum class TurnDir : uint8_t { None, Left, Right };

  void setState_(State s);
  ColorClass classify_(const ColorRgb& rgb) const;
  static float distSq_(float r, float g, float b, const ColorRgb& ref, bool normalized);

  Config cfg_;
  State state_;
  ColorRgb refs_[3];
  bool refsValid_;

  DriveStraight drive_;
  TurnToAngle   turn_;
  ColorMazeUI   ui_;

  TurnDir pendingTurn_;
  uint32_t cooldownUntilMs_;
};
