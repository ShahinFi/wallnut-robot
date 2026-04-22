#pragma once

class MovingAverage {
public:
  static const int N = 1;

  MovingAverage();
  void push(float x);
  float average() const;

private:
  float buf[N];
  bool initialized;
};
