#pragma once

class MovingAverage {
public:
  static const int N = 5;

  MovingAverage();
  void push(float x);
  float average() const;

private:
  float buf[N];
};
