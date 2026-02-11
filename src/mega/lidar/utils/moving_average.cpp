#include "moving_average.h"

MovingAverage::MovingAverage() {
  for (int i = 0; i < N; ++i) buf[i] = 0.0f;
}

void MovingAverage::push(float x) {
  for (int i = N - 1; i > 0; --i) {
    buf[i] = buf[i - 1];
  }
  buf[0] = x;
}

float MovingAverage::average() const {
  float sum = 0.0f;
  for (int i = 0; i < N; ++i) sum += buf[i];
  return sum / N;
}
