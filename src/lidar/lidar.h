#pragma once

class Lidar {
public:
  Lidar();
  bool begin();
  float getDistance();
  bool update(float &distanceCm);

private:
  bool measuring;
};
