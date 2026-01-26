#include <Arduino.h>

#include "compass/compass.h"
#include "lidar/lidar.h"
#include "lidar/utils/moving_average.h"
#include "motor/motor.h"
#include "joystick/joystick.h"
#include "ui/display_ui.h"

static Compass compass;
static Lidar lidar;
static MovingAverage filter;
static DisplayUI ui;

void setup() {
  Serial.begin(115200);

  if (!compass.begin()) {
    Serial.println("Compass not responding! Freezing.");
    while (1) {}
  }

  if (!lidar.begin()) {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1) {}
  }

  motorInit();
  ui.begin();
  ui.setFieldHz(DisplayField::Average, 1);
}

void loop() {
  static DisplayData data = {};
  float distanceCm = 0.0f;
  if (lidar.update(distanceCm)) {
    filter.push(distanceCm);
    float avg = filter.average();

    Serial.print("New distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");

    Serial.print("Moving average: ");
    Serial.println(avg);
    Serial.println(" cm");

    data.averageCm = avg;
  }

  ui.update(data);
}
