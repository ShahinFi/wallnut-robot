#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Mega ready");
}

void loop() {
  Serial.println("tick");
  delay(1000);
}
