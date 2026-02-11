#pragma once
#include <Arduino.h>

void encoderInit();

long encoderGetLeft();
long encoderGetRight();
void encoderReset();
