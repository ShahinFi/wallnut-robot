#pragma once
#include <Arduino.h>

void encoderInit();

long encoderGetLeft();
long encoderGetRight();
// CONTRACT: Hard-resets raw encoder counters to zero for both wheels.
// CONTRACT: Callers must re-base dependent odometry state before further integration.
// WHY: Prefer local odometry baselines or odometry-manager reset APIs for normal workflows.
void encoderReset();
