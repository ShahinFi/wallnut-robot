#pragma once

// Initialize motor pins and stop motors.
void motorInit();

// Drive motors with normalized commands in [-1..1].
void motorDrive(float leftCmd, float rightCmd);

// Per-wheel scaling to compensate hardware imbalance.
// Example: motorSetScale(1.15f, 1.00f) boosts the left wheel by 15%.
void motorSetScale(float leftScale, float rightScale);

// Read current scales (optional convenience).
float motorLeftScale();
float motorRightScale();
