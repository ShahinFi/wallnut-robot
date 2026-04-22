#pragma once

#include <stdint.h>

namespace maze_lcd {

enum class AuthState : uint8_t { Disarmed = 0, Pending = 1, Armed = 2, Locked = 3 };
enum class AutoState : uint8_t { Idle = 0, Scan = 1, Move = 2, Turn = 3, Run = 4 };

void init();

// IP string should be dotted quad (e.g. "192.168.0.104") or "0.0.0.0" if unknown.
void setIp(const char* ip);

// Latch last stop event for display (does not affect behavior).
void notifyStopRed();
void notifyStopFront();

// Update dashboard view (call periodically).
// Units:
// - x/y: absolute cm in map frame (integers)
// - headingDeg: 0..359
// - aheadCm: forward LiDAR distance in cm (integer; 0 if unknown)
// - colorClass: 0=NONE, 1..4 stored color index
void update(AuthState auth, AutoState autoState, int16_t x_cm, int16_t y_cm, uint16_t headingDeg, uint16_t aheadCm,
            uint8_t colorClass);

// Show fatal screen (for sensor init failures).
void showFatal(const char* line2, const char* line3);

}  // namespace maze_lcd

