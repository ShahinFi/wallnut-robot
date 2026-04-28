#include "display_sync.h"

#include "missions/maze/maze_lcd.h"
#include "odometry/world_odometry.h"
#include "odometry/odometry_manager.h"
#include "protocol_helpers.h"

static bool hasValidEspIp(const RuntimeState& rt) {
  return strncmp(rt.espIpStr, "0.0.0.0", 7) != 0;
}

void maybeRequestEspIp(RuntimeState& rt, uint32_t nowMs) {
  if (hasValidEspIp(rt)) return;
  if (rt.lastIpReqMs != 0 && (nowMs - rt.lastIpReqMs) < 1500) return;
  rt.lastIpReqMs = nowMs;
  Serial2.println("IPREQ");
}

void updateMazeLcd(RuntimeState& rt, const CompassData& heading, float lidarFilteredCm) {
  maze_lcd::setIp(rt.espIpStr);
  const maze_lcd::AuthState auth = rt.espLocked ? maze_lcd::AuthState::Locked
                                                : (rt.espArmed ? maze_lcd::AuthState::Armed : maze_lcd::AuthState::Disarmed);

  maze_lcd::AutoState astate = maze_lcd::AutoState::Idle;
  if (rt.turretSweep.active()) astate = maze_lcd::AutoState::Scan;
  else if (rt.seqExecTask.active()) astate = maze_lcd::AutoState::Run;

  const WorldOdomData w = odomWorldRead();
  const int16_t x = (int16_t)lroundf(w.eastCm);
  const int16_t y = (int16_t)lroundf(w.northCm);
  const uint16_t h = (uint16_t)lroundf(wrapDeg360(heading.headingDegWrapped));
  const uint16_t ahead = isfinite(lidarFilteredCm) ? (uint16_t)lroundf(lidarFilteredCm) : 0u;
  const uint8_t cls = (uint8_t)((rt.lastRgbClassSent < 0) ? 0 : rt.lastRgbClassSent);

  maze_lcd::update(auth, astate, x, y, h, ahead, cls);
}

