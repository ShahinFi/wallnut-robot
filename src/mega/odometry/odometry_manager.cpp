#include "odometry/odometry_manager.h"

#include "encoder/encoder.h"

namespace {
Odometry* gOdom = nullptr;
OdometryData gLast = {};

// Local baseline for "distance since local reset"
bool gLocalHasBase = false;
float gLocalBaseAvgCmSigned = 0.0f;

WorldOdomState gWorld = {};
bool gWorldInit = false;

static void refreshLast_() {
  if (!gOdom) return;
  gLast = gOdom->read();
}
}  // namespace

void odomManagerInit(Odometry* odom) {
  gOdom = odom;
  gLast = {};

  gLocalHasBase = false;
  gLocalBaseAvgCmSigned = 0.0f;

  worldOdomReset(gWorld);
  gWorldInit = true;

  // Prime cached reading/baselines (if the caller updates heading in the first loop,
  // worldOdomUpdate will rebase correctly).
  refreshLast_();
}

void odomManagerUpdate(float headingDeg) {
  if (!gOdom) return;
  if (!gWorldInit) {
    worldOdomReset(gWorld);
    gWorldInit = true;
  }

  refreshLast_();
  worldOdomUpdate(gWorld, headingDeg, gLast.avgCmSigned);
}

OdometryData odomRaw() { return gLast; }

void odomLocalReset() {
  refreshLast_();
  gLocalHasBase = true;
  gLocalBaseAvgCmSigned = gLast.avgCmSigned;
}

float odomLocalReadCmSigned() {
  refreshLast_();
  if (!gLocalHasBase) {
    gLocalHasBase = true;
    gLocalBaseAvgCmSigned = gLast.avgCmSigned;
    return 0.0f;
  }
  return gLast.avgCmSigned - gLocalBaseAvgCmSigned;
}

void odomWorldReset(float headingDeg) {
  refreshLast_();
  worldOdomReset(gWorld);
  // Establish a baseline immediately so the next update is clean.
  worldOdomRebase(gWorld, headingDeg, gLast.avgCmSigned);
}

WorldOdomData odomWorldRead() { return gWorld.pos; }

void odomWorldRebase(float headingDeg) {
  refreshLast_();
  worldOdomRebase(gWorld, headingDeg, gLast.avgCmSigned);
}

void odomHardResetKeepWorld(float headingDeg) {
  // WARNING: this resets the underlying encoder counters to 0.
  // Prefer odomLocalReset() for task/action baselining.
  encoderReset();
  refreshLast_();  // should now be near-zero
  worldOdomRebase(gWorld, headingDeg, gLast.avgCmSigned);
  // Local reset to avoid surprising "negative jump" for local callers.
  gLocalHasBase = true;
  gLocalBaseAvgCmSigned = gLast.avgCmSigned;
}

void odomHardResetAll(float headingDeg) {
  encoderReset();
  refreshLast_();
  worldOdomReset(gWorld);
  worldOdomRebase(gWorld, headingDeg, gLast.avgCmSigned);
  gLocalHasBase = true;
  gLocalBaseAvgCmSigned = gLast.avgCmSigned;
}
