#pragma once

#include "runtime.h"

void onButtonIsr();
bool handleColorCalButton(RuntimeState& rt, uint32_t nowMs);
bool tickColorCalibrationTask(RuntimeState& rt);

