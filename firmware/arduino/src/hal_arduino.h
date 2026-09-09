#pragma once

#include "types.h"

// Board boundary for the Arduino GIGA R1 WiFi implementation.
void hal_init();
InputSnapshot hal_readInputs();
void hal_setMotorEnable(bool enable);
void hal_setBrake(bool apply);
void hal_setMotorOutputs(float leftNorm, float rightNorm);
void hal_setStatusLED(SystemState state);
