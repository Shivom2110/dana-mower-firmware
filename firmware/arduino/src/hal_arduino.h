#pragma once

#include "types.h"

// Board boundary for the Arduino GIGA R1 WiFi implementation.
void hal_init();
// Call on every loop pass so CAN RX is drained and RPDOs keep their 10 ms period.
void hal_service(uint32_t nowMs);
InputSnapshot hal_readInputs();
// enable=false stops the motors gracefully (0 rpm hold, then PWM off).
// Wheel commands are -1..+1; the deck runs at a fixed speed when deckOn.
void hal_setDrive(bool enable, float leftNorm, float rightNorm, bool deckOn);
// Immediate PWM disable on every inverter (fault / E-stop path).
void hal_forceMotorsOff();
void hal_setBrake(bool apply);
void hal_setStatusLED(SystemState state);
void hal_printStatus(SystemState state);
