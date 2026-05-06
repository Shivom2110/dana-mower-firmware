/*
 * hal_stm32.h — STM32 Hardware Abstraction Layer for DANA Mower
 *
 * This header is INTENTIONALLY IDENTICAL in function signatures to
 * firmware/arduino/src/hal_arduino.h.
 *
 * Why identical?
 *   main.cpp uses only these five functions to interact with hardware.
 *   To switch platforms, change one line in main.cpp:
 *
 *     #include "hal_arduino.h"   →   #include "hal_stm32.h"
 *
 *   Every other file (safety.cpp, manual_control.cpp, state_machine.h, …)
 *   is pure C++ with no platform dependency and compiles unchanged.
 *
 * ─── FUTURE IMPROVEMENT ─────────────────────────────────────────────────────
 *
 * When both hal_arduino.h and hal_stm32.h are in use simultaneously (e.g.,
 * during a transition or in a CI build that compiles both targets), consider
 * extracting a shared interface file:
 *
 *   firmware/common/include/hal_interface.h
 *     — declares all five functions as pure virtual or as a C header
 *     — both hal_arduino.cpp and hal_stm32.cpp implement it
 *
 * For now, duplicating the signatures here keeps things simple.
 */

#pragma once
#include "types.h"    // InputSnapshot, SystemState (firmware/common/include)

// ─────────────────────────────────────────────────────────────────────────────
// hal_init()
//
// Initialize all STM32 peripherals to a safe state. Call once in main() after
// the CubeMX-generated HAL_Init() and MX_*_Init() peripheral inits complete.
//
// This function does NOT call HAL_Init() — CubeMX's generated main.c handles
// system clock, GPIO clock enables, and peripheral initialization. hal_init()
// only sets output levels to safe defaults (motors off, brakes on).
// ─────────────────────────────────────────────────────────────────────────────
void hal_init();

// ─────────────────────────────────────────────────────────────────────────────
// hal_readInputs()
//
// Read all sensors and switches, normalize joystick axes, invert active-LOW
// safety signals, and return a populated InputSnapshot.
//
// ADC reads use polling (HAL_ADC_PollForConversion). For production firmware
// with tighter timing, switch to DMA-mode continuous conversion.
// ─────────────────────────────────────────────────────────────────────────────
InputSnapshot hal_readInputs();

// ─────────────────────────────────────────────────────────────────────────────
// hal_setMotorEnable(bool enable)
//
// Assert or de-assert the inverter enable GPIO line.
// ─────────────────────────────────────────────────────────────────────────────
void hal_setMotorEnable(bool enable);

// ─────────────────────────────────────────────────────────────────────────────
// hal_setBrake(bool apply)
//
// Control the brake relay GPIO line. Same fail-safe assumption as Arduino:
// HIGH = brakes applied. Verify relay wiring before running.
// ─────────────────────────────────────────────────────────────────────────────
void hal_setBrake(bool apply);

// ─────────────────────────────────────────────────────────────────────────────
// hal_setMotorOutputs(float leftNorm, float rightNorm)
//
// Set wheel motor speed via TIM3 CH1/CH2 PWM compare registers.
//   -1.0 → timer compare = 0            (full reverse)
//    0.0 → timer compare = PERIOD / 2   (zero speed)
//   +1.0 → timer compare = PERIOD       (full forward)
//
// FUTURE: Replace with TM4 RS232 serial commands once the protocol is known.
// ─────────────────────────────────────────────────────────────────────────────
void hal_setMotorOutputs(float leftNorm, float rightNorm);

// ─────────────────────────────────────────────────────────────────────────────
// hal_setStatusLED(SystemState state)
//
// Drive the three status LEDs. Identical contract to the Arduino version:
// one LED lit at a time, FAULT LED blinks at ~2 Hz using HAL_GetTick().
// ─────────────────────────────────────────────────────────────────────────────
void hal_setStatusLED(SystemState state);
