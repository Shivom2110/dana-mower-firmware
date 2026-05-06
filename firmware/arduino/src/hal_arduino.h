/*
 * hal_arduino.h — Arduino Hardware Abstraction Layer for DANA Mower
 *
 * ─── WHY A HAL? ────────────────────────────────────────────────────────────
 *
 * Without a HAL, main.cpp calls analogRead(), digitalWrite(), and analogWrite()
 * directly and is tightly coupled to Arduino hardware. Porting to STM32 means
 * rewriting main.cpp, safety.cpp, and every other file that touches hardware.
 *
 * With a HAL, main.cpp only calls the five functions declared below. Porting to
 * STM32 means writing hal_stm32.cpp with the same function signatures — and the
 * rest of the firmware compiles unchanged.
 *
 * ─── USAGE PATTERN ─────────────────────────────────────────────────────────
 *
 *   setup()  → call hal_init()
 *   loop()   → call hal_readInputs()           (get sensor data)
 *              call hal_setMotorEnable(bool)    (enable/disable inverters)
 *              call hal_setBrake(bool)          (apply/release brakes)
 *              call hal_setMotorOutputs(L, R)   (send speed commands)
 *              call hal_setStatusLED(state)     (update status LEDs)
 *
 * ─── PORTING TO STM32 ──────────────────────────────────────────────────────
 *
 *   1. Copy this header to firmware/stm32/src/hal_stm32.h (same signatures).
 *   2. Write hal_stm32.cpp using STM32 HAL library calls.
 *   3. In main.cpp change: #include "hal_arduino.h"  →  #include "hal_stm32.h"
 *   4. Done — no other file changes needed.
 */

#pragma once
#include "types.h"    // InputSnapshot, SystemState (from firmware/common/include)

// ─────────────────────────────────────────────────────────────────────────────
// hal_init()
//
// Configure all GPIO, PWM, and ADC pins and set every output to a safe
// default (motors off, brakes on, LEDs off). Call once in setup().
// ─────────────────────────────────────────────────────────────────────────────
void hal_init();

// ─────────────────────────────────────────────────────────────────────────────
// hal_readInputs()
//
// Sample every sensor and switch, normalize the joystick axes (-1..+1 with
// deadband filtering), invert the active-LOW safety switches to active-HIGH
// logic, and return a populated InputSnapshot.
//
// The caller (main.cpp) receives clean, hardware-agnostic data and never needs
// to know which pin is analog, which is digital, or how the joystick is wired.
// ─────────────────────────────────────────────────────────────────────────────
InputSnapshot hal_readInputs();

// ─────────────────────────────────────────────────────────────────────────────
// hal_setMotorEnable(bool enable)
//
// Assert or de-assert the inverter enable line.
//   enable = true  → inverters allowed to accept speed commands
//   enable = false → inverters locked; ignore all speed commands
//
// Disabling here does NOT apply brakes — call hal_setBrake(true) separately
// to hold the machine stationary.
// ─────────────────────────────────────────────────────────────────────────────
void hal_setMotorEnable(bool enable);

// ─────────────────────────────────────────────────────────────────────────────
// hal_setBrake(bool apply)
//
// Control the spring-applied / electrically-released brake relay.
//   apply = true  → relay energized → brakes CLAMPED  (safe / stopped)
//   apply = false → relay releases  → brakes RELEASED (drive permitted)
//
// Fail-safe assumption: if the Arduino loses power or the relay wire is cut,
// the relay de-energizes and brakes engage automatically. Verify this matches
// your actual relay wiring before driving the machine.
// ─────────────────────────────────────────────────────────────────────────────
void hal_setBrake(bool apply);

// ─────────────────────────────────────────────────────────────────────────────
// hal_setMotorOutputs(float leftNorm, float rightNorm)
//
// Send a speed command to both wheel motors.
//   leftNorm, rightNorm : -1.0 (full reverse) .. 0.0 (stop) .. +1.0 (full fwd)
//
// CURRENT IMPLEMENTATION — PWM placeholder:
//   Converts -1..+1 to 0..255 and calls analogWrite(). Useful for bench-testing
//   wiring, ESC testers, or any PWM-input motor driver on the workbench.
//
// FUTURE — RS232 serial packets:
//   The TM4 inverters used on this mower accept speed commands via RS232.
//   Once the packet format is known, replace the analogWrite() calls inside
//   hal_arduino.cpp with a serial formatter — the caller never changes.
// ─────────────────────────────────────────────────────────────────────────────
void hal_setMotorOutputs(float leftNorm, float rightNorm);

// ─────────────────────────────────────────────────────────────────────────────
// hal_setStatusLED(SystemState state)
//
// Drive the three status LEDs based on the current system state. Only one LED
// is lit at a time, except FAULT which blinks at ~2 Hz using millis(). The
// blinking logic is non-blocking — safe to call every loop iteration.
//   SAFE_IDLE → IDLE LED solid on
//   MANUAL    → MANUAL LED solid on
//   FAULT     → FAULT LED blinking
// ─────────────────────────────────────────────────────────────────────────────
void hal_setStatusLED(SystemState state);
