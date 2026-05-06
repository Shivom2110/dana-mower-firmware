/*
 * pins.h — Arduino Uno hardware pin assignments for DANA Mower
 *
 * ALL pin numbers live in this one file. If you rewire the board, change it
 * here — the rest of the firmware never refers to a raw pin number.
 *
 * Arduino Uno hardware facts:
 *   PWM-capable digital pins : 3, 5, 6, 9, 10, 11
 *   External-interrupt pins  : 2 (INT0), 3 (INT1)
 *   Analog input pins        : A0–A5  (10-bit ADC, 0–1023)
 *   Built-in LED             : 13
 */

#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// ANALOG INPUTS — Joystick axes
//
// A two-axis joystick potentiometer connects here. At rest (center), the ADC
// reads ~512. Full deflection reads 0 or 1023. The HAL normalizes to -1..+1.
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_JOY_THROTTLE   A0    // Forward / backward axis
#define PIN_JOY_STEERING   A1    // Left / right axis

// ─────────────────────────────────────────────────────────────────────────────
// DIGITAL INPUTS — Operator safety switches
//
// All three switches are wired to use INPUT_PULLUP:
//   • Switch OPEN  (not pressed) → pin reads HIGH → flag = false (inactive)
//   • Switch CLOSED (pressed)    → pin reads LOW  → flag = true  (active)
//
// This is a fail-safe pattern: a broken or unplugged wire reads HIGH (inactive),
// which is the safe default. The e-stop wire being cut, for example, should NOT
// accidentally CLEAR the stop — this wiring ensures it does the opposite.
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_ESTOP            2   // Red mushroom e-stop button
                                 // On INT0 — can attach a hardware interrupt later
                                 // for instant response without polling.

#define PIN_OPERATOR_ENABLE  3   // Operator dead-man / enable switch
                                 // On INT1 — same interrupt upgrade path.

#define PIN_FAULT_ACK        4   // Fault-acknowledge / reset button

// ─────────────────────────────────────────────────────────────────────────────
// DIGITAL OUTPUTS — Motor controller gate and brake
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_MOTOR_ENABLE     7   // HIGH → inverters allowed to run
                                 // LOW  → inverters locked out, ignore speed commands

#define PIN_BRAKE_RELAY      8   // HIGH → brakes APPLIED   (safe / stopped)
                                 // LOW  → brakes RELEASED  (drive allowed)
                                 //
                                 // ASSUMPTION: relay coil energized = brakes ON.
                                 // This is fail-safe: loss of power clamps brakes.
                                 // If your relay has opposite sense, flip the
                                 // HIGH/LOW in hal_arduino.cpp — NOT here.

// ─────────────────────────────────────────────────────────────────────────────
// PWM OUTPUTS — Motor speed commands
//
// CURRENT STATUS: Placeholder only. analogWrite() generates a 0–255 PWM signal
// that is useful for bench-testing wiring and motor continuity.
//
// FUTURE: The TM4 inverters communicate via RS232 serial packets. Once the
// inverter protocol is documented, hal_setMotorOutputs() will be updated to
// send formatted serial commands instead of raw PWM. The pin assignments here
// may change to TX/RX lines. The rest of the firmware is unaffected.
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_MOTOR_LEFT       9   // Left  wheel motor speed (Timer 1 channel A)
#define PIN_MOTOR_RIGHT     10   // Right wheel motor speed (Timer 1 channel B)

// ─────────────────────────────────────────────────────────────────────────────
// DIGITAL OUTPUTS — Status LEDs
//
// Assumed wiring: LED anode → current-limiting resistor (~220 Ω) → Arduino pin
//                LED cathode → GND
// HIGH = LED on, LOW = LED off.
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_LED_IDLE        12   // Blue or green — SAFE_IDLE state
#define PIN_LED_MANUAL      13   // Yellow — MANUAL drive active (also onboard LED)
#define PIN_LED_FAULT       11   // Red    — FAULT state; blinks via software

// ─────────────────────────────────────────────────────────────────────────────
// JOYSTICK CALIBRATION CONSTANTS
//
// These numbers match a typical 5 kΩ two-axis joystick potentiometer.
// Measure your specific joystick's center and range with a quick sketch
// that prints analogRead() and adjust these values to match.
// ─────────────────────────────────────────────────────────────────────────────
#define JOY_CENTER     512    // Expected ADC value at mechanical center (0–1023)
#define JOY_DEADBAND    40    // ±40 ADC counts around center → zero output
                              // Prevents motor creep when joystick is released
#define JOY_MAX        450    // ADC counts from center to full-scale deflection
                              // Slightly less than 512 to account for pot
                              // non-linearity; output is clamped to -1..+1 anyway
