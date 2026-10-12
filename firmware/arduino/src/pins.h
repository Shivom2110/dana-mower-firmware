#pragma once

// Arduino GIGA R1 WiFi assignments from the approved wiring diagram.
// D2/D3 must receive protected 3.3 V logic from the 12 V input circuits;
// connect 12 V directly to neither pin.
//
// Each handle directly commands one drive wheel (skid-steer / zero-turn):
//   Left handle (A0)  → left wheel inverter  (INV3, DRIVE_LEFT role)
//   Right handle (A1) → right wheel inverter (INV4, DRIVE_RIGHT role)
#define PIN_HANDLE_LEFT  A0
#define PIN_HANDLE_RIGHT A1
#define PIN_IGNITION     2
#define PIN_ESTOP        3
// Deck (blade) motor switch; same protected 3.3 V conditioning as D2/D3.
#define PIN_DECK_SWITCH  4

// GIGA FDCAN2 logic connections to the isolated CAN transceiver.
#define PIN_CAN_RX 93
#define PIN_CAN_TX 94

// Set these to match the final conditioned-input circuit, after bench test.
#define IGNITION_ACTIVE_HIGH true
#define ESTOP_ACTIVE_LOW     true
#define DECK_SWITCH_ACTIVE_HIGH true

// Handle ADC calibration (12-bit, analogReadResolution(12)).
// Values are ×4 from bench-measured 10-bit readings (neutral=65, fwd=37, rev=97).
// Verify and update once main firmware is bench-tested with analogReadResolution(12).
#define HANDLE_ADC_BITS       12
#define HANDLE_ADC_CENTER    260   // neutral position (≈ 65 × 4)
#define HANDLE_ADC_DEADBAND   32   // ± counts around neutral (≈ 8 × 4)
#define HANDLE_ADC_MAX       112   // counts from neutral to full deflection (≈ 28 × 4)
