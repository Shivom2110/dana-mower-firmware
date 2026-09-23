#pragma once

// Arduino GIGA R1 WiFi assignments from the approved wiring diagram.
// D2/D3 must receive protected 3.3 V logic from the 12 V input circuits;
// connect 12 V directly to neither pin.
#define PIN_JOY_THROTTLE A0
#define PIN_JOY_STEERING A1
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

#define JOY_ADC_BITS       12
#define JOY_ADC_CENTER   2048
#define JOY_ADC_DEADBAND  160
#define JOY_ADC_MAX      1900
