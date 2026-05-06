/*
 * hal_arduino.cpp — Arduino HAL implementation for DANA Mower
 *
 * This is the ONLY file in the entire firmware that calls Arduino hardware
 * functions (analogRead, digitalRead, digitalWrite, analogWrite, millis).
 *
 * Every other source file (main.cpp, safety.cpp, manual_control.cpp) is
 * hardware-agnostic and compiles unchanged on any platform.
 *
 * When migrating to STM32: write hal_stm32.cpp using the STM32 HAL library,
 * keep the same function signatures declared in hal_arduino.h, and swap the
 * #include in main.cpp. Nothing else changes.
 */

#include <Arduino.h>
#include "hal_arduino.h"
#include "pins.h"

// ─────────────────────────────────────────────────────────────────────────────
// File-private helpers
// (static = visible only inside this .cpp file, not exported)
// ─────────────────────────────────────────────────────────────────────────────

/*
 * normalizeJoystick(rawADC)
 *
 * Converts a raw 10-bit ADC reading (0–1023) from a joystick potentiometer
 * into a clean normalized float in -1.0 .. +1.0.
 *
 * Step-by-step:
 *   1. Subtract JOY_CENTER (ideally 512) → signed offset, -512 .. +511
 *   2. Deadband: if |offset| < JOY_DEADBAND, snap to 0 to prevent creep
 *   3. Divide by JOY_MAX to scale to -1..+1
 *   4. Clamp output to exactly -1..+1 (handles pot non-linearity / over-travel)
 *
 * Why deadband?
 *   Joystick wiper contacts have mechanical play. At rest, the ADC might read
 *   505 instead of 512, producing a small but non-zero output that causes
 *   the mower to drift. The deadband converts that to exactly zero.
 */
static float normalizeJoystick(int rawADC) {
    int centered = rawADC - JOY_CENTER;

    if (centered > -JOY_DEADBAND && centered < JOY_DEADBAND) {
        return 0.0f;
    }

    float normalized = (float)centered / (float)JOY_MAX;
    return clamp(normalized, -1.0f, 1.0f);
}

/*
 * floatToPWM(norm)
 *
 * Maps a normalized speed command (-1..+1) to an 8-bit PWM value (0..255).
 *
 *   norm = -1.0  →  pwm =   0  (full reverse)
 *   norm =  0.0  →  pwm = 127  (stopped / zero speed)
 *   norm = +1.0  →  pwm = 255  (full forward)
 *
 * IMPORTANT — This mapping is a PLACEHOLDER:
 *   A 0–255 PWM duty cycle has no standard meaning to a TM4 inverter. It is
 *   only useful for bench-testing with an ESC, servo tester, or oscilloscope.
 *
 *   When the TM4 RS232 protocol is documented, replace the analogWrite() calls
 *   in hal_setMotorOutputs() with serial packet transmissions. This helper
 *   function will no longer be needed and can be deleted at that point.
 */
static uint8_t floatToPWM(float norm) {
    float scaled = (norm + 1.0f) * 127.5f;   // map -1..+1 → 0.0..255.0
    int   pwm    = (int)scaled;
    if (pwm < 0)   pwm = 0;
    if (pwm > 255) pwm = 255;
    return (uint8_t)pwm;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public HAL functions
// ─────────────────────────────────────────────────────────────────────────────

void hal_init() {
    // ── Analog inputs ────────────────────────────────────────────────────────
    // analogRead() handles ADC pins automatically; explicit pinMode(INPUT) is
    // optional but documents intent clearly for a future reader.
    pinMode(PIN_JOY_THROTTLE,    INPUT);
    pinMode(PIN_JOY_STEERING,    INPUT);

    // ── Digital inputs — safety switches (active-LOW, internal pull-up) ─────
    // INPUT_PULLUP enables the Uno's ~50 kΩ internal resistor to Vcc.
    // The pin reads HIGH when the switch is open (safe default) and LOW when
    // the switch is closed (active). A disconnected wire reads HIGH = inactive,
    // which is the correct fail-safe behavior for an e-stop circuit.
    pinMode(PIN_ESTOP,           INPUT_PULLUP);
    pinMode(PIN_OPERATOR_ENABLE, INPUT_PULLUP);
    pinMode(PIN_FAULT_ACK,       INPUT_PULLUP);

    // ── Digital / PWM outputs ────────────────────────────────────────────────
    pinMode(PIN_MOTOR_ENABLE,    OUTPUT);
    pinMode(PIN_BRAKE_RELAY,     OUTPUT);
    pinMode(PIN_MOTOR_LEFT,      OUTPUT);   // PWM via analogWrite
    pinMode(PIN_MOTOR_RIGHT,     OUTPUT);   // PWM via analogWrite
    pinMode(PIN_LED_IDLE,        OUTPUT);
    pinMode(PIN_LED_MANUAL,      OUTPUT);
    pinMode(PIN_LED_FAULT,       OUTPUT);

    // ── Safe power-on defaults ───────────────────────────────────────────────
    // These writes happen last so that if a future addition to hal_init() ever
    // crashes partway through, the last thing done is asserting a safe state.
    digitalWrite(PIN_MOTOR_ENABLE, LOW);   // Inverters disabled
    digitalWrite(PIN_BRAKE_RELAY,  HIGH);  // Brakes applied (see hal_setBrake)
    analogWrite(PIN_MOTOR_LEFT,    floatToPWM(0.0f));   // Zero speed
    analogWrite(PIN_MOTOR_RIGHT,   floatToPWM(0.0f));   // Zero speed
    digitalWrite(PIN_LED_IDLE,     LOW);
    digitalWrite(PIN_LED_MANUAL,   LOW);
    digitalWrite(PIN_LED_FAULT,    LOW);
}

InputSnapshot hal_readInputs() {
    InputSnapshot in;

    // ── Joystick analog reads ─────────────────────────────────────────────────
    // analogRead() blocks for ~100 µs while the ADC converts.
    // For a 10 ms loop rate this is negligible. If loop timing becomes tight,
    // consider free-running ADC mode (ATmega328P datasheet §23.5).
    in.throttleNorm = normalizeJoystick(analogRead(PIN_JOY_THROTTLE));
    in.steeringNorm = normalizeJoystick(analogRead(PIN_JOY_STEERING));

    // ── Digital safety inputs (active-LOW inversion) ──────────────────────────
    // digitalRead() returns LOW (0) when the switch is closed (active).
    // We invert: (== LOW) → true so that the rest of the firmware uses
    // positive logic (true = "this condition is active").
    in.estopPressed     = (digitalRead(PIN_ESTOP)           == LOW);
    in.operatorEnable   = (digitalRead(PIN_OPERATOR_ENABLE) == LOW);
    in.faultAcknowledge = (digitalRead(PIN_FAULT_ACK)       == LOW);

    // ── Signal-alive flag ─────────────────────────────────────────────────────
    // When inputs come from a local joystick wired directly to the Arduino,
    // they are always available — there is no "stream" that can time out.
    // If the architecture later adds an RC receiver or serial link from the
    // Jetson, set this flag false when that link stops updating. The
    // SafetyManager in safety.cpp will then trigger a watchdog fault.
    in.inputSignalAlive = true;

    return in;
}

void hal_setMotorEnable(bool enable) {
    // HIGH asserts the enable line to the inverters.
    // The inverters may still be commanding zero speed at this point — enabling
    // here only unlocks the inverter's internal power stage.
    digitalWrite(PIN_MOTOR_ENABLE, enable ? HIGH : LOW);
}

void hal_setBrake(bool apply) {
    // WIRING ASSUMPTION:
    //   PIN_BRAKE_RELAY HIGH = relay coil energized = brakes APPLIED (clamped)
    //   PIN_BRAKE_RELAY LOW  = relay coil off       = brakes RELEASED
    //
    // This is the "power-to-apply" wiring. If your relay is wired in the
    // opposite sense ("power-to-release"), swap HIGH and LOW here.
    // Do NOT modify main.cpp — the HAL hides this detail intentionally.
    //
    // FAIL-SAFE BEHAVIOR:
    //   If the Arduino resets or loses power mid-operation, the output pin
    //   defaults to LOW, the relay releases, and brakes spring-apply.
    //   This is the desired fail-safe: loss of control = machine stops.
    digitalWrite(PIN_BRAKE_RELAY, apply ? HIGH : LOW);
}

void hal_setMotorOutputs(float leftNorm, float rightNorm) {
    // ─────────────────────────────────────────────────────────────────────────
    // PLACEHOLDER IMPLEMENTATION — Read before running on real hardware!
    // ─────────────────────────────────────────────────────────────────────────
    //
    // analogWrite() produces a 490 Hz PWM signal on pins 9 and 10 (Timer 1).
    // This is ONLY for bench-testing — the TM4 inverters on this mower do NOT
    // accept raw PWM as a speed reference. They expect RS232 serial commands.
    //
    // WHEN THE TM4 PROTOCOL IS KNOWN:
    //   1. Remove the analogWrite() lines below.
    //   2. Format and transmit a serial packet over Serial1 (or SoftwareSerial).
    //   3. The function signature (leftNorm, rightNorm) stays the same.
    //   4. No other file in the firmware needs to change.
    //
    // EXAMPLE FUTURE BODY (pseudocode, adapt to actual TM4 docs):
    //   tm4_sendSpeedPacket(TM4_AXIS_LEFT,  leftNorm);
    //   tm4_sendSpeedPacket(TM4_AXIS_RIGHT, rightNorm);
    // ─────────────────────────────────────────────────────────────────────────
    analogWrite(PIN_MOTOR_LEFT,  floatToPWM(leftNorm));
    analogWrite(PIN_MOTOR_RIGHT, floatToPWM(rightNorm));
}

void hal_setStatusLED(SystemState state) {
    // Compute desired on/off state for each LED based on the system state.
    // For FAULT we blink at ~2 Hz using millis() — no blocking delay.
    //
    // How the blink works:
    //   millis() / 250  → increments every 250 ms  (0, 1, 2, 3, 4, ...)
    //   % 2             → alternates 0, 1, 0, 1, ...
    //   == 0            → true for the first 250 ms of each 500 ms cycle

    bool idleOn   = false;
    bool manualOn = false;
    bool faultOn  = false;

    switch (state) {
        case SystemState::SAFE_IDLE:
            idleOn = true;
            break;

        case SystemState::MANUAL:
            manualOn = true;
            break;

        case SystemState::FAULT:
            faultOn = ((millis() / 250UL) % 2UL) == 0;
            break;

        default:
            // Unknown / unhandled state — flash ALL LEDs fast (100 ms) as a
            // visible "something is wrong with the state machine" indicator.
            {
                bool flash = ((millis() / 100UL) % 2UL) == 0;
                idleOn = manualOn = faultOn = flash;
            }
            break;
    }

    digitalWrite(PIN_LED_IDLE,   idleOn   ? HIGH : LOW);
    digitalWrite(PIN_LED_MANUAL, manualOn ? HIGH : LOW);
    digitalWrite(PIN_LED_FAULT,  faultOn  ? HIGH : LOW);
}
