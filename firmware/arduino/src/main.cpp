/*
 * main.cpp — DANA Mower firmware entry point (Arduino)
 *
 * This file owns the application loop only — no raw hardware calls live here.
 * All hardware access goes through hal_arduino.h / hal_arduino.cpp.
 *
 * Architecture:
 *   hal_readInputs()        → InputSnapshot  (hardware → data)
 *   SafetyManager::update() → SafetyStatus   (data → safety verdict)
 *   StateMachine            → SystemState     (safety verdict → state)
 *   ManualMixer::mix()      → WheelCommand    (state + inputs → wheel targets)
 *   SlewRateLimiter         → WheelCommand    (targets → rate-limited outputs)
 *   hal_setMotorOutputs()                     (outputs → hardware)
 *
 * To port to STM32: change #include "hal_arduino.h" → #include "hal_stm32.h".
 * All other code compiles unchanged.
 */

#include <Arduino.h>

#include "types.h"
#include "state_machine.h"
#include "safety.h"
#include "manual_control.h"
#include "slew_rate_limiter.h"

#include "hal_arduino.h"   // <── swap to hal_stm32.h when migrating

// ─────────────────────────────────────────────────────────────────────────────
// Application-layer objects
// ─────────────────────────────────────────────────────────────────────────────
static StateMachine    sm;
static SafetyManager   safety;
static ManualMixer     mixer;

// Slew limiters cap acceleration to 2 units/sec (0→full in ~500 ms).
// Increase the value to allow faster acceleration; decrease to slow it down.
static SlewRateLimiter leftLimiter(2.0f);
static SlewRateLimiter rightLimiter(2.0f);

static uint32_t lastMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);

    hal_init();        // Configure all pins, assert safe defaults
    safety.begin();    // Initialize SafetyManager internal state
    sm.setState(SystemState::SAFE_IDLE);

    Serial.println("DANA Mower — manual control firmware");
    Serial.println("State: SAFE_IDLE. Enable operator switch to drive.");
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    const uint32_t now = millis();
    const float dt = (lastMs == 0) ? 0.0f
                                   : (float)(now - lastMs) / 1000.0f;
    lastMs = now;

    // ── 1. Read all physical sensors through the HAL ──────────────────────────
    InputSnapshot in = hal_readInputs();

    // ── 2. Run safety logic ───────────────────────────────────────────────────
    SafetyStatus ss = safety.update(in, now);

    if (ss.mustFault) {
        sm.setState(SystemState::FAULT);
    }

    // ── 3. State machine ──────────────────────────────────────────────────────
    WheelCommand wheels{};   // zero-initialized

    switch (sm.state()) {

        // ── SAFE_IDLE: brakes on, motors off, waiting for operator enable ─────
        case SystemState::SAFE_IDLE:
            hal_setBrake(true);
            hal_setMotorEnable(false);
            hal_setMotorOutputs(0.0f, 0.0f);

            if (ss.canEnableDrive) {
                leftLimiter.reset(0.0f);
                rightLimiter.reset(0.0f);
                sm.setState(SystemState::MANUAL);
            }
            break;

        // ── MANUAL: operator-controlled differential drive ────────────────────
        case SystemState::MANUAL: {
            if (!ss.canEnableDrive) {
                sm.setState(SystemState::SAFE_IDLE);
                break;
            }

            hal_setBrake(false);
            hal_setMotorEnable(true);

            DriveCommand cmd;
            cmd.throttle    = in.throttleNorm;
            cmd.steering    = in.steeringNorm;
            cmd.enableDrive = true;

            wheels = mixer.mix(cmd);
            wheels.left  = leftLimiter.step(wheels.left,  dt);
            wheels.right = rightLimiter.step(wheels.right, dt);

            hal_setMotorOutputs(wheels.left, wheels.right);
        } break;

        // ── FAULT: brakes on, motors off, wait for operator acknowledge ───────
        case SystemState::FAULT:
            hal_setBrake(true);
            hal_setMotorEnable(false);
            hal_setMotorOutputs(0.0f, 0.0f);

            // Clear fault only when:
            //   (a) safety conditions are clear (estop released, inputs alive)
            //   (b) operator explicitly presses the fault-acknowledge button
            if (ss.faultCleared && in.faultAcknowledge) {
                sm.setState(SystemState::SAFE_IDLE);
            }
            break;

        // ── Catch-all: unknown state → safe idle ──────────────────────────────
        default:
            sm.setState(SystemState::SAFE_IDLE);
            break;
    }

    // ── 4. Update status LEDs ─────────────────────────────────────────────────
    hal_setStatusLED(sm.state());

    // ── 5. Debug print (250 ms interval, non-blocking) ───────────────────────
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 250) {
        lastPrint = now;

        const char* stateNames[] = {"IDLE", "MANUAL", "AUTO", "DOCKING", "FAULT"};
        uint8_t si = (uint8_t)sm.state();

        Serial.print("State="); Serial.print(stateNames[si < 5 ? si : 0]);
        Serial.print(" estop=");  Serial.print(in.estopPressed);
        Serial.print(" enable="); Serial.print(in.operatorEnable);
        Serial.print(" thr=");    Serial.print(in.throttleNorm,  2);
        Serial.print(" str=");    Serial.print(in.steeringNorm,  2);
        Serial.print(" L=");      Serial.print(wheels.left,      2);
        Serial.print(" R=");      Serial.println(wheels.right,   2);
    }

    delay(10);   // 100 Hz target loop rate
}
