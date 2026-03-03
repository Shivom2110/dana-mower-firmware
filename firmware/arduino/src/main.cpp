#include <Arduino.h>

#include "types.h"
#include "state_machine.h"
#include "safety.h"
#include "manual_control.h"
#include "slew_rate_limiter.h"

// =======================
// Placeholder "HAL" layer
// Replace these with real inverter + brake control later.
// =======================
static void motorsEnable(bool en) {
  // TODO: set inverter enable pin, etc.
  (void)en;
}

static void brakesApply(bool apply) {
  // TODO: control 24V brake relay (apply=true -> brakes ON)
  (void)apply;
}

static void setMotorOutputs(float leftNorm, float rightNorm) {
  // TODO: convert -1..+1 to PWM/CAN/UART for inverters
  (void)leftNorm;
  (void)rightNorm;
}

// =======================
// Inputs (placeholder)
// =======================
static InputSnapshot readInputs() {
  InputSnapshot in;

  // For now, fake inputs so the code is understandable and testable:
  // - Press 'e' on serial monitor to toggle estop
  // - Press 'm' to toggle operator enable
  // - Use 'w/s' throttle up/down, 'a/d' steering left/right
  static bool estop = false;
  static bool enable = false;
  static float throttle = 0.0f;
  static float steer = 0.0f;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'e') estop = !estop;
    if (c == 'm') enable = !enable;
    if (c == 'w') throttle = clamp(throttle + 0.1f, -1.0f, 1.0f);
    if (c == 's') throttle = clamp(throttle - 0.1f, -1.0f, 1.0f);
    if (c == 'd') steer    = clamp(steer + 0.1f, -1.0f, 1.0f);
    if (c == 'a') steer    = clamp(steer - 0.1f, -1.0f, 1.0f);
    if (c == 'c') { throttle = 0.0f; steer = 0.0f; } // center
    if (c == 'r') in.faultAcknowledge = true;        // reset/ack
  }

  in.estopPressed = estop;
  in.operatorEnable = enable;
  in.throttleNorm = throttle;
  in.steeringNorm = steer;

  // In real life, you’d set this false if RC/serial/etc. stops updating
  in.inputSignalAlive = true;

  return in;
}

// =======================
// Control objects
// =======================
StateMachine sm;
SafetyManager safety;
ManualMixer mixer;
SlewRateLimiter leftLimiter(2.0f);   // 2 units/sec
SlewRateLimiter rightLimiter(2.0f);  // 2 units/sec

uint32_t lastMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  safety.begin();
  sm.setState(SystemState::SAFE_IDLE);

  Serial.println("DANA Mower Manual Control (placeholder)");
  Serial.println("Keys: e=toggle estop, m=toggle enable, w/s throttle, a/d steer, c=center, r=ack fault");
}

void loop() {
  const uint32_t now = millis();
  const float dt = (lastMs == 0) ? 0.0f : (now - lastMs) / 1000.0f;
  lastMs = now;

  InputSnapshot in = readInputs();
  SafetyStatus ss = safety.update(in, now);

  // Force FAULT if safety says so
  if (ss.mustFault) {
    sm.setState(SystemState::FAULT);
  }

  WheelCommand wheels{0, 0};

  switch (sm.state()) {
    case SystemState::SAFE_IDLE:
      brakesApply(true);
      motorsEnable(false);
      setMotorOutputs(0, 0);

      if (ss.canEnableDrive) {
        sm.setState(SystemState::MANUAL);
        leftLimiter.reset(0);
        rightLimiter.reset(0);
      }
      break;

    case SystemState::MANUAL: {
      if (!ss.canEnableDrive) {
        sm.setState(SystemState::SAFE_IDLE);
        break;
      }

      brakesApply(false);
      motorsEnable(true);

      DriveCommand cmd;
      cmd.throttle = in.throttleNorm;
      cmd.steering = in.steeringNorm;
      cmd.enableDrive = true;

      wheels = mixer.mix(cmd);
      wheels.left  = leftLimiter.step(wheels.left, dt);
      wheels.right = rightLimiter.step(wheels.right, dt);

      setMotorOutputs(wheels.left, wheels.right);
    } break;

    case SystemState::FAULT:
      motorsEnable(false);
      brakesApply(true);
      setMotorOutputs(0, 0);

      // Clear latched fault only with operator ack AND safety cleared
      if (ss.faultCleared && in.faultAcknowledge) {
        // NOTE: In a real implementation, you would also clear the latched fault inside SafetyManager.
        // For this starter code, we just go back to SAFE_IDLE.
        sm.setState(SystemState::SAFE_IDLE);
      }
      break;

    default:
      sm.setState(SystemState::SAFE_IDLE);
      break;
  }

  // Debug print occasionally
  static uint32_t lastPrint = 0;
  if (now - lastPrint > 250) {
    lastPrint = now;
    Serial.print("State=");
    Serial.print((int)sm.state());
    Serial.print(" estop=");
    Serial.print(in.estopPressed);
    Serial.print(" enable=");
    Serial.print(in.operatorEnable);
    Serial.print(" thr=");
    Serial.print(in.throttleNorm, 2);
    Serial.print(" str=");
    Serial.print(in.steeringNorm, 2);
    Serial.print(" L=");
    Serial.print(wheels.left, 2);
    Serial.print(" R=");
    Serial.println(wheels.right, 2);
  }

  delay(10);
}
