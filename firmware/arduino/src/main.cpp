#include <Arduino.h>

#include "hal_arduino.h"
#include "manual_control.h"
#include "safety.h"
#include "slew_rate_limiter.h"
#include "state_machine.h"

static StateMachine sm;
static SafetyManager safety;
static ManualMixer mixer;
static SlewRateLimiter leftLimiter(2.0f);
static SlewRateLimiter rightLimiter(2.0f);
static uint32_t lastMs = 0;

void setup() {
  Serial.begin(115200);
  hal_init();
  safety.begin();
  sm.setState(SystemState::SAFE_IDLE);
  Serial.println("DANA mower GIGA CAN controller: torque locked pending TM4 protocol configuration.");
}

void loop() {
  const uint32_t now = millis();
  const float dt = lastMs == 0 ? 0.0f : (float)(now - lastMs) / 1000.0f;
  lastMs = now;

  const InputSnapshot in = hal_readInputs();
  const SafetyStatus ss = safety.update(in, now);
  if (ss.mustFault) sm.setState(SystemState::FAULT);

  WheelCommand wheels{};
  switch (sm.state()) {
    case SystemState::SAFE_IDLE:
      hal_setMotorOutputs(0.0f, 0.0f);
      hal_setMotorEnable(false);
      hal_setBrake(true);
      if (ss.canEnableDrive) {
        leftLimiter.reset();
        rightLimiter.reset();
        sm.setState(SystemState::MANUAL);
      }
      break;

    case SystemState::MANUAL:
      if (!ss.canEnableDrive) {
        sm.setState(SystemState::SAFE_IDLE);
        break;
      }
      hal_setBrake(false);
      hal_setMotorEnable(true);
      wheels = mixer.mix({in.throttleNorm, in.steeringNorm, true});
      wheels.left = leftLimiter.step(wheels.left, dt);
      wheels.right = rightLimiter.step(wheels.right, dt);
      hal_setMotorOutputs(wheels.left, wheels.right);
      break;

    case SystemState::FAULT:
      hal_setMotorOutputs(0.0f, 0.0f);
      hal_setMotorEnable(false);
      hal_setBrake(true);
      if (ss.faultCleared) sm.setState(SystemState::SAFE_IDLE);
      break;

    default:
      sm.setState(SystemState::SAFE_IDLE);
      break;
  }

  hal_setStatusLED(sm.state());
  delay(10);
}
