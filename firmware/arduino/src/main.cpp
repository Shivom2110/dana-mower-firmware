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
static constexpr uint32_t CONTROL_PERIOD_MS = 10;
static constexpr uint32_t STATUS_PERIOD_MS = 1000;
static uint32_t lastMs = 0;
static uint32_t lastStatusMs = 0;

void setup() {
  Serial.begin(115200);
  hal_init();
  safety.begin();
  sm.setState(SystemState::SAFE_IDLE);
  Serial.println("DANA mower GIGA CAN controller: drive engages with ignition on, E-stop released, healthy CAN, neutral controls, deck switch off.");
}

void loop() {
  const uint32_t now = millis();
  // CAN is serviced every pass (not every control tick) so the 1 ms RPDO
  // stagger and 10 ms period match the proven laptop test.
  hal_service(now);
  if (lastMs != 0 && now - lastMs < CONTROL_PERIOD_MS) return;

  const float dt = lastMs == 0 ? 0.0f : (float)(now - lastMs) / 1000.0f;
  lastMs = now;

  const InputSnapshot in = hal_readInputs();
  const SafetyStatus ss = safety.update(in, now);
  if (ss.mustFault) sm.setState(SystemState::FAULT);

  WheelCommand wheels{};
  switch (sm.state()) {
    case SystemState::SAFE_IDLE:
      hal_setDrive(false, 0.0f, 0.0f, false);
      hal_setBrake(true);
      if (ss.canEnableDrive) {
        leftLimiter.reset();
        rightLimiter.reset();
        sm.setState(SystemState::MANUAL);
      }
      break;

    case SystemState::MANUAL:
      if (!ss.driveAllowed) {
        hal_setDrive(false, 0.0f, 0.0f, false);
        sm.setState(SystemState::SAFE_IDLE);
        break;
      }
      hal_setBrake(false);
      wheels = mixer.mix({in.throttleNorm, in.steeringNorm, true});
      wheels.left = leftLimiter.step(wheels.left, dt);
      wheels.right = rightLimiter.step(wheels.right, dt);
      hal_setDrive(true, wheels.left, wheels.right, in.deckSwitchOn);
      break;

    case SystemState::FAULT:
      hal_forceMotorsOff();
      hal_setBrake(true);
      if (ss.faultCleared) sm.setState(SystemState::SAFE_IDLE);
      break;

    default:
      sm.setState(SystemState::SAFE_IDLE);
      break;
  }

  hal_setStatusLED(sm.state());
  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    hal_printStatus(sm.state());
  }
}
