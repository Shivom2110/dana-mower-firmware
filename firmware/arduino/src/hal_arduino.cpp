#include <Arduino.h>

#include "hal_arduino.h"
#include "pins.h"
#include "tm4_can.h"

namespace {
Tm4Can tm4;

float normalizeJoystick(int raw) {
  const int centered = raw - JOY_ADC_CENTER;
  if (centered > -JOY_ADC_DEADBAND && centered < JOY_ADC_DEADBAND) return 0.0f;
  return clamp((float)centered / (float)JOY_ADC_MAX, -1.0f, 1.0f);
}

bool readConditionedInput(uint8_t pin, bool activeHigh) {
  const bool high = digitalRead(pin) == HIGH;
  return activeHigh ? high : !high;
}
}

void hal_init() {
  analogReadResolution(JOY_ADC_BITS);
  pinMode(PIN_JOY_THROTTLE, INPUT);
  pinMode(PIN_JOY_STEERING, INPUT);

  // Input circuits supply a defined 3.3 V level; no internal pull-up is used
  // because it could conceal a wiring or conditioning-circuit fault.
  pinMode(PIN_IGNITION, INPUT);
  pinMode(PIN_ESTOP, INPUT);

  // CAN transport starts fail-closed until its Dana protocol configuration is
  // completed. Therefore boot cannot enable torque.
  tm4.begin();
}

InputSnapshot hal_readInputs() {
  tm4.update(millis());

  InputSnapshot in;
  in.throttleNorm = normalizeJoystick(analogRead(PIN_JOY_THROTTLE));
  in.steeringNorm = normalizeJoystick(analogRead(PIN_JOY_STEERING));
  in.ignitionOn = readConditionedInput(PIN_IGNITION, IGNITION_ACTIVE_HIGH);
  in.estopPressed = readConditionedInput(PIN_ESTOP, ESTOP_ACTIVE_LOW);
  in.canNetworkHealthy = tm4.healthy();
  return in;
}

void hal_setMotorEnable(bool enable) {
  // Enable is a CAN protocol action; never infer a raw GPIO safety output.
  if (!enable) tm4.sendZeroTorque();
}

void hal_setBrake(bool apply) {
  (void)apply;
  // No brake actuator is shown in the supplied diagram. Add a protected,
  // documented output only after the vehicle braking circuit is specified.
}

void hal_setMotorOutputs(float leftNorm, float rightNorm) {
  if (!tm4.sendWheelCommands(leftNorm, rightNorm)) {
    tm4.sendZeroTorque();
  }
}

void hal_setStatusLED(SystemState state) {
  (void)state;
  // Status LED wiring is not present in the diagram.
}
