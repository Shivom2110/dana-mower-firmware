#include <Arduino.h>

#include "hal_arduino.h"
#include "pins.h"
#include "tm4_can.h"

namespace {
Tm4Can tm4Bus;

float normalizeJoystick(int raw) {
  const int centered = raw - JOY_ADC_CENTER;
  if (centered > -JOY_ADC_DEADBAND && centered < JOY_ADC_DEADBAND) return 0.0f;
  return clamp((float)centered / (float)JOY_ADC_MAX, -1.0f, 1.0f);
}

bool readConditionedInput(uint8_t pin, bool activeHigh) {
  const bool high = digitalRead(pin) == HIGH;
  return activeHigh ? high : !high;
}

int16_t toRpm(float norm, int16_t maxRpm, int8_t direction) {
  return (int16_t)lroundf(clamp(norm, -1.0f, 1.0f) * maxRpm * direction);
}

const char* phaseName(Tm4Inverter::Phase p) {
  switch (p) {
    case Tm4Inverter::Phase::OFF: return "OFF";
    case Tm4Inverter::Phase::PWM_REQUESTED: return "PWM_REQ";
    case Tm4Inverter::Phase::RUNNING: return "RUN";
    case Tm4Inverter::Phase::STOPPING: return "STOPPING";
  }
  return "?";
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
  pinMode(PIN_DECK_SWITCH, INPUT);

  // RPDOs stream from boot with every motor OFF (RPDO2 all zero), like the
  // laptop scripts; the inverters fault if RPDOs stop for 100 ms.
  if (!tm4Bus.begin(millis())) {
    Serial.println("CAN bus failed to start at 250 kbit/s.");
  }
}

void hal_service(uint32_t nowMs) {
  tm4Bus.update(nowMs);
}

InputSnapshot hal_readInputs() {
  InputSnapshot in;
  in.throttleNorm = normalizeJoystick(analogRead(PIN_JOY_THROTTLE));
  in.steeringNorm = normalizeJoystick(analogRead(PIN_JOY_STEERING));
  in.ignitionOn = readConditionedInput(PIN_IGNITION, IGNITION_ACTIVE_HIGH);
  in.estopPressed = readConditionedInput(PIN_ESTOP, ESTOP_ACTIVE_LOW);
  in.deckSwitchOn = readConditionedInput(PIN_DECK_SWITCH, DECK_SWITCH_ACTIVE_HIGH);
  in.canNetworkHealthy = tm4Bus.healthy();

  // Power Ready (contactor closed) only with ignition on and E-stop released;
  // otherwise request a return to Startup.
  tm4Bus.setPowerRequest(in.ignitionOn && !in.estopPressed);
  return in;
}

void hal_setDrive(bool enable, float leftNorm, float rightNorm, bool deckOn) {
  // Enable is a CAN protocol action; never infer a raw GPIO safety output.
  for (uint8_t k = 0; k < tm4Bus.count(); k++) {
    Tm4Inverter& inv = tm4Bus.inverter(k);
    const InverterConfig& cfg = inv.config();
    switch (cfg.role) {
      case InverterRole::DECK:
        inv.request(enable && deckOn, toRpm(1.0f, TM4_DECK_RPM, cfg.direction));
        break;
      case InverterRole::DRIVE_LEFT:
        inv.request(enable, toRpm(leftNorm, TM4_DRIVE_MAX_RPM, cfg.direction));
        break;
      case InverterRole::DRIVE_RIGHT:
        inv.request(enable, toRpm(rightNorm, TM4_DRIVE_MAX_RPM, cfg.direction));
        break;
    }
  }
}

void hal_forceMotorsOff() {
  for (uint8_t k = 0; k < tm4Bus.count(); k++) tm4Bus.inverter(k).forceOff();
}

void hal_setBrake(bool apply) {
  (void)apply;
  // The motor safe brake is driven by each inverter (Driver Out 2, "Safe Brake
  // Mot1" in SmartView), not by the GIGA.
}

void hal_setStatusLED(SystemState state) {
  (void)state;
  // Status LED wiring is not present in the diagram.
}

void hal_printStatus(SystemState state) {
  static const char* const names[] = {"SAFE_IDLE", "MANUAL", "AUTO", "DOCKING", "FAULT"};
  const uint8_t s = (uint8_t)state;
  Serial.print("state=");
  Serial.print(s < 5 ? names[s] : "?");
  Serial.print(" can_healthy=");
  Serial.print(tm4Bus.healthy());
  Serial.print(" rx=");
  Serial.print(tm4Bus.rxCount());
  Serial.print(" tx=");
  Serial.print(tm4Bus.txCount());
  Serial.print(" tx_err=");
  Serial.println(tm4Bus.txErrors());

  const uint32_t now = millis();
  for (uint8_t k = 0; k < tm4Bus.count(); k++) {
    Tm4Inverter& inv = tm4Bus.inverter(k);
    if (!inv.config().present) continue;
    const tm4::Status& st = inv.status();
    Serial.print("  ");
    Serial.print(inv.config().name);
    Serial.print(": healthy=");
    Serial.print(inv.healthy(now));
    Serial.print(" phase=");
    Serial.print(phaseName(inv.phase()));
    if (inv.pwmTimedOut()) Serial.print("(PWM confirm timeout)");
    Serial.print(" mains=");
    Serial.print(st.mainsState);
    Serial.print(" fault=");
    Serial.print(st.faultCode);
    Serial.print(" level=");
    Serial.print(st.faultLevel);
    Serial.print(" pwm=");
    Serial.print(st.pwmOutput);
    Serial.print(" rpm=");
    Serial.print(st.actualRpm);
    Serial.print(" dc=");
    Serial.println(st.dcBusVoltage, 1);
  }
}
