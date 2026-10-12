// Bench-only GIGA replacement for V2_CAN_Test.py + V2_Motor_Control.py
// (env giga_r1_m7_can_bench). CAN only: ignores ignition, deck switch,
// and E-stop. The physical E-stop must still remove inverter power.
//
// Serial monitor (115200), one command per line:
//   s        start RPDO1-4 streaming, motors OFF
//   p        toggle pot-controlled drive (left handle → right wheel, max 100 rpm)
//   <rpm>    manually command rpm on all present inverters
//   x        stop motors (0 rpm hold, then PWM off; streaming continues)
//   q        stop motors and stop streaming
//   v        toggle raw RX printing
//   l        loopback self-test (disconnect from inverter first)
#include <Arduino.h>
#include <Arduino_CAN.h>

#include "tm4_can.h"

// Internal loopback test: sets FDCAN into loopback mode via STM32 registers.
// In this mode the peripheral ACKs its own frames; no external bus needed.
// Works regardless of CANH/CANL wiring state.
namespace {
void setFdcanLoopback(bool enable) {
#if defined(FDCAN2)
  FDCAN_GlobalTypeDef* const fd = FDCAN2;
#elif defined(FDCAN1)
  FDCAN_GlobalTypeDef* const fd = FDCAN1;
#else
  (void)enable; return;
#endif
  // Enter init mode so config registers are writable
  fd->CCCR |= 0x01u;                          // INIT
  const uint32_t t0 = millis();
  while (!(fd->CCCR & 0x01u) && millis()-t0 < 10) {}
  fd->CCCR |= 0x02u;                          // CCE
  if (enable) {
    fd->CCCR |= 0x80u;                        // TEST
    fd->TEST |= 0x10u;                        // LBCK
  } else {
    fd->TEST &= ~0x10u;
    fd->CCCR &= ~0x80u;
  }
  fd->CCCR &= ~0x01u;                         // exit init
  const uint32_t t1 = millis();
  while ((fd->CCCR & 0x01u) && millis()-t1 < 10) {}
}

bool runLoopbackTest() {
  setFdcanLoopback(true);
  delay(5);
  // Drain any stale RX
  while (CAN.available()) CAN.read();
  // Send a known frame
  const uint8_t payload[8] = {0xDE,0xAD,0xBE,0xEF,0x01,0x02,0x03,0x04};
  CAN.write(CanMsg(CanStandardId(0x123), 8, payload));
  const uint32_t t = millis();
  bool passed = false;
  while (millis() - t < 50) {
    while (CAN.available()) {
      const CanMsg rx = CAN.read();
      const uint32_t id = rx.isExtendedId() ? rx.getExtendedId() : rx.getStandardId();
      if (id == 0x123) { passed = true; break; }
    }
    if (passed) break;
  }
  setFdcanLoopback(false);
  delay(5);
  return passed;
}
Tm4Can tm4Bus;
bool verbose    = false;
bool potControl = false;
int16_t lastPotRpm = 0;
uint32_t lastReportMs = 0;
uint32_t lastRxCount  = 0;
char line[16];
uint8_t lineLen = 0;

// Calibrated from measured left-handle travel (10-bit ADC, bench env).
// Forward = lower ADC (handle pushed forward → wiper moves toward GND).
static constexpr int POT_NEUTRAL  = 65;
static constexpr int POT_FWD_MAX  = 37;   // full-forward limit
static constexpr int POT_REV_MAX  = 97;   // full-reverse limit
static constexpr int POT_DEADBAND = 8;
static constexpr int16_t POT_MAX_RPM = 100;  // safe first-test ceiling

int16_t potToRpm(int raw) {
  if (raw < POT_NEUTRAL - POT_DEADBAND) {
    const int d = POT_NEUTRAL - raw;
    const int r = POT_NEUTRAL - POT_FWD_MAX;   // 28 counts = full forward
    return (int16_t)min((int)POT_MAX_RPM, (int)POT_MAX_RPM * d / max(1, r));
  }
  if (raw > POT_NEUTRAL + POT_DEADBAND) {
    const int d = raw - POT_NEUTRAL;
    const int r = POT_REV_MAX - POT_NEUTRAL;    // 32 counts = full reverse
    return -(int16_t)min((int)POT_MAX_RPM, (int)POT_MAX_RPM * d / max(1, r));
  }
  return 0;
}

void printRx(uint32_t id, const uint8_t* data, uint8_t len) {
  if (!verbose) return;
  Serial.print("RX: 0x");
  Serial.print(id, HEX);
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(data[i] < 0x10 ? " 0" : " ");
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

void requestAll(bool run, int16_t rpm) {
  for (uint8_t k = 0; k < tm4Bus.count(); k++) tm4Bus.inverter(k).request(run, rpm);
}

void handleCommand(const char* cmd, uint32_t now) {
  if (strcmp(cmd, "s") == 0) {
    tm4Bus.setPowerRequest(true);
    tm4Bus.setTransmit(true, now);
    Serial.println(tm4Bus.transmitting() ? "RPDO1-RPDO4 streaming every 10 ms, motors OFF."
                                        : "CAN not started.");
  } else if (strcmp(cmd, "p") == 0) {
    if (!tm4Bus.transmitting()) {
      Serial.println("Start streaming with 's' first.");
    } else {
      potControl = !potControl;
      if (potControl) {
        Serial.print("Pot control ON — left handle drives right wheel, max ");
        Serial.print(POT_MAX_RPM);
        Serial.println(" rpm. 'p' or 'q' to stop.");
      } else {
        requestAll(false, 0);
        lastPotRpm = 0;
        Serial.println("Pot control OFF.");
      }
    }
  } else if (strcmp(cmd, "x") == 0) {
    potControl = false;
    lastPotRpm = 0;
    requestAll(false, 0);
    Serial.println("Motor stop requested.");
  } else if (strcmp(cmd, "q") == 0) {
    potControl = false;
    lastPotRpm = 0;
    tm4Bus.setTransmit(false, now);
    Serial.println("Streaming stopped; inverters will time out their RPDOs.");
  } else if (strcmp(cmd, "v") == 0) {
    verbose = !verbose;
  } else if (strcmp(cmd, "l") == 0) {
    potControl = false;
    tm4Bus.setTransmit(false, now);
    Serial.println("Loopback test (disconnect from inverter first)...");
    const bool ok = runLoopbackTest();
    Serial.println(ok ? "Loopback PASS — FDCAN + transceiver OK."
                      : "Loopback FAIL — check SN65HVD230 wiring/power.");
    Serial.println("Type 's' to resume normal streaming.");
  } else {
    char* end = nullptr;
    const long rpm = strtol(cmd, &end, 10);
    if (end == cmd || *end != '\0' || rpm < -10000 || rpm > 10000) {
      Serial.println("Commands: s, p, <rpm>, x, q, v");
    } else if (!tm4Bus.transmitting()) {
      Serial.println("Start streaming with 's' first.");
    } else {
      potControl = false;
      requestAll(true, (int16_t)rpm);
      Serial.print("Motor request: ");
      Serial.print(rpm);
      Serial.println(" rpm");
    }
  }
}

void report(uint32_t now) {
  if (tm4Bus.rxCount() == lastRxCount) Serial.println("No CAN message received.");
  lastRxCount = tm4Bus.rxCount();
  for (uint8_t k = 0; k < tm4Bus.count(); k++) {
    Tm4Inverter& inv = tm4Bus.inverter(k);
    if (!inv.config().present) continue;
    const tm4::Status& st = inv.status();
    Serial.print(inv.config().name);
    Serial.print(": healthy=");
    Serial.print(inv.healthy(now));
    Serial.print(" phase=");
    Serial.print((int)inv.phase());
    if (inv.pwmTimedOut()) Serial.print(" PWM-CONFIRM-TIMEOUT");
    Serial.print(" mains=");
    Serial.print(st.mainsState);
    Serial.print(" fault=");
    Serial.print(st.faultCode);
    Serial.print(" level=");
    Serial.print(st.faultLevel);
    Serial.print(" pwm=");
    Serial.print(st.pwmOutput);
    Serial.print(" mstate=");
    Serial.print(st.motorState);
    Serial.print(" rpm=");
    Serial.print(st.actualRpm);
    Serial.print(" dc=");
    Serial.print(st.dcBusVoltage, 1);
    Serial.print(" tx_err=");
    Serial.println(tm4Bus.txErrors());
  }

  // Handle position display
  const int raw = analogRead(A0);
  const char* dir;
  int pct = 0;
  if (raw < POT_NEUTRAL - POT_DEADBAND) {
    dir = "FORWARD";
    pct = min(100, (POT_NEUTRAL - raw) * 100 / max(1, POT_NEUTRAL - POT_FWD_MAX));
  } else if (raw > POT_NEUTRAL + POT_DEADBAND) {
    dir = "REVERSE";
    pct = min(100, (raw - POT_NEUTRAL) * 100 / max(1, POT_REV_MAX - POT_NEUTRAL));
  } else {
    dir = "NEUTRAL";
  }
  Serial.print("handle=");
  Serial.print(raw);
  Serial.print(" (");
  Serial.print(dir);
  if (pct > 0) { Serial.print(" "); Serial.print(pct); Serial.print("%)"); }
  else Serial.print(")");
  if (potControl) {
    Serial.print("  cmd_rpm=");
    Serial.print(lastPotRpm);
  }
  Serial.println();
}
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("Connecting to CAN...");
  const bool started = tm4Bus.begin(millis());
  tm4Bus.setTransmit(false, millis());
  tm4Bus.onReceive(printRx);
  Serial.println(started ? "Connected: FDCAN2 @ 250000 bit/s" : "CAN not started.");
  Serial.println("Commands: s (start), p (pot drive), <rpm>, x (stop), q (stop all), v (raw RX)");
}

void loop() {
  const uint32_t now = millis();
  tm4Bus.update(now);

  // Pot-controlled drive: update motor command whenever handle position changes
  if (potControl && tm4Bus.transmitting()) {
    const int16_t rpm = potToRpm(analogRead(A0));
    if (rpm != lastPotRpm) {
      requestAll(rpm != 0, rpm);
      lastPotRpm = rpm;
    }
  }

  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      line[lineLen] = '\0';
      if (lineLen) handleCommand(line, now);
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }

  if (now - lastReportMs >= 1000) {
    lastReportMs = now;
    report(now);
  }
}
