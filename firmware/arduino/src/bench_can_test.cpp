// Bench test env (giga_r1_m7_can_bench). CAN only — no ignition, deck switch,
// or E-stop logic. Physical E-stop must still cut inverter power.
//
// Serial commands (115200 baud):
//   s        start RPDO1-4 streaming, motors OFF
//   p        toggle pot-controlled drive (left handle → right wheel)
//   <rpm>    manually command rpm on all present inverters
//   x        stop motors (streaming continues)
//   q        stop motors and streaming
//   v        toggle raw RX printing
//   d        toggle fast pot ADC reading (10 Hz) for calibration
//   l        loopback self-test (disconnect from inverter first)
#include <Arduino.h>
#include <Arduino_CAN.h>

#include "tm4_can.h"

// Loopback test via STM32 FDCAN registers — ACKs its own frames internally,
// bypasses the transceiver. PASS confirms FDCAN peripheral, not physical wiring.
namespace {
void setFdcanLoopback(bool enable) {
#if defined(FDCAN2)
  FDCAN_GlobalTypeDef* const fd = FDCAN2;
#elif defined(FDCAN1)
  FDCAN_GlobalTypeDef* const fd = FDCAN1;
#else
  (void)enable; return;
#endif
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
  while (CAN.available()) CAN.read();
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
bool fastPot    = false;
int16_t lastPotRpm = 0;
uint32_t lastReportMs  = 0;
uint32_t lastFastPotMs = 0;
uint32_t lastRxCount   = 0;
char line[16];
uint8_t lineLen = 0;

// Handle pot calibration (10-bit ADC on A0, bench env).
// Forward push = lower ADC; reverse push = higher ADC.
// Use 'd' command to measure actual values before changing these.
static constexpr int POT_NEUTRAL   = 378;  // ADC at handle rest/center (measured 2026-10-18: 377-379)
static constexpr int POT_FWD_RANGE = 55;   // counts from neutral to full-forward (measured 321)
static constexpr int POT_REV_RANGE = 28;   // counts from neutral to full-reverse (measured 408)
static constexpr int POT_DEADBAND  = 15;   // dead zone each side of neutral (neutral returns 369-392)
// Motor rpm (the wheel turns ~30x slower through the gearbox). Below ~200 rpm
// the inverter cannot hold speed (it hunts 0-110 rpm), so the handle starts at
// POT_MIN_RPM just past the dead zone and reaches POT_MAX_RPM at full travel.
static constexpr int16_t POT_MIN_RPM = 200;
static constexpr int16_t POT_MAX_RPM = 1000; // RPM ceiling — raise after confirming spin
// A neutral reading must last this long before the motor is stopped. Brief
// dips (pot noise near the reverse threshold) otherwise restart the 500 ms
// stop + PWM handshake over and over, clicking the brake without turning.
static constexpr uint32_t POT_STOP_DEBOUNCE_MS = 300;
uint32_t potZeroSinceMs = 0;
// The raw handle reading spikes by ~25 counts at neutral (seen down to 351
// at a 378 neutral), enough to cross the forward threshold. Sample every
// 5 ms and low-pass filter (~50 ms time constant) so spikes cannot command
// the wheel.
static constexpr uint32_t POT_SAMPLE_MS = 5;
static constexpr float POT_FILTER_ALPHA = 0.1f;
float potFiltered = -1.0f;  // < 0: not primed yet
uint32_t lastPotSampleMs = 0;

// Hysteresis: once running, the stop threshold is 3 counts wider than the
// engage threshold, so boundary noise cannot toggle the motor off.
// Distance d past neutral (beyond the dead zone) -> POT_MIN_RPM..POT_MAX_RPM.
int16_t scaleRpm(int d, int range) {
  const int span = max(1, range - POT_DEADBAND);
  const int over = constrain(d - POT_DEADBAND, 0, span);
  return (int16_t)(POT_MIN_RPM + (long)(POT_MAX_RPM - POT_MIN_RPM) * over / span);
}

int16_t potToRpm(int raw, int16_t current) {
  const int fwdThresh = (current > 0) ? (POT_NEUTRAL - POT_DEADBAND + 3)
                                       : (POT_NEUTRAL - POT_DEADBAND);
  const int revThresh = (current < 0) ? (POT_NEUTRAL + POT_DEADBAND - 3)
                                       : (POT_NEUTRAL + POT_DEADBAND);
  if (raw < fwdThresh) {
    const int d = POT_NEUTRAL - raw;
    return scaleRpm(d, POT_FWD_RANGE);
  }
  if (raw > revThresh) {
    const int d = raw - POT_NEUTRAL;
    return -scaleRpm(d, POT_REV_RANGE);
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
      potFiltered = -1.0f;
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
  } else if (strcmp(cmd, "d") == 0) {
    fastPot = !fastPot;
    if (fastPot) Serial.println("Pot debug ON — raw ADC @ 10 Hz. Move handle slowly. 'd' to stop.");
    else         Serial.println("Pot debug OFF.");
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

  // Handle position (4-sample average)
  int potSum = 0;
  for (int i = 0; i < 4; i++) potSum += analogRead(A0);
  const int raw = potSum >> 2;
  const char* dir;
  int pct = 0;
  if (raw < POT_NEUTRAL - POT_DEADBAND) {
    dir = "FORWARD";
    pct = min(100, (POT_NEUTRAL - raw) * 100 / max(1, POT_FWD_RANGE));
  } else if (raw > POT_NEUTRAL + POT_DEADBAND) {
    dir = "REVERSE";
    pct = min(100, (raw - POT_NEUTRAL) * 100 / max(1, POT_REV_RANGE));
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

  // Pot-controlled drive: 8-sample average, low-pass filter, hysteresis
  if (potControl && tm4Bus.transmitting() && now - lastPotSampleMs >= POT_SAMPLE_MS) {
    lastPotSampleMs = now;
    int potSum = 0;
    for (int i = 0; i < 8; i++) potSum += analogRead(A0);
    const float raw = potSum / 8.0f;
    potFiltered = potFiltered < 0.0f ? raw : potFiltered + POT_FILTER_ALPHA * (raw - potFiltered);
    int16_t rpm = potToRpm((int)lroundf(potFiltered), lastPotRpm);
    if (rpm == 0 && lastPotRpm != 0) {
      if (potZeroSinceMs == 0) potZeroSinceMs = now;
      if (now - potZeroSinceMs < POT_STOP_DEBOUNCE_MS) rpm = lastPotRpm;
    } else {
      potZeroSinceMs = 0;
    }
    if (rpm != lastPotRpm) {
      requestAll(rpm != 0, rpm);
      lastPotRpm = rpm;
    }
  }

  if (fastPot && now - lastFastPotMs >= 100) {
    lastFastPotMs = now;
    int sum = 0;
    for (int i = 0; i < 4; i++) sum += analogRead(A0);
    Serial.println(sum >> 2);
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
