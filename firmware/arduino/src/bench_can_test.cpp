// Bench-only GIGA replacement for V2_CAN_Test.py + V2_Motor_Control.py
// (env giga_r1_m7_can_bench). CAN only: ignores handles, ignition, deck
// switch, and the E-stop input, like the laptop scripts. The physical E-stop
// must still remove inverter power.
//
// Serial monitor (115200), one command per line:
//   s        start RPDO1-4 streaming (V2_CAN_Test.py), motors OFF
//   <rpm>    enable motor 1 on every present inverter and command rpm
//            (PWM request -> TPDO4 confirmation -> reference enable)
//   x        stop motors: 0 rpm for 500 ms, then PWM off (streaming continues)
//   q        stop motors immediately and stop streaming
//   v        toggle raw RX printing
#include <Arduino.h>

#include "tm4_can.h"

namespace {
Tm4Can tm4;
bool verbose = false;
uint32_t lastReportMs = 0;
uint32_t lastRxCount = 0;
char line[16];
uint8_t lineLen = 0;

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
  for (uint8_t k = 0; k < tm4.count(); k++) tm4.inverter(k).request(run, rpm);
}

void handleCommand(const char* cmd, uint32_t now) {
  if (strcmp(cmd, "s") == 0) {
    tm4.setPowerRequest(true);
    tm4.setTransmit(true, now);
    Serial.println(tm4.transmitting() ? "RPDO1-RPDO4 streaming every 10 ms, motors OFF."
                                      : "CAN not started.");
  } else if (strcmp(cmd, "x") == 0) {
    requestAll(false, 0);
    Serial.println("Motor stop requested (0 rpm hold, then PWM off).");
  } else if (strcmp(cmd, "q") == 0) {
    tm4.setTransmit(false, now);
    Serial.println("Streaming stopped; inverters will time out their RPDOs.");
  } else if (strcmp(cmd, "v") == 0) {
    verbose = !verbose;
  } else {
    char* end = nullptr;
    const long rpm = strtol(cmd, &end, 10);
    if (end == cmd || *end != '\0' || rpm < -10000 || rpm > 10000) {
      Serial.println("Commands: s, <rpm>, x, q, v");
    } else if (!tm4.transmitting()) {
      Serial.println("Start streaming with 's' first.");
    } else {
      requestAll(true, (int16_t)rpm);
      Serial.print("Motor request: ");
      Serial.print(rpm);
      Serial.println(" rpm");
    }
  }
}

void report(uint32_t now) {
  if (tm4.rxCount() == lastRxCount) Serial.println("No CAN message received.");
  lastRxCount = tm4.rxCount();
  for (uint8_t k = 0; k < tm4.count(); k++) {
    Tm4Inverter& inv = tm4.inverter(k);
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
    Serial.print(" rpm=");
    Serial.print(st.actualRpm);
    Serial.print(" dc=");
    Serial.print(st.dcBusVoltage, 1);
    Serial.print(" tx_err=");
    Serial.println(tm4.txErrors());
  }
}
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("Connecting to CAN...");
  const bool started = tm4.begin(millis());
  // begin() streams by default (controller behaviour); the bench waits for 's'.
  tm4.setTransmit(false, millis());
  tm4.onReceive(printRx);
  Serial.println(started ? "Connected: FDCAN2 @ 250000 bit/s" : "CAN not started.");
  Serial.println("Commands: s (start RPDOs), <rpm>, x (stop motor), q (stop all), v (raw RX)");
}

void loop() {
  const uint32_t now = millis();
  tm4.update(now);

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
