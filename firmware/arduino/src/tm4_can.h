#pragma once

#include <stdint.h>

#include "tm4_config.h"
#include "tm4_protocol.h"

// One TAU Generic Slave AC-X1 inverter. Motor 1 speed-mode state machine:
//   OFF           → RPDO2 all zero
//   PWM_REQUESTED → speed mode + PWM enable; waits for TPDO4 pwmOutput confirmation
//   RUNNING       → reference enabled, speed = commanded rpm
//   STOPPING      → 0 rpm for TM4_STOP_HOLD_MS, then back to OFF
class Tm4Inverter {
public:
  enum class Phase : uint8_t { OFF, PWM_REQUESTED, RUNNING, STOPPING };

  void init(const InverterConfig* cfg) { _cfg = cfg; }
  void request(bool run, int16_t rpm);  // graceful: goes through STOPPING
  void forceOff();                       // immediate: PWM off on next RPDO2

  void update(uint32_t nowMs);
  bool handleRx(uint32_t id, const uint8_t* data, uint8_t len, uint32_t nowMs);
  void encodeRpdo(uint8_t index, uint8_t mainsRequest, uint8_t out[8]) const;

  bool healthy(uint32_t nowMs) const;
  const InverterConfig& config() const { return *_cfg; }
  const tm4::Status& status() const { return _status; }
  Phase phase() const { return _phase; }
  bool pwmTimedOut() const { return _pwmTimeout; }
  uint32_t nextTxMs[4] = {};

private:
  const InverterConfig* _cfg = nullptr;
  Phase _phase = Phase::OFF;
  bool _run = false;
  int16_t _rpm = 0;
  bool _pwmTimeout = false;
  uint32_t _phaseMs = 0;
  bool _rxSeen = false;
  uint32_t _lastRxMs = 0;
  tm4::Status _status;
};

// CAN transport for all configured inverters. 250 kbit/s on GIGA FDCAN2
// (CANRX/CANTX header, D93/D94). Streams RPDO1-4 to every present inverter.
class Tm4Can {
public:
  bool begin(uint32_t nowMs);
  void update(uint32_t nowMs);  // call every loop: drains RX, steps motors, sends RPDOs

  // Keep transmitting while inverters are powered — 100 ms RPDO timeout will fault them.
  void setTransmit(bool on, uint32_t nowMs);
  bool transmitting() const { return _transmit; }
  // true → RPDO1 requests Power Ready Direct (mains=8); false → Startup (mains=1).
  void setPowerRequest(bool on) { _powerRequest = on; }

  bool healthy() const { return _healthy; }
  uint8_t count() const { return TM4_INVERTER_COUNT; }
  Tm4Inverter& inverter(uint8_t i) { return _inv[i]; }

  uint32_t rxCount() const { return _rxCount; }
  uint32_t txCount() const { return _txCount; }
  uint32_t txErrors() const { return _txErrors; }

  typedef void (*RxCallback)(uint32_t id, const uint8_t* data, uint8_t len);
  void onReceive(RxCallback cb) { _rxCallback = cb; }

private:
  bool send(uint16_t id, const uint8_t data[8]);
  bool sendRaw(uint16_t id, const uint8_t* data, uint8_t len);

  Tm4Inverter _inv[TM4_INVERTER_COUNT];
  bool _busStarted = false;
  bool _transmit = false;
  bool _powerRequest = false;
  bool _healthy = false;
  uint8_t _consecutiveTxErrors = 0;
  uint32_t _rxCount = 0;
  uint32_t _txCount = 0;
  uint32_t _txErrors = 0;
  uint32_t _syncNextMs = 0;
  RxCallback _rxCallback = nullptr;
};
