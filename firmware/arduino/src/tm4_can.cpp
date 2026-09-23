#include "tm4_can.h"

#include <Arduino.h>
#include <Arduino_CAN.h>

namespace {
constexpr uint8_t MAX_CONSECUTIVE_TX_ERRORS = 20;
}

// ---- Tm4Inverter ----------------------------------------------------------

void Tm4Inverter::request(bool run, int16_t rpm) {
  _run = run && _cfg->present;
  _rpm = run ? rpm : 0;
}

void Tm4Inverter::forceOff() {
  _run = false;
  _rpm = 0;
  _phase = Phase::OFF;
}

void Tm4Inverter::update(uint32_t nowMs) {
  // A PWM-confirmation timeout stays latched until the request is dropped.
  if (!_run) _pwmTimeout = false;

  switch (_phase) {
    case Phase::OFF:
      if (_run && !_pwmTimeout) {
        _phase = Phase::PWM_REQUESTED;
        _phaseMs = nowMs;
      }
      break;

    case Phase::PWM_REQUESTED:
      if (!_run) {
        _phase = Phase::OFF;
      } else if (_status.pwmOutput) {
        _phase = Phase::RUNNING;
      } else if (nowMs - _phaseMs > TM4_PWM_CONFIRM_TIMEOUT_MS) {
        _pwmTimeout = true;
        _phase = Phase::OFF;
      }
      break;

    case Phase::RUNNING:
      if (!_run) {
        _phase = Phase::STOPPING;
        _phaseMs = nowMs;
      }
      break;

    case Phase::STOPPING:
      // Software delay only; not proof the rotor has stopped.
      if (nowMs - _phaseMs >= TM4_STOP_HOLD_MS) _phase = Phase::OFF;
      break;
  }
}

bool Tm4Inverter::handleRx(uint32_t id, const uint8_t* data, uint8_t len, uint32_t nowMs) {
  if (id == _cfg->tpdo1) {
    tm4::decodeTpdo1(data, len, _status);
  } else if (id == _cfg->tpdo3) {
    tm4::decodeTpdo3(data, len, _status);
  } else if (id == _cfg->tpdo4) {
    tm4::decodeTpdo4(data, len, _status);
  } else if (id != 0x700u + _cfg->slaveId) {
    return false;
  }
  _rxSeen = true;
  _lastRxMs = nowMs;
  return true;
}

void Tm4Inverter::encodeRpdo(uint8_t index, uint8_t mainsRequest, uint8_t out[8]) const {
  switch (index) {
    case 0:
      tm4::encodeRpdo1(out, mainsRequest);
      return;
    case 1: {
      tm4::MotorRequest m;  // OFF: all zero, as in debugCAN_python_can_v1.py cell 13
      if (_phase != Phase::OFF) {
        m.controlMode = tm4::CONTROL_SPEED;
        m.pwmEnable = true;
      }
      if (_phase == Phase::RUNNING || _phase == Phase::STOPPING) {
        m.refEnable = true;
        m.torqueLimitPct = TM4_TORQUE_LIMIT_PCT;
        m.droopPct = TM4_DROOP_PCT;
        m.speedRpm = (_phase == Phase::RUNNING) ? _rpm : 0;
      }
      tm4::encodeRpdo2(out, m);
      return;
    }
    case 2:
      tm4::encodeRpdo3(out);
      return;
    default:
      tm4::encodeRpdo4(out, TM4_BATT_CHARGE_LIMIT_A, TM4_BATT_DISCHARGE_LIMIT_A);
      return;
  }
}

bool Tm4Inverter::healthy(uint32_t nowMs) const {
  return _cfg->present && _rxSeen && (nowMs - _lastRxMs <= TM4_RX_TIMEOUT_MS) &&
         _status.mainsState != tm4::MAINS_ALARMED &&
         _status.faultLevel != tm4::FAULT_BLOCKING &&
         _status.faultLevel != tm4::FAULT_STOPPING && !_pwmTimeout;
}

// ---- Tm4Can ---------------------------------------------------------------

bool Tm4Can::begin(uint32_t nowMs) {
  for (uint8_t i = 0; i < TM4_INVERTER_COUNT; i++) _inv[i].init(&TM4_INVERTERS[i]);
  _healthy = false;
  // The GIGA `CAN` object is FDCAN2 on the CANRX/CANTX header (D93/D94),
  // wired to the isolated transceiver. SmartView: Baud Rate 250K.
  _busStarted = CAN.begin(CanBitRate::BR_250k);
  setTransmit(_busStarted, nowMs);
  return _busStarted;
}

void Tm4Can::setTransmit(bool on, uint32_t nowMs) {
  on = on && _busStarted;
  if (on && !_transmit) {
    // Spread every frame of every inverter across the period, 1 ms apart.
    uint8_t slot = 0;
    for (uint8_t k = 0; k < TM4_INVERTER_COUNT; k++) {
      for (uint8_t i = 0; i < 4; i++) {
        _inv[k].nextTxMs[i] = nowMs + (slot * TM4_RPDO_STAGGER_MS) % TM4_RPDO_PERIOD_MS;
        if (TM4_INVERTERS[k].present) slot++;
      }
    }
  }
  if (!on) {
    for (uint8_t k = 0; k < TM4_INVERTER_COUNT; k++) _inv[k].forceOff();
  }
  _transmit = on;
}

bool Tm4Can::send(uint16_t id, const uint8_t data[8]) {
  const CanMsg msg(CanStandardId(id), 8, data);
  if (CAN.write(msg) <= 0) {
    _txErrors++;
    if (_consecutiveTxErrors < 255) _consecutiveTxErrors++;
    return false;
  }
  _txCount++;
  _consecutiveTxErrors = 0;
  return true;
}

void Tm4Can::update(uint32_t nowMs) {
  if (!_busStarted) {
    _healthy = false;
    return;
  }

  while (CAN.available()) {
    const CanMsg msg = CAN.read();
    const uint32_t id = msg.isExtendedId() ? msg.getExtendedId() : msg.getStandardId();
    _rxCount++;
    if (!msg.isExtendedId()) {
      for (uint8_t k = 0; k < TM4_INVERTER_COUNT; k++) {
        if (_inv[k].config().present && _inv[k].handleRx(id, msg.data, msg.data_length, nowMs)) break;
      }
    }
    if (_rxCallback) _rxCallback(id, msg.data, msg.data_length);
  }

  for (uint8_t k = 0; k < TM4_INVERTER_COUNT; k++) _inv[k].update(nowMs);

  if (_transmit) {
    const uint8_t mains = _powerRequest ? tm4::MAINS_REQ_POWER_READY_DIRECT : tm4::MAINS_REQ_STARTUP;
    for (uint8_t k = 0; k < TM4_INVERTER_COUNT; k++) {
      Tm4Inverter& inv = _inv[k];
      if (!inv.config().present) continue;
      for (uint8_t i = 0; i < 4; i++) {
        if ((int32_t)(nowMs - inv.nextTxMs[i]) < 0) continue;
        uint8_t data[8];
        inv.encodeRpdo(i, mains, data);
        send(inv.config().rpdo[i], data);
        inv.nextTxMs[i] += TM4_RPDO_PERIOD_MS;
        // After a stall, resume the period from now instead of bursting.
        if ((int32_t)(nowMs - inv.nextTxMs[i]) >= 0) inv.nextTxMs[i] = nowMs + TM4_RPDO_PERIOD_MS;
      }
    }
  }

  bool all = _consecutiveTxErrors < MAX_CONSECUTIVE_TX_ERRORS;
  bool any = false;
  for (uint8_t k = 0; k < TM4_INVERTER_COUNT; k++) {
    if (!_inv[k].config().present) continue;
    any = true;
    all = all && _inv[k].healthy(nowMs);
  }
  _healthy = any && all;
}
