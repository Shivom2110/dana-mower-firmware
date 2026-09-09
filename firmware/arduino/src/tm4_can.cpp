#include "tm4_can.h"

#include <Arduino.h>

bool Tm4Can::begin() {
  // D93 (RX) / D94 (TX) are connected to the isolated transceiver. A concrete
  // driver and bitrate are deliberately not guessed: they must match the Dana
  // manual and be proven on the bench before torque can be enabled.
  _busStarted = false;
  _healthy = false;
  return false;
}

void Tm4Can::update(uint32_t nowMs) {
  (void)nowMs;
  // TODO: validate heartbeat/status from all four TM4 nodes, record the latest
  // status timestamp, and set _healthy only if no node reports a fault.
  _healthy = false;
}

bool Tm4Can::healthy() const {
  return _busStarted && _healthy;
}

bool Tm4Can::sendZeroTorque() {
  // TODO: transmit Dana-defined disable/zero-torque frames to every TM4.
  return false;
}

bool Tm4Can::sendWheelCommands(float leftNorm, float rightNorm) {
  (void)leftNorm;
  (void)rightNorm;
  // Fail closed: without approved frames this cannot command motion.
  return false;
}
