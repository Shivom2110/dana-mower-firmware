#include "safety.h"

static constexpr uint32_t INPUT_TIMEOUT_MS = 500; // placeholder watchdog

void SafetyManager::begin() {
  _lastAliveMs = 0;
  _latchedFault = false;
}

SafetyStatus SafetyManager::update(const InputSnapshot& in, uint32_t nowMs) {
  SafetyStatus s{};

  // Track input "alive" watchdog.
  if (in.inputSignalAlive) {
    _lastAliveMs = nowMs;
  }
  const bool timedOut = (_lastAliveMs != 0) && (nowMs - _lastAliveMs > INPUT_TIMEOUT_MS);

  // E-stop or timeout => latched fault.
  if (in.estopPressed || timedOut) {
    _latchedFault = true;
  }

  s.mustFault = _latchedFault;

  // We only allow drive when:
  // - no latched fault
  // - operator enable is true
  // - inputs are alive (not timed out)
  s.canEnableDrive = (!_latchedFault) && in.operatorEnable && !timedOut;

  // Fault is considered "cleared" only when estop is released and inputs alive.
  s.faultCleared = (!in.estopPressed) && !timedOut;

  // NOTE: _latchedFault remains latched until user acknowledges fault in FAULT state (handled in main loop).
  return s;
}
