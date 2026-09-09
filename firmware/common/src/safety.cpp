#include "safety.h"

static constexpr uint32_t INPUT_TIMEOUT_MS = 500;
static constexpr float NEUTRAL_THRESHOLD = 0.05f;

void SafetyManager::begin() {
  _lastAliveMs = 0;
  _latchedFault = false;
  _ignitionWasOffSinceFault = false;
}

SafetyStatus SafetyManager::update(const InputSnapshot& in, uint32_t nowMs) {
  SafetyStatus s{};

  if (in.canNetworkHealthy) {
    _lastAliveMs = nowMs;
  }
  const bool timedOut = (_lastAliveMs == 0) || (nowMs - _lastAliveMs > INPUT_TIMEOUT_MS);
  const bool controlsNeutral =
      (in.throttleNorm > -NEUTRAL_THRESHOLD && in.throttleNorm < NEUTRAL_THRESHOLD) &&
      (in.steeringNorm > -NEUTRAL_THRESHOLD && in.steeringNorm < NEUTRAL_THRESHOLD);

  if (in.estopPressed || timedOut || !in.ignitionOn) {
    _latchedFault = true;
  }

  // No reset button appears in the wiring diagram. A fault requires an
  // ignition-off cycle, then ignition-on with neutral controls and healthy CAN.
  if (_latchedFault && !in.ignitionOn) {
    _ignitionWasOffSinceFault = true;
  }
  if (_latchedFault && _ignitionWasOffSinceFault && in.ignitionOn &&
      !in.estopPressed && !timedOut && controlsNeutral) {
    _latchedFault = false;
    _ignitionWasOffSinceFault = false;
  }

  s.mustFault = _latchedFault;

  s.canEnableDrive = (!_latchedFault) && in.ignitionOn && controlsNeutral && !timedOut;

  s.faultCleared = !_latchedFault;
  return s;
}
