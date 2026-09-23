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
  // CAN that has not come up yet (inverters still booting) only blocks drive;
  // CAN that was healthy and then lost latches a fault.
  const bool canAlive = (_lastAliveMs != 0) && (nowMs - _lastAliveMs <= INPUT_TIMEOUT_MS);
  const bool canLost = (_lastAliveMs != 0) && !canAlive;
  const bool controlsNeutral =
      (in.throttleNorm > -NEUTRAL_THRESHOLD && in.throttleNorm < NEUTRAL_THRESHOLD) &&
      (in.steeringNorm > -NEUTRAL_THRESHOLD && in.steeringNorm < NEUTRAL_THRESHOLD);
  // Blades must not start the moment drive engages.
  const bool readyToEngage = controlsNeutral && !in.deckSwitchOn;

  if (in.estopPressed || canLost || !in.ignitionOn) {
    _latchedFault = true;
  }

  // No reset button appears in the wiring diagram. A fault requires an
  // ignition-off cycle, then ignition-on with neutral controls, deck switch
  // off, and healthy CAN.
  if (_latchedFault && !in.ignitionOn) {
    _ignitionWasOffSinceFault = true;
  }
  if (_latchedFault && _ignitionWasOffSinceFault && in.ignitionOn &&
      !in.estopPressed && canAlive && readyToEngage) {
    _latchedFault = false;
    _ignitionWasOffSinceFault = false;
  }

  s.mustFault = _latchedFault;

  s.driveAllowed = (!_latchedFault) && in.ignitionOn && !in.estopPressed && canAlive;
  s.canEnableDrive = s.driveAllowed && readyToEngage;

  s.faultCleared = !_latchedFault;
  return s;
}
