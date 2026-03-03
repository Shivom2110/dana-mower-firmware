#pragma once
#include "types.h"

struct SafetyStatus {
  bool mustFault = false;     // if true => force FAULT state now
  bool canEnableDrive = false; // if true => okay to enable motors/brakes release
  bool faultCleared = false;  // estop released + conditions okay
};

class SafetyManager {
public:
  void begin();

  // nowMs is passed so we can do watchdog style checks.
  SafetyStatus update(const InputSnapshot& in, uint32_t nowMs);

private:
  uint32_t _lastAliveMs = 0;
  bool _latchedFault = false;
};
