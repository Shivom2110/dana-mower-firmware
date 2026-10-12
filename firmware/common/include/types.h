#pragma once
#include <stdint.h>

enum class SystemState : uint8_t {
  SAFE_IDLE = 0,
  MANUAL    = 1,
  AUTO      = 2,   // stub for later
  DOCKING   = 3,   // stub for later
  FAULT     = 4
};

struct InputSnapshot {
  // Skid-steer: each handle maps directly to one drive wheel.
  // Left handle (A0): -1 = full reverse, 0 = neutral, +1 = full forward.
  // Right handle (A1): same convention.
  float leftHandleNorm  = 0.0f;
  float rightHandleNorm = 0.0f;

  // Operator/safety inputs from the GIGA wiring diagram.
  bool ignitionOn = false;
  bool estopPressed = true;        // true => emergency stop
  bool deckSwitchOn = false;       // operator request for the blade motors
  bool canNetworkHealthy = false;  // true only after all required TM4s report healthy
};

struct DriveCommand {
  float left  = 0.0f;   // -1..+1, left wheel speed
  float right = 0.0f;   // -1..+1, right wheel speed
  bool  enableDrive = false;
};

struct WheelCommand {
  float left  = 0.0f;      // -1..+1
  float right = 0.0f;      // -1..+1
};

static inline float clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
