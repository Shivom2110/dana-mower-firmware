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
  float throttleNorm = 0.0f;  // -1 back .. +1 forward
  float steeringNorm = 0.0f;  // -1 left .. +1 right

  // Operator/safety inputs from the GIGA wiring diagram.
  bool ignitionOn = false;
  bool estopPressed = true;        // true => emergency stop
  bool canNetworkHealthy = false;  // true only after all required TM4s report healthy
};

struct DriveCommand {
  float throttle = 0.0f;   // -1..+1
  float steering = 0.0f;   // -1..+1
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
