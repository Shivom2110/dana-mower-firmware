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
  // Normalized manual controls (-1..+1). These are placeholders for now.
  float throttleNorm = 0.0f;  // -1 back .. +1 forward
  float steeringNorm = 0.0f;  // -1 left .. +1 right

  // Operator/safety inputs
  bool estopPressed = false;      // true => emergency stop
  bool operatorEnable = false;    // true => operator requests drive enabled
  bool faultAcknowledge = false;  // true => operator acknowledges fault reset

  // For watchdog logic (if inputs stop updating)
  bool inputSignalAlive = true;
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
