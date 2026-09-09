#pragma once

#include <stdint.h>

// Transport boundary for the four Dana TM4 inverters. Frame IDs, byte layout,
// scaling, and enable sequencing must come from the Dana CAN documentation.
class Tm4Can {
public:
  bool begin();
  void update(uint32_t nowMs);
  bool healthy() const;
  bool sendZeroTorque();
  bool sendWheelCommands(float leftNorm, float rightNorm);

private:
  bool _busStarted = false;
  bool _healthy = false;
  uint32_t _lastStatusMs = 0;
};
