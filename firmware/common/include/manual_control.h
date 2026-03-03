#pragma once
#include "types.h"

// Converts throttle/steering into left/right wheel commands (differential drive).
class ManualMixer {
public:
  WheelCommand mix(const DriveCommand& cmd) const;
};
