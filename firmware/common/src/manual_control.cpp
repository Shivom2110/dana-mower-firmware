#include "manual_control.h"

WheelCommand ManualMixer::mix(const DriveCommand& cmd) const {
  // Skid-steer / zero-turn: each handle directly commands its own wheel.
  // No differential mixing — the operator steers by independently varying
  // left and right handle positions.
  WheelCommand w;
  w.left  = clamp(cmd.left,  -1.0f, 1.0f);
  w.right = clamp(cmd.right, -1.0f, 1.0f);
  return w;
}
