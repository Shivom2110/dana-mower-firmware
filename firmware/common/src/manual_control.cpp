#include "manual_control.h"

WheelCommand ManualMixer::mix(const DriveCommand& cmd) const {
  // Differential mix:
  // left  = throttle + steering
  // right = throttle - steering
  WheelCommand w;
  w.left  = clamp(cmd.throttle + cmd.steering, -1.0f, 1.0f);
  w.right = clamp(cmd.throttle - cmd.steering, -1.0f, 1.0f);
  return w;
}
