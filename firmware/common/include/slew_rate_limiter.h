#pragma once
#include "types.h"

// Limits how fast a value can change per second.
class SlewRateLimiter {
public:
  explicit SlewRateLimiter(float maxDeltaPerSec) : _maxDeltaPerSec(maxDeltaPerSec) {}

  float step(float target, float dtSec) {
    const float maxStep = _maxDeltaPerSec * dtSec;
    const float delta = target - _value;

    if (delta >  maxStep) _value += maxStep;
    else if (delta < -maxStep) _value -= maxStep;
    else _value = target;

    return _value;
  }

  void reset(float v = 0.0f) { _value = v; }

private:
  float _maxDeltaPerSec;
  float _value = 0.0f;
};
