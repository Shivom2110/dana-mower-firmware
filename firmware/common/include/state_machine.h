#pragma once
#include "types.h"

class StateMachine {
public:
  SystemState state() const { return _state; }
  void setState(SystemState s) { _state = s; }

private:
  SystemState _state = SystemState::SAFE_IDLE;
};
