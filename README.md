# DANA Mower Firmware

Firmware repository for the DANA autonomous lawn mower project.

## Current Scope (Phase 2/Now)
Manual driving only:
- Emergency stop handling (software side)
- Enable/disable logic
- Throttle + steering mixing to left/right motor commands
- Fault state behavior (stop + disable)

## Future Scope
- Autonomous navigation (path planning, obstacle handling, boundary enforcement)
- Docking mode
- Full robot state machine (manual/auto/fault/docking) with safety overrides

## Repo Layout
- `firmware/common/` MCU-agnostic logic (should remain stable across Arduino → STM32)
- `firmware/arduino/` Arduino glue code (temporary)
- `firmware/stm32/` STM32 glue code (future)

## Build (Arduino)
This repo uses PlatformIO (recommended).
