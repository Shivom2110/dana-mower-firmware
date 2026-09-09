# DANA Mower Firmware

Manual-drive controller firmware for an Arduino GIGA R1 WiFi and four Dana TM4
inverters on an isolated CAN bus.

## Hardware contract

- A0: left handle potentiometer; A1: right handle potentiometer (both 3.3 V).
- D2: conditioned ignition input; D3: conditioned normally-closed E-stop input.
- D93: CAN RX; D94: CAN TX, connected to the logic side of the isolated CAN
  transceiver.
- The CAN bus has exactly two 120 ohm terminators: at the controller and at the
  physical final inverter. Intermediate inverter terminators stay disabled.

Never connect the 12 V ignition or E-stop wiring directly to GIGA pins. The
conditioner must provide a defined, protected 3.3 V input and fail safely on a
broken E-stop circuit.

## Current safety state

The code is intentionally torque-locked. The Dana protocol documents are still
required to configure CAN mode/bitrate, node IDs, command frames, status frame
validation, fault decoding, and the inverter enable sequence. Until those are
implemented in `firmware/arduino/src/tm4_can.cpp`, CAN health is false and the
state machine stays faulted.

An E-stop or loss of all-inverter CAN health latches a fault. Clearing requires
an ignition-off cycle followed by ignition-on with the controls neutral and the
CAN network healthy. The physical E-stop must independently remove torque; this
software logic is an additional safeguard, not its replacement.

## Layout

- `firmware/common/`: hardware-independent safety, mixer, state, and slew logic.
- `firmware/arduino/`: Arduino GIGA board HAL, pin map, TM4 CAN boundary, and
  PlatformIO configuration.

## Codebase overview

- `firmware/common/` is the reusable control layer. It contains the system
  states, safety decisions, differential-drive mixer, and acceleration limits;
  it does not know any board pin numbers or CAN frame details.
- `firmware/arduino/src/pins.h` is the single source of truth for the wiring:
  A0/A1 for the handle potentiometers, D2 for ignition, D3 for the E-stop, and
  D93/D94 for the isolated CAN transceiver.
- `firmware/arduino/src/hal_arduino.cpp` is the hardware abstraction layer. It
  reads the GIGA inputs and exposes them as board-independent data to the
  control layer.
- `firmware/arduino/src/main.cpp` is the application loop. It runs at a target
  rate of 100 Hz and coordinates safety, state changes, drive mixing, and output.
- `firmware/arduino/src/tm4_can.*` is the only place Dana TM4 CAN traffic
  belongs. Keeping it separate prevents inverter protocol details from leaking
  into safety or driving logic.

### Runtime flow

```text
potentiometers / ignition / E-stop / CAN status
                 |
                 v
           GIGA hardware layer
                 |
                 v
 safety manager -> state machine -> drive mixer -> slew limiter -> TM4 CAN
```

Before drive can be enabled, ignition must be on, the E-stop must be released,
the controls must be neutral, and every required inverter must be healthy. An
E-stop or CAN-health failure creates a latched fault. Recovery requires an
ignition-off/on cycle under safe conditions.

## Build

Install PlatformIO, then run from `firmware/arduino`:

```sh
pio run
```

The target is `giga_r1_m7` using the Arduino framework.
