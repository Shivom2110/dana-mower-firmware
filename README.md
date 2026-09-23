# DANA Mower Firmware

Manual-drive controller firmware for an Arduino GIGA R1 WiFi and four Dana TM4
AC-X1 inverters (TAU Generic Slave firmware) on an isolated CAN bus. The GIGA
replaces the laptop and PCAN adapter used for bench testing.

## Hardware contract

- A0: left handle potentiometer; A1: right handle potentiometer (both 3.3 V).
- D2: conditioned ignition input; D3: conditioned normally-closed E-stop input.
- D4: conditioned deck (blade) motor switch input.
- D93: CAN RX; D94: CAN TX, connected to the logic side of the isolated CAN
  transceiver.
- The CAN bus has exactly two 120 ohm terminators: at the controller and at the
  physical final inverter. Intermediate inverter terminators stay disabled.

Never connect the 12 V ignition, E-stop, or deck-switch wiring directly to GIGA
pins. The conditioner must provide a defined, protected 3.3 V input and fail
safely on a broken E-stop circuit.

Inverter numbering follows the wiring pinout: inverters 1 and 2 drive the deck
motors, and inverters 3 and 4 drive the left and right wheels.

## TM4 CAN protocol

The protocol comes from `docs/can/`: the Dana DBC (`GSL_DBC_v0.dbc`), Jens'
`debugCAN.m`, the proven laptop scripts (`V2_CAN_Test.py`,
`V2_Motor_Control.py`, `debugCAN_python_can_v1.py`), and SmartView screenshots
of the configured inverter. The firmware encodes the same bytes as cantools
does for the DBC, verified for every RPDO state the firmware sends.

- **Streaming.** From boot, the GIGA sends RPDO1..RPDO4 to every configured
  inverter every 10 ms, staggered 1 ms apart, at 250 kbit/s. The inverters are
  set up with a 100 ms RPDO timeout, so streaming never pauses while they are
  powered. If the GIGA stops or resets, the inverters time out on their own.
- **Power.** RPDO1 requests Power Ready Direct (8) while ignition is on and the
  E-stop is released. Otherwise it requests a return to Startup (1). RPDO4
  sends a battery discharge limit of 20 A and a charge limit of 0 A.
- **Motor sequence** (RPDO2, Motor 1), following Jens' procedure:
  1. Request speed mode and PWM enable.
  2. Wait for TPDO4 `Motor1_PWM_Output = 1`. If it does not arrive within
     10 s, the inverter reports unhealthy.
  3. Enable the speed reference and send the commanded rpm, with a 50% torque
     limit.
  4. To stop, hold 0 rpm for 500 ms, then clear all RPDO2 fields.

  On a fault or E-stop, RPDO2 is cleared immediately.
- **Commands.** The drive wheels take the mixed handle command multiplied by
  `TM4_DRIVE_MAX_RPM`. The deck motors run at `TM4_DECK_RPM` while the deck
  switch is on. Both default to Jens' bench value of 200 rpm, set in
  `tm4_config.h`.
- **Health.** An inverter is healthy when:
  - it sent a TPDO or heartbeat within the last 300 ms (its TPDOs repeat every
    100 ms);
  - it is not ALARMED;
  - it reports no Blocking or Stopping fault;
  - it has no PWM-confirmation timeout.

  CAN is healthy only when every configured inverter is healthy.

### Inverter configuration

`firmware/arduino/src/tm4_config.h` lists the four inverters. Only INV1 (deck)
is configured and bench-tested so far. Its settings, taken from SmartView on
2026-09-16:

- Slave ID 2, heartbeat on 0x702. The DBC lists 0x701, which is wrong for this
  inverter.
- Startup management set to Full Autonomously.
- "Node Id added to PDO Id" turned off.
- RPDO IDs 0x201, 0x301, 0x401, 0x501, each with a 100 ms timeout.
- TPDO IDs 0x182 to 0x6A2, sent every 100 ms.

INV2 to INV4 share the bus, so each needs its own Slave ID and its own IDs for
all PDOs. Program the IDs suggested in `tm4_config.h` in SmartView, verify
them with the bench tool, then set `present = true` and check each motor's
`direction`. Before running all four:

- Raise `TM4_RPDO_PERIOD_MS` to 20. At 10 ms, four inverters load the
  250 kbit/s bus to about 85%.
- Consider switching from Power Ready Direct (8) to the manual's Complete Power
  Ready sequence (2 → 4 → 6). The manual warns that request 8 can cause
  pre-charge faults with more than one controller.

### Motor brake

The spring-applied brake on each motor is controlled only by its inverter,
under SmartView > Motor/Control1 > Safety Functions > Safe Brake. The GIGA
cannot command it. INV1 is configured as follows:

- **Output:** Driver Output 2 (K1-27) at 1000 Hz. This is a low-side PWM
  output, so the coil's positive side must go to **Coil Return (K1-25)**,
  which carries battery/KEY voltage (48 V).
- **Voltage:** Pull-In 50% (24 V) for 100 ms, then Hold 30% (14 V). SmartView
  calculates these as a percentage of battery voltage.
- **Timing:** opening delay 100 ms, closing delay 100 ms, closing timeout 0
  (disabled).

The inverter opens the brake only when:

- PWM is enabled;
- the reference is enabled;
- a non-zero speed is requested.

Streaming alone (`V2_CAN_Test.py`, or bench command `s`) never releases it.

If the brake stays on with a speed command active:

1. **Clear any Blocking fault first.** With a Blocking fault, PWM never
   enables. SmartView on 2026-09-16 showed ENCODER1 FAULT [30] active. The
   encoder's Sin input also sat at about 4.0 V with almost no swing, where a
   sin/cos signal would normally center near 2.5 V. Check the encoder supply
   and wiring.
2. **Check the coil wiring.** If the coil's positive side is on a separate
   24 V supply instead of Coil Return, 50%/30% PWM gives only about 12 V/7 V
   and the brake will not lift.
3. **Isolate brake from inverter.** Apply 24 V directly to the coil from a
   bench supply. If the wheel then turns freely, the brake is fine and the
   problem is in the inverter output or its configuration.
4. **Set a Brake Closing Timeout.** The manual strongly recommends it.
   Consider raising Pull-In Time and the opening delay to the manual's
   suggested 200 ms.

## Safety behaviour

Drive (`MANUAL`) engages only when all of these hold:

- ignition is on;
- the E-stop is released;
- CAN is healthy;
- the handles are neutral;
- the deck switch is off.

Once engaged, moving the handles or switching the deck on is allowed. Losing
any of the first three conditions leaves drive.

An E-stop, ignition-off, or loss of CAN health after it was established latches
a fault. To clear it, turn the ignition off, then on again with the E-stop
released, CAN healthy, the handles neutral, and the deck switch off. CAN that is
still coming up at power-on only blocks drive; it does not latch a fault.

The physical E-stop must independently remove power. This software logic is an
additional safeguard, not its replacement.

## Layout

- `firmware/common/`: hardware-independent safety, mixer, state, and slew
  logic, as a PlatformIO library.
- `firmware/arduino/src/pins.h`: the single source of truth for the wiring.
- `firmware/arduino/src/hal_arduino.cpp`: reads the GIGA inputs and maps wheel
  and deck commands onto the inverters.
- `firmware/arduino/src/main.cpp`: 10 ms control loop. CAN is serviced on every
  pass so RPDO timing stays exact.
- `firmware/arduino/src/tm4_protocol.h`: DBC byte layouts (encode and decode).
- `firmware/arduino/src/tm4_config.h`: inverter IDs, roles, and command limits.
- `firmware/arduino/src/tm4_can.*`: RPDO streaming, TPDO decoding, per-inverter
  motor sequencing, and health.
- `firmware/arduino/src/bench_can_test.cpp`: bench replacement for the laptop
  scripts.
- `docs/can/`: DBC, reference scripts, and SmartView configuration screenshots.

```text
potentiometers / ignition / E-stop / deck switch / TPDO feedback
                 |
                 v
           GIGA hardware layer
                 |
                 v
 safety manager -> state machine -> drive mixer -> slew limiter -> TM4 CAN
```

## Build

Install PlatformIO, then run from `firmware/arduino`:

```sh
pio run -e giga_r1_m7 -t upload               # full controller
pio run -e giga_r1_m7_can_bench -t upload     # bench tool
pio device monitor -b 115200
```

The bench tool (`giga_r1_m7_can_bench`) replaces `V2_CAN_Test.py` plus
`V2_Motor_Control.py`. It ignores the handles, ignition, deck switch, and E-stop
input. Type these commands in the serial monitor:

- `s`: start RPDO streaming with the motors off.
- a number: run the motors at that rpm.
- `x`: stop the motors gracefully.
- `q`: stop the motors immediately and stop streaming.
- `v`: toggle printing of raw received frames.

It prints mains state, fault code and level, PWM state, actual rpm, and DC
voltage every second. Use it first to confirm that the GIGA spins the deck
motor exactly as the laptop did.

The full controller prints its state and the same per-inverter status once a
second.
