#pragma once

#include <stdint.h>

// Inverter network and command settings. The IDs must match each inverter's
// SmartView configuration (CAN Network > CAN Interface / PDOs).

enum class InverterRole : uint8_t { DECK, DRIVE_LEFT, DRIVE_RIGHT };

struct InverterConfig {
  const char* name;
  InverterRole role;
  bool present;       // false until configured in SmartView and bench-tested
  uint8_t slaveId;    // CAN Interface > Slave ID; heartbeat is 0x700 + slaveId
  uint16_t rpdo[4];   // PDOs > RX IDs for RPDO1..RPDO4
  uint16_t tpdo1, tpdo3, tpdo4;  // PDOs > TX IDs that are decoded
  int8_t direction;   // +1/-1 so a positive command is forward / cutting direction
};

// Inverter numbering follows the wiring pinout: 1-2 deck, 3-4 drive.
static const InverterConfig TM4_INVERTERS[] = {
  // Configured and bench-tested (SmartView 2026-09-16): 250K, Slave ID 2,
  // full autonomous startup, "Node Id added to PDO Id" off, TPDOs every
  // 100 ms, RPDO timeout 100 ms.
  {"INV1 deck", InverterRole::DECK, true, 2,
   {0x201, 0x301, 0x401, 0x501}, 0x182, 0x382, 0x482, +1},

  // Not configured yet. Each inverter on the shared bus needs unique IDs for
  // all its PDOs (TPDO1..6 and RPDO1..4) and a unique Slave ID. Program these
  // suggested values in SmartView, verify, then set present = true.
  {"INV2 deck", InverterRole::DECK, false, 3,
   {0x202, 0x302, 0x402, 0x502}, 0x183, 0x383, 0x483, +1},
  {"INV3 drive L", InverterRole::DRIVE_LEFT, false, 4,
   {0x203, 0x303, 0x403, 0x503}, 0x184, 0x384, 0x484, +1},
  {"INV4 drive R", InverterRole::DRIVE_RIGHT, false, 5,
   {0x204, 0x304, 0x404, 0x504}, 0x185, 0x385, 0x485, +1},
};
static constexpr uint8_t TM4_INVERTER_COUNT =
    sizeof(TM4_INVERTERS) / sizeof(TM4_INVERTERS[0]);

// Timing proven by V2_CAN_Test.py: each RPDO every 10 ms, 1 ms stagger. The
// inverters fault if an RPDO is missing for 100 ms. With all four inverters
// streaming at 250 kbit/s, 10 ms is ~85% bus load; use 20 ms there.
static constexpr uint32_t TM4_RPDO_PERIOD_MS = 10;
static constexpr uint32_t TM4_RPDO_STAGGER_MS = 1;

// Values from V2_CAN_Test.py / V2_Motor_Control.py / Jens' debugCAN.m.
// Mains request 8 is the manual's single-controller "Fast Power Ready"
// procedure; with several inverters the manual recommends the Complete Power
// Ready procedure (2 -> 4 -> 6) to avoid pre-charge faults.
static constexpr uint16_t TM4_BATT_DISCHARGE_LIMIT_A = 20;
static constexpr uint16_t TM4_BATT_CHARGE_LIMIT_A = 0;
static constexpr float TM4_TORQUE_LIMIT_PCT = 50.0f;
static constexpr float TM4_DROOP_PCT = 0.0f;  // manual suggests 25% for smoother speed control

// Bench values (Jens' test used 200 rpm). Raise only after testing.
static constexpr int16_t TM4_DECK_RPM = 200;
static constexpr int16_t TM4_DRIVE_MAX_RPM = 200;

static constexpr uint32_t TM4_PWM_CONFIRM_TIMEOUT_MS = 10000;  // debugCAN.m
static constexpr uint32_t TM4_STOP_HOLD_MS = 500;               // V2_Motor_Control.py
static constexpr uint32_t TM4_RX_TIMEOUT_MS = 300;              // 3 missed 100 ms TPDOs
