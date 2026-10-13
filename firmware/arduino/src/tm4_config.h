#pragma once

#include <stdint.h>

// Inverter network topology. IDs must match SmartView: CAN Network > CAN Interface / PDOs.

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

// Inverters 1-2: deck blades. Inverters 3-4: drive wheels.
// SmartView settings for all: 250K baud, "Node Id Added To PDO Id" = OFF,
// TPDO interval 100 ms, RPDO timeout 100 ms, full autonomous startup.
static const InverterConfig TM4_INVERTERS[] = {
  // INV1 — deck blade, Slave ID 2. Configured SmartView 2026-09-16.
  {"INV1 deck", InverterRole::DECK, false, 2,
   {0x201, 0x301, 0x401, 0x501}, 0x182, 0x382, 0x482, +1},

  // INV2 — deck blade, Slave ID 3. Not configured yet.
  {"INV2 deck", InverterRole::DECK, false, 3,
   {0x202, 0x302, 0x402, 0x502}, 0x183, 0x383, 0x483, +1},

  // INV3 — left drive wheel, Slave ID 4. Not configured yet.
  {"INV3 drive L", InverterRole::DRIVE_LEFT, false, 4,
   {0x203, 0x303, 0x403, 0x503}, 0x184, 0x384, 0x484, +1},

  // INV4 — right drive wheel, Slave ID 5. Configured SmartView (Inverter_4_Wheel.clon).
  {"INV4 drive R", InverterRole::DRIVE_RIGHT, true, 5,
   {0x205, 0x305, 0x405, 0x505}, 0x185, 0x385, 0x485, +1},
};
static constexpr uint8_t TM4_INVERTER_COUNT =
    sizeof(TM4_INVERTERS) / sizeof(TM4_INVERTERS[0]);

// RPDO period 10 ms, 1 ms stagger between frames. Inverters fault if RPDO
// is missing for 100 ms. Note: 4 inverters × 4 RPDOs at 250 kbit/s ≈ 85% bus
// load at 10 ms — raise to 20 ms if adding more nodes.
static constexpr uint32_t TM4_RPDO_PERIOD_MS  = 10;
static constexpr uint32_t TM4_RPDO_STAGGER_MS = 1;
// CANopen SYNC (COB-ID 0x080). Missing SYNC raises Fault 74 (CO Synchro Lost).
static constexpr uint32_t TM4_SYNC_PERIOD_MS  = 10;

// Mains request 8 = "Fast Power Ready" (single-controller procedure from manual).
// For multiple inverters use the staged sequence (2→4→6) to avoid pre-charge faults.
static constexpr uint16_t TM4_BATT_DISCHARGE_LIMIT_A = 20;
static constexpr uint16_t TM4_BATT_CHARGE_LIMIT_A    = 0;
static constexpr float    TM4_TORQUE_LIMIT_PCT        = 50.0f;
static constexpr float    TM4_DROOP_PCT               = 0.0f;  // 25% recommended for smoother speed control

static constexpr int16_t  TM4_DECK_RPM                = 200;
static constexpr int16_t  TM4_DRIVE_MAX_RPM            = 200;

static constexpr uint32_t TM4_PWM_CONFIRM_TIMEOUT_MS  = 10000;
static constexpr uint32_t TM4_STOP_HOLD_MS             = 500;
static constexpr uint32_t TM4_RX_TIMEOUT_MS            = 300;  // 3 missed 100 ms TPDOs
