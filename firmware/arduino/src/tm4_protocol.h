#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

// Byte layouts from docs/can/GSL_DBC_v0.dbc (all signals Intel byte order)
// and the TAU Generic Slave user manual, sections 6 (Powering) and 7 (Moving).
// Only Motor 1 is used: each AC-X1 drives one motor, RPDO3 (Motor 2) stays 0.
namespace tm4 {

// RPDO1 byte 0: Request_for_Mains_State.
enum MainsRequest : uint8_t {
  MAINS_REQ_STARTUP = 1,             // return to STARTUP from any state
  MAINS_REQ_POWER_READY_DIRECT = 8,  // proven on the bench by V2_CAN_Test.py
};

// TPDO1 byte 0: MainsState.
enum MainsState : uint8_t {
  MAINS_UNKNOWN = 0,
  MAINS_STARTUP = 1,
  MAINS_POWER_READY = 6,
  MAINS_ALARMED = 7,  // power stage locked; only a key cycle resets it
};

// TPDO3 bits 61..63: Fault_Level.
enum FaultLevel : uint8_t {
  FAULT_READY = 0,
  FAULT_BLOCKING = 1,
  FAULT_STOPPING = 2,
  FAULT_LIMITING = 3,
  FAULT_WARNING = 4,
};

enum ControlMode : uint8_t { CONTROL_NONE = 0, CONTROL_SPEED = 1, CONTROL_TORQUE = 2 };

// RPDO2 Motor 1 request fields.
struct MotorRequest {
  int16_t speedRpm = 0;          // Motor1_SpeedRef_Lim, -10000..10000
  float torqueLimitPct = 0.0f;   // Motor1_TorqueRef_Lim, scale 0.0030581 %
  float droopPct = 0.0f;         // Motor1_DroopRate, scale 0.392157 %
  uint8_t controlMode = CONTROL_NONE;
  bool pwmEnable = false;
  bool refEnable = false;
  bool emergencyStop = false;
  bool safeStop = false;
  bool speedLimit = false;
};

// Feedback decoded from TPDO1/3/4.
struct Status {
  uint8_t mainsState = MAINS_UNKNOWN;
  uint8_t faultCode = 0;
  float dcBusVoltage = 0.0f;
  uint8_t faultLevel = FAULT_READY;
  int16_t actualRpm = 0;
  uint8_t controlMode = CONTROL_NONE;
  bool pwmOutput = false;
  uint8_t motorState = 0;
};

inline void put16(uint8_t* d, uint16_t v) {
  d[0] = (uint8_t)(v & 0xFF);
  d[1] = (uint8_t)(v >> 8);
}

inline int16_t getS16(const uint8_t* d) {
  return (int16_t)(uint16_t)(d[0] | (d[1] << 8));
}

inline long clampRaw(float raw, long lo, long hi) {
  const long r = lroundf(raw);
  return r < lo ? lo : (r > hi ? hi : r);
}

inline void encodeRpdo1(uint8_t d[8], uint8_t mainsRequest) {
  memset(d, 0, 8);
  d[0] = mainsRequest;
}

inline void encodeRpdo2(uint8_t d[8], const MotorRequest& m) {
  memset(d, 0, 8);
  put16(&d[0], (uint16_t)m.speedRpm);
  put16(&d[2], (uint16_t)(int16_t)clampRaw(m.torqueLimitPct / 0.0030581f, -32768, 32767));
  d[4] = (uint8_t)clampRaw(m.droopPct / 0.392157f, 0, 255);
  d[5] = (uint8_t)((m.controlMode & 0x03) | (m.pwmEnable << 2) | (m.refEnable << 3) |
                   (m.emergencyStop << 4) | (m.safeStop << 5) | (m.speedLimit << 6));
}

// RPDO3 carries Motor 2, unused on a single-motor AC-X1: all zero.
inline void encodeRpdo3(uint8_t d[8]) {
  memset(d, 0, 8);
}

inline void encodeRpdo4(uint8_t d[8], uint16_t chargeLimitA, uint16_t dischargeLimitA) {
  memset(d, 0, 8);
  put16(&d[0], chargeLimitA);
  put16(&d[2], dischargeLimitA);
}

inline void decodeTpdo1(const uint8_t* d, uint8_t len, Status& s) {
  if (len < 4) return;
  s.mainsState = d[0];
  s.faultCode = d[1];
  s.dcBusVoltage = getS16(&d[2]) * 0.1f;
}

inline void decodeTpdo3(const uint8_t* d, uint8_t len, Status& s) {
  if (len < 8) return;
  s.faultLevel = (uint8_t)(d[7] >> 5);
}

inline void decodeTpdo4(const uint8_t* d, uint8_t len, Status& s) {
  if (len < 8) return;
  s.actualRpm = getS16(&d[0]);
  s.controlMode = (uint8_t)(d[6] & 0x03);
  s.pwmOutput = (d[6] >> 2) & 0x01;
  s.motorState = (uint8_t)(d[7] & 0x0F);
}

}  // namespace tm4
