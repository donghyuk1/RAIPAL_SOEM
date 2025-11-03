//
// Created by dongghk on 25. 9. 4.
//

#ifndef ETHERCAT_ACTUATOR_HPP
#define ETHERCAT_ACTUATOR_HPP


// include/ethercat_actuator.hpp
#pragma once
#include "ethercat_slave.hpp"
#include <cstdint>
#include <cstdio>

// ---- Your PDO map (from the C code) ----
struct ActuatorPDOMap {
  // RxPDO
  size_t off_target_pos  = 0x00; // int32
  size_t off_target_vel  = 0x04; // int32
  size_t off_target_tor  = 0x08; // int16
  size_t off_controlword = 0x0A; // u8  (your map uses 8-bit CW)
  size_t off_mode        = 0x0B; // u8
  size_t off_kp          = 0x0C; // u16
  size_t off_kd          = 0x0E; // u16
  // TxPDO
  size_t off_pos_act     = 0x00; // int32
  size_t off_tor_act     = 0x04; // int16
  size_t off_vel_act     = 0x06; // int32
  size_t off_status      = 0x0A; // u8 (low byte)
  size_t off_temp        = 0x0B; // u8
  size_t off_error       = 0x0C; // u8
};

// ---- Modes & controlwords ----
// ---- Modes of Operation ----
enum ModeOfOperation : uint8_t {
  MODE_NO_MODE                        = 0x00,
  MODE_PROFILE_POSITION               = 0x01,
  MODE_PROFILE_VELOCITY               = 0x03,
  MODE_PROFILE_TORQUE                 = 0x04,
  MODE_HOMING                         = 0x06,
  MODE_INTERPOLATION_POSITION         = 0x07,
  MODE_CYCLIC_SYNCHRONOUS_POSITION    = 0x08,
  MODE_CYCLIC_SYNCHRONOUS_VELOCITY    = 0x09,
  MODE_CYCLIC_SYNCHRONOUS_TORQUE      = 0x0A,
  MODE_PROFILE_COMMUTATION            = 0x0F
};

enum ControlWord : uint8_t {
  CW_ZERO              = 0x00,
  CW_SHUTDOWN          = 0x06,
  CW_SWITCH_ON         = 0x07,
  CW_ENABLE_OPERATION  = 0x0F,
  CW_DISABLE_OPERATION = 0x07,
  CW_FAULT_RESET       = 0x80
};

// Feedback/command mirrors
struct ActuatorFeedback {
  int32_t  pos{0};
  int32_t  vel{0};
  int16_t  tor{0};
  uint8_t  status{0};
  uint8_t  temp{0};
  uint8_t  error{0};
};

struct ActuatorCommand {
  uint8_t  controlword{CW_FAULT_RESET};
  uint8_t  mode{0};
  int32_t  target_pos{0};
  int32_t  target_vel{0};
  int16_t  target_tor{0};
  uint16_t kp{0};
  uint16_t kd{0};
};

class EthercatActuator : public EthercatSlave {
public:
  EthercatActuator(uint16_t index, const ActuatorPDOMap& map)
  : EthercatSlave(index), map_(map) {}

  // Read TxPDO -> feedback
  void readFeedback(ActuatorFeedback& fb) const {
    const auto* in = inBase();
    fb.pos   = rd_s32(in, map_.off_pos_act);
    fb.vel   = rd_s32(in, map_.off_vel_act);
    fb.tor   = rd_s16(in, map_.off_tor_act);
    fb.status= rd_u8 (in, map_.off_status);
    fb.temp  = rd_u8 (in, map_.off_temp);
    fb.error = rd_u8 (in, map_.off_error);
  }

  // Write RxPDO from command
  void writeCommand(const ActuatorCommand& cmd) {
    auto* out = outBase();
    wr_u8 (out, map_.off_controlword, cmd.controlword);
    wr_u8 (out, map_.off_mode,       cmd.mode);
    wr_s32(out, map_.off_target_pos, cmd.target_pos);
    wr_s32(out, map_.off_target_vel, cmd.target_vel);
    wr_s16(out, map_.off_target_tor, cmd.target_tor);
    // gains (host->LE is safe; your FW expects LE per your C code)
    uint16_t kp = cmd.kp, kd = cmd.kd;
    wr_s16(out, map_.off_kp, static_cast<int16_t>(kp));
    wr_s16(out, map_.off_kd, static_cast<int16_t>(kd));
  }

  // Small CiA-402 helper (matches your C logic on low byte)
  void advanceCiA402(const ActuatorFeedback& fb, ActuatorCommand& cmd) {
    // Your states (low byte masked in your C)
    switch (fb.status) {
      case 0x21: /* ReadyToSwitchOn */  cmd.controlword = CW_SWITCH_ON; break;
      case 0x23: /* SwitchedOn       */  cmd.controlword = CW_ENABLE_OPERATION; break;
      case 0x27: /* OperationEnabled */  cmd.controlword = CW_ENABLE_OPERATION; break;
      case 0x40: /* SwitchedOnDisabled */ cmd.controlword = CW_SHUTDOWN; break;
      case 0x08: /* Fault */              cmd.controlword = CW_FAULT_RESET; break;
      default: /* keep last */ break;
    }
  }

private:
  ActuatorPDOMap map_;
};

#endif //ETHERCAT_ACTUATOR_HPP
