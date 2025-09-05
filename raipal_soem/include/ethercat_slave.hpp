//
// Created by dongghk on 25. 9. 4.
//
// include/ethercat_slave.hpp


#ifndef ETHERCAT_SLAVE_HPP
#define ETHERCAT_SLAVE_HPP

#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>
#include "soem_c_api.hpp"

class EthercatSlave {
public:
  explicit EthercatSlave(uint16_t index) : idx_(index) {}
  virtual ~EthercatSlave() = default;

  uint16_t index() const { return idx_; }

  // Presence check: index in range AND either configadr != 0 or state > NONE
  bool present() const {
    if (idx_ == 0 || idx_ > static_cast<uint16_t>(ec_slavecount)) return false;
    const auto& s = ec_slave[idx_];
    return (s.configadr != 0) || (s.state > EC_STATE_NONE);
  }

  std::string name() const {
    return (present() && ec_slave[idx_].name) ? ec_slave[idx_].name : "";
  }

  // -------- SDO helpers (SOEM expects 'int' sizes) --------
  bool sdoRead(uint16_t idx, uint8_t subidx, void* buf, size_t& len) const {
    int ilen = static_cast<int>(len);
    int wkc = ec_SDOread(index(), idx, subidx, FALSE, &ilen, buf, EC_TIMEOUTRXM);
    if (wkc > 0) len = static_cast<size_t>(ilen);
    return (wkc > 0);
  }

  bool sdoWrite(uint16_t idx, uint8_t subidx, const void* buf, size_t len) const {
    int ilen = static_cast<int>(len);
    int wkc = ec_SDOwrite(index(), idx, subidx, FALSE, ilen, const_cast<void*>(buf), EC_TIMEOUTRXM);
    return (wkc > 0);
  }

  template<typename T>
  bool sdoReadT(uint16_t idx, uint8_t subidx, T& out) const {
    size_t len = sizeof(T);
    return sdoRead(idx, subidx, &out, len) && (len == sizeof(T));
  }

  template<typename T>
  bool sdoWriteT(uint16_t idx, uint8_t subidx, const T& value) const {
    return sdoWrite(idx, subidx, &value, sizeof(T));
  }

protected:
  // -------- PDO byte access (endianness same as your C helpers) --------
  const uint8_t* inBase()  const { return ec_slave[idx_].inputs;  }
  uint8_t*       outBase()       { return ec_slave[idx_].outputs; }

  static inline int32_t  rd_s32(const uint8_t* b, size_t off){ int32_t v; std::memcpy(&v,b+off,4); return etohl(v); }
  static inline int16_t  rd_s16(const uint8_t* b, size_t off){ int16_t v; std::memcpy(&v,b+off,2); return etohs(v); }
  static inline uint8_t  rd_u8 (const uint8_t* b, size_t off){ return *(b+off); }

  static inline void wr_s32(uint8_t* b, size_t off, int32_t v){ int32_t le = htoel(v); std::memcpy(b+off,&le,4); }
  static inline void wr_s16(uint8_t* b, size_t off, int16_t v){ int16_t le = htoes(v); std::memcpy(b+off,&le,2); }
  static inline void wr_u8 (uint8_t* b, size_t off, uint8_t v){ *(b+off) = v; }

private:
  uint16_t idx_;
};

#endif //ETHERCAT_SLAVE_HPP
