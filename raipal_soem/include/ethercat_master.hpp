//
// Created by dongghk on 25. 9. 4.
//

#ifndef ETHERCAT_MASTER_HPP
#define ETHERCAT_MASTER_HPP


// include/ethercat_master.hpp
#pragma once
#include <string>
#include <stdexcept>
#include "soem_c_api.hpp"

class EthercatMaster {
public:
    explicit EthercatMaster(const std::string& ifname);
    ~EthercatMaster();

    // Single bus exchange: send RxPDO, receive TxPDO
    int tickOnce();
    int expectedWKC() const { return expectedWKC_; }
    bool inOP() const { return inOP_; }
    void setInOP(bool v) { inOP_ = v; }

    // UPDATE: expose discovered slave count for auto-loop creation
    int slaveCount() const { return slaveCount_; }


private:
    char   IOmap_[4096]{};
    int    expectedWKC_{0};
    bool   inOP_{false};
    int    slaveCount_{0}; // UPDATE: store number of discovered slaves


    // UPDATE: discovery & mapping routine
    bool discoverAndMap(const char* ifname);
};

#endif //ETHERCAT_MASTER_HPP
