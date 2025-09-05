//
// Created by dongghk on 25. 9. 4.
//

// src/ethercat_master.cpp
#include "ethercat_master.hpp"
#include <cstring>
#include <cstdio>

EthercatMaster::EthercatMaster(const std::string& ifname) {
    if (!ec_init(ifname.c_str()))
        throw std::runtime_error("ec_init failed (are you root, correct NIC?)");

    if (ec_config_init(FALSE) <= 0) {
        ec_close();
        throw std::runtime_error("no EtherCAT slaves found");
    }
    // Map PDOs and DC
    ec_config_map(&IOmap_[0]);
    ec_configdc();

    // SAFE_OP, then ask for OP
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    expectedWKC_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    // send zeros once before OP request
    for (int s = 1; s <= ec_slavecount; ++s) {
        if (ec_slave[s].Obytes > 0) std::memset(ec_slave[s].outputs, 0, ec_slave[s].Obytes);
    }
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_writestate(0);
    int chk = 200;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        ec_close();
        throw std::runtime_error("not all slaves reached OP");
    }
    inOP_ = true;
}

EthercatMaster::~EthercatMaster() {
    // request INIT and close
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
}

int EthercatMaster::tickOnce() {
    ec_send_processdata();
    return ec_receive_processdata(EC_TIMEOUTRET);
}

