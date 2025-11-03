//
// Created by dongghk on 25. 9. 4.
//

// src/ethercat_master.cpp
#include "ethercat_master.hpp"
#include <cstring>
#include <cstdio>

EthercatMaster::EthercatMaster(const std::string& ifname)
{
    // UPDATE: call internal discovery function
    if (!discoverAndMap(ifname.c_str()))
        throw std::runtime_error("EtherCAT discovery failed (no slaves found or network issue)");

    // Request OP state
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    int chk = 200;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        ec_readstate();
        for (int i = 1; i <= slaveCount_; ++i) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                std::fprintf(stderr,
                    "Slave %d failed to reach OP (state=0x%02X, AL=0x%04X): %s\n",
                    i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                    ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
        ec_close();
        throw std::runtime_error("Not all slaves reached OPERATIONAL");
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


// ---------------------------------------------------------------------------
// UPDATE: discoverAndMap() — performs ec_init, slave scan, map, and DC setup
// ---------------------------------------------------------------------------
bool EthercatMaster::discoverAndMap(const char* ifname)
{
    printf("EthercatMaster: starting discovery on %s\n", ifname);

    if (!ec_init(ifname)) {
        std::fprintf(stderr, "ERROR: ec_init failed on %s\n", ifname);
        return false;
    }

    // Scan and configure slaves
    if (ec_config_init(FALSE) <= 0) {
        std::fprintf(stderr, "ERROR: no EtherCAT slaves found on %s\n", ifname);
        ec_close();
        return false;
    }

    // Map PDOs & configure distributed clocks
    ec_config_map(&IOmap_[0]);
    ec_configdc();

    // Wait for SAFE_OP (same as slaveinfo.c)
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);

    // Store slave count and expected work counter
    slaveCount_  = ec_slavecount;
    expectedWKC_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

    printf("%d slaves found and configured.\n", slaveCount_);
    printf("Calculated expected WKC: %d\n", expectedWKC_);

    // (Optional) print quick summary like slaveinfo.c
    for (int i = 1; i <= slaveCount_; ++i) {
        printf("  Slave %d: %s | Output: %d bytes | Input: %d bytes | Has DC: %d\n",
               i, ec_slave[i].name, ec_slave[i].Obytes, ec_slave[i].Ibytes, ec_slave[i].hasdc);
    }

    // Send zeroed outputs once (same as before)
    for (int s = 1; s <= slaveCount_; ++s) {
        if (ec_slave[s].Obytes > 0 && ec_slave[s].outputs) {
            std::memset(ec_slave[s].outputs, 0, ec_slave[s].Obytes);
        }
    }
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    return (slaveCount_ > 0);
}
