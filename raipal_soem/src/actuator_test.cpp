//
// Created by dongghk on 25. 11. 3.
//
// Created by dongghk on 25. 9. 4.
// src/actuator_test.cpp
#include <cstdio>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <csignal>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

// ---------- Test Parameters ----------
static constexpr auto    kControlPeriod     = std::chrono::microseconds(5000); // 5ms
static constexpr auto    kLegDuration       = std::chrono::minutes(1);         // 5 minutes per velocity leg


// THERMAL WATCHDOG: thresholds with hysteresis
static constexpr int     kTempHighC = 60;    // disable when >= 60°C
static constexpr int     kTempLowC  = 50;    // re-enable when <= 50°C


static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

// ---------- YAML config (same shape & style as main.cpp) ----------
struct actuator_cfg_t {
    std::string name;
    uint8_t  mode;              // not strictly used (we force modes per stage)
    int16_t  target_torque;
    int32_t  target_velocity;   // used for +/− velocity legs
    int32_t  target_position;
    uint16_t kp;
    uint16_t kd;
};

static uint8_t parse_mode(int mode_value)
{
    switch (mode_value)
    {
        case 0x00: return MODE_NO_MODE;
        case 0x01: return MODE_PROFILE_POSITION;
        case 0x03: return MODE_PROFILE_VELOCITY;
        case 0x04: return MODE_PROFILE_TORQUE;
        case 0x06: return MODE_HOMING;
        case 0x07: return MODE_INTERPOLATION_POSITION;
        case 0x08: return MODE_CYCLIC_SYNCHRONOUS_POSITION;
        case 0x09: return MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
        case 0x0A: return MODE_CYCLIC_SYNCHRONOUS_TORQUE;
        case 0x0F: return MODE_PROFILE_COMMUTATION;
        default:
            std::fprintf(stderr, "Warning: unknown mode value %d, defaulting to NO_MODE\n", mode_value);
            return MODE_NO_MODE;
    }
}

static std::vector<actuator_cfg_t> load_cfg(const char* yaml_path, int expected_size) {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root.IsSequence()) {
        std::fprintf(stderr, "Config error: top-level must be a sequence\n");
        std::exit(2);
    }
    if (static_cast<int>(root.size()) != expected_size) {
        std::fprintf(stderr, "Config error: YAML entries (%zu) != slave count (%d)\n",
                     root.size(), expected_size);
        std::exit(3);
    }
    std::vector<actuator_cfg_t> v;
    v.reserve(root.size());
    for (size_t i = 0; i < root.size(); ++i) {
        const auto& n = root[i];
        actuator_cfg_t c{};
        c.name            = n["name"]            ? n["name"].as<std::string>() : ("slave_" + std::to_string(i+1));
        c.mode            = n["mode"]            ? parse_mode(n["mode"].as<int>()) : MODE_NO_MODE;
        c.target_torque   = n["target_torque"]   ? n["target_torque"].as<int16_t>()        : 0;
        c.target_velocity = n["target_velocity"] ? n["target_velocity"].as<int32_t>()      : 0;
        c.target_position = n["target_position"] ? n["target_position"].as<int32_t>()      : 0;
        c.kp              = n["kp"]              ? n["kp"].as<uint16_t>()                  : 0;
        c.kd              = n["kd"]              ? n["kd"].as<uint16_t>()                  : 0;
        v.push_back(c);
    }
    return v;
}

// ---------- State machine for the 2-stage cycle ----------
enum class Stage {
    VELOCITY_FWD = 0,
    VELOCITY_REV = 1,
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::puts("Usage: actuator_test <ifname> <config.yaml>\n  e.g., actuator_test eth0 config.yaml");
        return 1;
    }

    std::signal(SIGINT, handle_sigint);

    try {
        const char* ifname = argv[1];
        const char* ypath  = argv[2];

        // 1) Bring up EtherCAT master (auto-discovery, OP)
        EthercatMaster master(ifname);
        const int slave_count = master.slaveCount();
        if (slave_count <= 0) {
            std::fprintf(stderr, "No EtherCAT slaves discovered\n");
            return 2;
        }

        // 2) Load YAML configuration (one item per slave)
        auto cfg = load_cfg(ypath, slave_count);

        // 3) Create one EthercatActuator per slave (PDO map from your headers)
        ActuatorPDOMap map{};
        std::vector<EthercatActuator> acts;
        acts.reserve(slave_count);
        for (int sid = 1; sid <= slave_count; ++sid) {
            acts.emplace_back(sid, map);
        }

        // 4) Allocate commands/feedback
        std::vector<ActuatorCommand>  cmds(slave_count);
        std::vector<ActuatorFeedback> fbs (slave_count);

        // 5) Initialize commands (FaultReset, preload gains)
        for (int k = 0; k < slave_count; ++k) {
            cmds[k].controlword = CW_FAULT_RESET; // clear faults first
            cmds[k].kp          = cfg[k].kp;
            cmds[k].kd          = cfg[k].kd;
        }

        // Per-stage bookkeeping
        Stage stage = Stage::VELOCITY_FWD;
        auto leg_start = std::chrono::steady_clock::now();

        // THERMAL WATCHDOG: per-slave overheat flags
        std::vector<bool> overheated(slave_count, false);

        // Control loop
        while (!g_stop) {
            // 6) Write outputs for all slaves
            for (int k = 0; k < slave_count; ++k) {
                acts[k].writeCommand(cmds[k]);
            }

            // Exchange PD once
            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                bool all_op_enabled = true;

                // Read inputs + CiA-402 bring-up for all
                for (int k = 0; k < slave_count; ++k) {
                    acts[k].readFeedback(fbs[k]);
                    acts[k].advanceCiA402(fbs[k], cmds[k]);

                    // If drive is in OP, keep EO latched (individually)
                    if (fbs[k].status == 0x27) {
                        cmds[k].controlword = CW_ENABLE_OPERATION;
                    }

                    // CHANGE: per-actuator thermal hysteresis
                    if (!overheated[k] && fbs[k].temp >= kTempHighC) {
                        overheated[k] = true;
                        std::printf("\n[THERMAL] Slave %d OVERHEAT (T=%d°C) → DISABLE OP\n", k+1, fbs[k].temp);
                    } else if (overheated[k] && fbs[k].temp <= kTempLowC) {
                        overheated[k] = false;
                        std::printf("\n[THERMAL] Slave %d COOLED (T=%d°C) → re-ENABLE OP\n", k+1, fbs[k].temp);
                    }
                }

                // CHANGE: Issue per-actuator commands — overheated ones are disabled; others continue the stage
                for (int k = 0; k < slave_count; ++k) {
                    if (overheated[k]) {
                        // Force a safe stop on this actuator only
                        cmds[k].controlword = CW_ZERO;
                        cmds[k].mode        = MODE_CYCLIC_SYNCHRONOUS_VELOCITY; // mode value doesn't matter while disabled
                        cmds[k].target_vel  = 0;
                        cmds[k].target_pos  = fbs[k].pos;   // keep current pos if mode flips later
                        cmds[k].target_tor  = 0;
                        continue;
                    }

                    // Actuator is allowed to run → apply stage command
                    switch (stage) {
                        case Stage::VELOCITY_FWD:
                            cmds[k].mode       = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                            cmds[k].target_vel = cfg[k].target_velocity;          // forward (+)
                            cmds[k].kp = cfg[k].kp; cmds[k].kd = cfg[k].kd;
                            break;
                        case Stage::VELOCITY_REV:
                            cmds[k].mode       = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                            cmds[k].target_vel = (cfg[k].target_velocity > 0)
                                                   ? -cfg[k].target_velocity
                                                   : cfg[k].target_velocity;      // if already ≤ 0
                            cmds[k].kp = cfg[k].kp; cmds[k].kd = cfg[k].kd;
                            break;
                    }
                }

                // CHANGE: Stage timing continues regardless of any individual overheat
                if (std::chrono::steady_clock::now() - leg_start >= kLegDuration) {
                    stage = (stage == Stage::VELOCITY_FWD) ? Stage::VELOCITY_REV : Stage::VELOCITY_FWD;
                    leg_start = std::chrono::steady_clock::now();
                }

                // Debug line: show temp and overheat flag per actuator
                std::printf("STG=%d WKC=%d |", static_cast<int>(stage), wkc);
                for (int k = 0; k < slave_count; ++k) {
                    std::printf(" S%d: sw=0x%02X vel=%d pos=%d T=%d%s",
                                k+1, fbs[k].status, fbs[k].vel, fbs[k].pos, fbs[k].temp,
                                overheated[k] ? "(!)" : "");
                    if (k != slave_count - 1) std::printf(" |");
                }
                std::printf(" \r");
                std::fflush(stdout);
            }

            std::this_thread::sleep_for(kControlPeriod);
        }

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }

    std::puts("\nStopped.");
    return 0;
}
