//
// Created by dongghk on 25. 9. 4.
//
// src/main.cpp
#include <cstdio>
#include <thread>
#include <chrono>
#include <vector>               // UPDATE: vector for N actuators
#include <string>               // UPDATE: yaml path handling

#include <yaml-cpp/yaml.h>      // UPDATE: YAML parsing (link with yaml-cpp)

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"



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

// UPDATE: one YAML item per slave (1..N)
struct actuator_cfg_t {
    std::string name;
    uint8_t  mode;
    int16_t  target_torque;
    int32_t  target_velocity;
    int32_t  target_position;
    uint16_t kp;
    uint16_t kd;
};

// UPDATE: load YAML, assert size == slave_count
static std::vector<actuator_cfg_t> load_cfg(const char* yaml_path, int expected_size) {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node actuators_node = root["actuators"];
    if (!actuators_node.IsSequence()) {
        std::fprintf(stderr, "Config error: 'actuators' key must be a sequence\n");
        std::exit(2);
    }
    if (static_cast<int>(actuators_node.size()) != expected_size) {
        std::fprintf(stderr, "Config error: YAML entries in 'actuators' (%zu) != slave count (%d)\n",
                     actuators_node.size(), expected_size);
        std::exit(3);
    }
    std::vector<actuator_cfg_t> v;
    v.reserve(actuators_node.size());
    for (size_t i = 0; i < actuators_node.size(); ++i) {
        const auto& n = actuators_node[i];
        actuator_cfg_t c{};
        c.name            = n["name"]            ? n["name"].as<std::string>() : ("slave_" + std::to_string(i+1));
        // c.mode            = n["mode"]            ? parse_mode(n["mode"].as<std::string>()) : CST;
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

int main(int argc, char** argv) {
    // UPDATE: require YAML path
    if (argc < 3) {
        std::puts("Usage: demo <ifname> <config.yaml>\n  e.g., demo eth0 config.yaml");
        return 1;
    }

    try {
        const char* ifname = argv[1];            // same naming
        const char* ypath  = argv[2];            // UPDATE: yaml path

        EthercatMaster master(ifname);           // auto-discovers & enters OP (your Method B)
        const int slave_count = master.slaveCount(); // UPDATE: use discovered N
        if (slave_count <= 0) {
            std::fprintf(stderr, "No EtherCAT slaves discovered\n");
            return 2;
        }

        // UPDATE: load YAML config (exactly one entry per slave)
        auto cfg = load_cfg(ypath, slave_count);

        // One actuator per slave with your PDO layout (same type names)
        ActuatorPDOMap map{}; // uses defaults from header
        std::vector<EthercatActuator> acts;
        acts.reserve(slave_count);
        for (int sid = 1; sid <= slave_count; ++sid) {
            acts.emplace_back(sid, map);
        }

        // Commands/feedback arrays (keeps your naming style)
        std::vector<ActuatorCommand>  cmds(slave_count);
        std::vector<ActuatorFeedback> fbs (slave_count);

        // UPDATE: initialize commands from YAML (FaultReset first)
        for (int k = 0; k < slave_count; ++k) {
            cmds[k].controlword = CW_FAULT_RESET;
            cmds[k].mode        = cfg[k].mode;           // pre-set desired mode
            cmds[k].target_tor  = cfg[k].target_torque;  // pre-load setpoints
            cmds[k].target_vel  = cfg[k].target_velocity;
            cmds[k].target_pos  = cfg[k].target_position;
            cmds[k].kp          = cfg[k].kp;             // impedance gains if mapped
            cmds[k].kd          = cfg[k].kd;
        }

        const auto period = std::chrono::microseconds(5000); // 5 ms (same)

        for (int i = 0; i < 10000; ++i) {
            // Write desired commands into PDO for all slaves
            for (int k = 0; k < slave_count; ++k) {
                acts[k].writeCommand(cmds[k]);
            }

            // Exchange once for whole bus
            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                // Read feedback & advance CiA-402 per slave
                for (int k = 0; k < slave_count; ++k) {
                    acts[k].readFeedback(fbs[k]);
                    acts[k].advanceCiA402(fbs[k], cmds[k]); // keep your helper

                    // When each drive reaches OP-ENABLED (0x27 low byte),
                    // keep EO and stream the proper setpoint for its mode
                    if (fbs[k].status == 0x27) {
                        cmds[k].controlword = CW_ENABLE_OPERATION;
                        cmds[k].mode        = cfg[k].mode;


                        switch (cfg[k].mode) {
                            case MODE_CYCLIC_SYNCHRONOUS_TORQUE:
                                cmds[k].target_tor = cfg[k].target_torque; break;
                            case MODE_CYCLIC_SYNCHRONOUS_VELOCITY:
                                cmds[k].target_vel = cfg[k].target_velocity; break;
                            case MODE_CYCLIC_SYNCHRONOUS_POSITION:
                                cmds[k].target_pos = cfg[k].target_position; break;
                            case MODE_PROFILE_TORQUE:
                                cmds[k].target_tor = cfg[k].target_torque; break;
                            case MODE_PROFILE_VELOCITY:
                                cmds[k].target_vel = cfg[k].target_velocity; break;
                            case MODE_PROFILE_POSITION:
                                cmds[k].target_pos = cfg[k].target_position; break;
                            case MODE_HOMING: // (Optional) vendor-specific homing handling here
                                break;
                            default:
                                // For NO_MODE or unrecognized modes, do nothing
                                    break;
                        }
                        // keep gains refreshed if your PDO maps them
                        cmds[k].kp = cfg[k].kp;
                        cmds[k].kd = cfg[k].kd;
                    }
                }

                // Optional: debug print (compact, similar style)
                std::printf("WKC=%d |", wkc);
                for (int k = 0; k < slave_count; ++k) {
                    std::printf(" S%d: sw=0x%02X tor=%d pos=%d vel=%d temp=%d",
                                k+1, fbs[k].status, fbs[k].tor, fbs[k].pos, fbs[k].vel, fbs[k].temp);
                    if (k != slave_count - 1) std::printf(" |");
                }
                std::printf(" \r");
                std::fflush(stdout);
            }

            std::this_thread::sleep_for(period);
        }

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }
    std::puts("\nDone.");
    return 0;
}

