//
// Created by dongghk on 25. 9. 4.
//
// src/main.cpp
#include <cstdio>
#include <thread>
#include <chrono>
#include <vector>               // UPDATE: vector for N actuators
#include <string>               // UPDATE: yaml path handling
#include <iostream>

#include <yaml-cpp/yaml.h>      // UPDATE: YAML parsing (link with yaml-cpp)

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>


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
    if (argc < 3) {
        std::puts("Usage: demo <ifname> <config.yaml>");
        return 1;
    }

    try {
        const char* ifname = argv[1];
        const char* ypath  = argv[2];

        // ---- TCP CLIENT ----
		int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (sock_fd < 0) {
			perror("socket");
			return 1;
		}

		sockaddr_in serv{};
		serv.sin_family = AF_INET;
		serv.sin_port   = htons(8081);  // <-- C++ port
		inet_pton(AF_INET, "127.0.0.1", &serv.sin_addr);

		if (connect(sock_fd, (sockaddr*)&serv, sizeof(serv)) < 0) {
			perror("connect");
			return 1;
		}

		std::cout << "[TCP] Connected to server (C++ client)\n";
		// ---------------------

        EthercatMaster master(ifname);
        const int slave_count = master.slaveCount();
        if (slave_count <= 0) {
            std::fprintf(stderr, "No EtherCAT slaves found\n");
            return 2;
        }

        auto cfg = load_cfg(ypath, slave_count);

        ActuatorPDOMap map{};
        std::vector<EthercatActuator> acts;
        acts.reserve(slave_count);
        for (int sid = 1; sid <= slave_count; ++sid)
            acts.emplace_back(sid, map);

        std::vector<ActuatorCommand>  cmds(slave_count);
        std::vector<ActuatorFeedback> fbs (slave_count);

        for (int k = 0; k < slave_count; ++k) {
            cmds[k].controlword = CW_FAULT_RESET;
            cmds[k].mode        = cfg[k].mode;
            cmds[k].target_tor  = cfg[k].target_torque;
            cmds[k].target_vel  = cfg[k].target_velocity;
            cmds[k].target_pos  = cfg[k].target_position;
            cmds[k].kp          = cfg[k].kp;
            cmds[k].kd          = cfg[k].kd;
        }

        const auto period = std::chrono::microseconds(5000);

        for (int i = 0; i < 10000; ++i) {

            for (int k = 0; k < slave_count; ++k)
                acts[k].writeCommand(cmds[k]);

            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                for (int k = 0; k < slave_count; ++k) {
                    acts[k].readFeedback(fbs[k]);
                    acts[k].advanceCiA402(fbs[k], cmds[k]);

                    if (fbs[k].status == 0x27) {
                        cmds[k].controlword = CW_ENABLE_OPERATION;
                        cmds[k].mode        = cfg[k].mode;

                        switch (cfg[k].mode) {
                            case MODE_CYCLIC_SYNCHRONOUS_TORQUE:
                            case MODE_PROFILE_TORQUE:
                                cmds[k].target_tor = cfg[k].target_torque;
                                break;
                            case MODE_CYCLIC_SYNCHRONOUS_VELOCITY:
                            case MODE_PROFILE_VELOCITY:
                                cmds[k].target_vel = cfg[k].target_velocity;
                                break;
                            case MODE_CYCLIC_SYNCHRONOUS_POSITION:
                            case MODE_PROFILE_POSITION:
                                cmds[k].target_pos = cfg[k].target_position;
                                break;
                        }

                        cmds[k].kp = cfg[k].kp;
                        cmds[k].kd = cfg[k].kd;
                    }
                }

                // SEND TARGET TORQUE TO TCP SERVER
                for (int k = 0; k < slave_count; ++k) {
					int16_t t = htons(fbs[k].tor);
					send(sock_fd, &t, sizeof(t), MSG_DONTWAIT);
				}

                // debug print
                std::printf("WKC=%d\r", wkc);
                std::fflush(stdout);
            }

            std::this_thread::sleep_for(period);
        }

        close(sock_fd);  // <-- close TCP socket

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }

    std::puts("\nDone.");
    return 0;
}
