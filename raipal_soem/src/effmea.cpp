// /src/effmea.cpp

#include <cstdio>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

// ================= TCP 설정 =================
static constexpr const char* PY_HOST = "127.0.0.1";
static constexpr int PY_PORT = 8080;
static constexpr int TEMP_HIGH_C = 70;
static constexpr int TEMP_LOW_C  = 60;
static constexpr size_t RX_PACKET_SIZE = 16;
static constexpr size_t TX_PACKET_SIZE = 29;

enum TelemetryStateCode : uint8_t {
    STATE_RUNNING        = 0,
    STATE_THERMAL_PAUSED = 1,
    STATE_DRIVE_FAULT    = 2,
    STATE_DRIVE_ERROR    = 3,
};

enum TelemetryErrorCode : uint8_t {
    ERR_OK                 = 0,
    ERR_ACTUATOR_FAULT     = 1,
    ERR_LOAD_FAULT         = 2,
    ERR_ACTUATOR_DRIVE_ERR = 3,
    ERR_LOAD_DRIVE_ERR     = 4,
    ERR_THERMAL_PAUSED     = 5,
};

// ================= mode parser =================
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
        default:   return MODE_NO_MODE;
    }
}

// ================= MAIN =================
int main(int argc, char** argv)
{
    if (argc < 2) {
        std::puts("Usage: effmea <ifname>");
        return 1;
    }

    try {

        // ================= TCP CLIENT =================
        int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_fd < 0) {
            perror("socket");
            return 1;
        }

        sockaddr_in serv{};
        serv.sin_family = AF_INET;
        serv.sin_port   = htons(PY_PORT);
        inet_pton(AF_INET, PY_HOST, &serv.sin_addr);

        if (connect(sock_fd, (sockaddr*)&serv, sizeof(serv)) < 0) {
            perror("connect");
            return 1;
        }

        fcntl(sock_fd, F_SETFL, O_NONBLOCK);
        std::cout << "[TCP] Connected to Python\n";

        // ================= EtherCAT 2개 초기화 =================
        const char* ifname = argv[1];

        EthercatMaster master(ifname);

        if (master.slaveCount() < 2) {
            std::fprintf(stderr, "Need at least 2 EtherCAT slaves\n");
            return 2;
        }

        ActuatorPDOMap map{};

        EthercatActuator act1(1, map);  // actuator
        EthercatActuator act2(2, map);  // load

        ActuatorCommand  cmd1{}, cmd2{};
        ActuatorFeedback fb1{}, fb2{};

        cmd1.controlword = CW_FAULT_RESET;
        cmd1.mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

        cmd2.controlword = CW_FAULT_RESET;
        cmd2.mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

        const auto period = std::chrono::microseconds(5000);

        std::vector<uint8_t> rxbuffer;
        bool thermal_paused = false;

        while (true)
        {
            auto loop_start = std::chrono::steady_clock::now();

            // ================= Python → C++ 수신 =================
            uint8_t tmp[32];
            ssize_t n = recv(sock_fd, tmp, sizeof(tmp), MSG_DONTWAIT);
            if (n > 0)
                rxbuffer.insert(rxbuffer.end(), tmp, tmp + n);

            while (rxbuffer.size() >= RX_PACKET_SIZE)
            {
                int16_t a_tor_net, a_mode_net;
                int32_t a_vel_net;

                int16_t l_tor_net, l_mode_net;
                int32_t l_vel_net;

                std::memcpy(&a_tor_net,  &rxbuffer[0], 2);
                std::memcpy(&a_vel_net,  &rxbuffer[2], 4);
                std::memcpy(&a_mode_net, &rxbuffer[6], 2);

                std::memcpy(&l_tor_net,  &rxbuffer[8], 2);
                std::memcpy(&l_vel_net,  &rxbuffer[10], 4);
                std::memcpy(&l_mode_net, &rxbuffer[14], 2);

                rxbuffer.erase(rxbuffer.begin(), rxbuffer.begin() + RX_PACKET_SIZE);

                cmd1.target_tor = ntohs(a_tor_net);
                cmd1.target_vel = ntohl(a_vel_net);
                cmd1.mode       = parse_mode(ntohs(a_mode_net));

                cmd2.target_tor = ntohs(l_tor_net);
                cmd2.target_vel = ntohl(l_vel_net);
                cmd2.mode       = parse_mode(ntohs(l_mode_net));
            }

            if (thermal_paused) {
                // Keep drives enabled but force zero command while overheated.
                cmd1.mode = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                cmd2.mode = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                cmd1.target_tor = 0;
                cmd2.target_tor = 0;
                cmd1.target_vel = 0;
                cmd2.target_vel = 0;
            }

            // ================= EtherCAT Write =================
            act1.writeCommand(cmd1);
            act2.writeCommand(cmd2);

            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC())
            {
                act1.readFeedback(fb1);
                act2.readFeedback(fb2);

                if (!thermal_paused && (fb1.temp >= TEMP_HIGH_C || fb2.temp >= TEMP_HIGH_C)) {
                    thermal_paused = true;
                } else if (thermal_paused && (fb1.temp <= TEMP_LOW_C && fb2.temp <= TEMP_LOW_C)) {
                    thermal_paused = false;
                }

                act1.advanceCiA402(fb1, cmd1);
                act2.advanceCiA402(fb2, cmd2);

                if (fb1.status == 0x27)
                    cmd1.controlword = CW_ENABLE_OPERATION;

                if (fb2.status == 0x27)
                    cmd2.controlword = CW_ENABLE_OPERATION;

                // ================= C++ → Python 송신 =================
                uint8_t txbuf[TX_PACKET_SIZE];

                txbuf[0] = fb1.status;
                txbuf[1] = fb1.error;
                txbuf[2] = fb1.temp;

                int16_t tor1 = htons(fb1.tor);
                int32_t vel1 = htonl(fb1.vel);
                int32_t pos1 = htonl(fb1.pos);

                std::memcpy(&txbuf[3],  &tor1, 2);
                std::memcpy(&txbuf[5],  &vel1, 4);
                std::memcpy(&txbuf[9],  &pos1, 4);

                txbuf[13] = fb2.status;
                txbuf[14] = fb2.error;
                txbuf[15] = fb2.temp;

                int16_t tor2 = htons(fb2.tor);
                int32_t vel2 = htonl(fb2.vel);
                int32_t pos2 = htonl(fb2.pos);

                std::memcpy(&txbuf[16], &tor2, 2);
                std::memcpy(&txbuf[18], &vel2, 4);
                std::memcpy(&txbuf[22], &pos2, 4);
                txbuf[26] = thermal_paused ? 1 : 0;
                uint8_t error_code = ERR_OK;
                if (fb1.status == 0x08) {
                    error_code = ERR_ACTUATOR_FAULT;
                } else if (fb2.status == 0x08) {
                    error_code = ERR_LOAD_FAULT;
                } else if (fb1.error != 0) {
                    error_code = ERR_ACTUATOR_DRIVE_ERR;
                } else if (fb2.error != 0) {
                    error_code = ERR_LOAD_DRIVE_ERR;
                } else if (thermal_paused) {
                    error_code = ERR_THERMAL_PAUSED;
                }

                uint8_t state_code = STATE_RUNNING;
                if (error_code == ERR_ACTUATOR_FAULT || error_code == ERR_LOAD_FAULT) {
                    state_code = STATE_DRIVE_FAULT;
                } else if (error_code == ERR_ACTUATOR_DRIVE_ERR || error_code == ERR_LOAD_DRIVE_ERR) {
                    state_code = STATE_DRIVE_ERROR;
                } else if (thermal_paused) {
                    state_code = STATE_THERMAL_PAUSED;
                }

                txbuf[27] = state_code;
                txbuf[28] = error_code;

                send(sock_fd, txbuf, TX_PACKET_SIZE, MSG_DONTWAIT);

                std::printf("A:T=%u tor=%d vel=%d pos=%d | L:T=%u tor=%d vel=%d pos=%d | TH=%d ST=%u ER=%u\n",
                            fb1.temp, fb1.tor, fb1.vel, fb1.pos,
                            fb2.temp, fb2.tor, fb2.vel, fb2.pos,
                            thermal_paused ? 1 : 0,
                            state_code, error_code);
                std::fflush(stdout);
            }

            std::this_thread::sleep_until(loop_start + period);
        }

        close(sock_fd);
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }

    return 0;
}
