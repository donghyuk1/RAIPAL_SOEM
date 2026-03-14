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

        EthercatActuator act1(2, map);  // actuator
        EthercatActuator act2(1, map);  // load

        ActuatorCommand  cmd1{}, cmd2{};
        ActuatorFeedback fb1{}, fb2{};

        cmd1.controlword = CW_FAULT_RESET;
        cmd1.mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

        cmd2.controlword = CW_FAULT_RESET;
        cmd2.mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

        const auto period = std::chrono::microseconds(5000);

        std::vector<uint8_t> rxbuffer;

        while (true)
        {
            auto loop_start = std::chrono::steady_clock::now();

            // ================= Python → C++ 수신 =================
            uint8_t tmp[32];
            ssize_t n = recv(sock_fd, tmp, sizeof(tmp), MSG_DONTWAIT);
            if (n > 0)
                rxbuffer.insert(rxbuffer.end(), tmp, tmp + n);

            while (rxbuffer.size() >= 16)
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

                rxbuffer.erase(rxbuffer.begin(), rxbuffer.begin() + 16);

                cmd1.target_tor = ntohs(a_tor_net);
                cmd1.target_vel = ntohl(a_vel_net);
                cmd1.mode       = parse_mode(ntohs(a_mode_net));

                cmd2.target_tor = ntohs(l_tor_net);
                cmd2.target_vel = ntohl(l_vel_net);
                cmd2.mode       = parse_mode(ntohs(l_mode_net));
            }

            // ================= EtherCAT Write =================
            act1.writeCommand(cmd1);
            act2.writeCommand(cmd2);

            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC())
            {
                act1.readFeedback(fb1);
                act2.readFeedback(fb2);

                act1.advanceCiA402(fb1, cmd1);
                act2.advanceCiA402(fb2, cmd2);

                if (fb1.status == 0x27)
                    cmd1.controlword = CW_ENABLE_OPERATION;

                if (fb2.status == 0x27)
                    cmd2.controlword = CW_ENABLE_OPERATION;

                // ================= C++ → Python 송신 =================
                uint8_t txbuf[24];

                txbuf[0] = fb1.status;
                txbuf[1] = fb1.error;

                int16_t tor1 = htons(fb1.tor);
                int32_t vel1 = htonl(fb1.vel);
                int32_t pos1 = htonl(fb1.pos);

                std::memcpy(&txbuf[2],  &tor1, 2);
                std::memcpy(&txbuf[4],  &vel1, 4);
                std::memcpy(&txbuf[8],  &pos1, 4);

                txbuf[12] = fb2.status;
                txbuf[13] = fb2.error;

                int16_t tor2 = htons(fb2.tor);
                int32_t vel2 = htonl(fb2.vel);
                int32_t pos2 = htonl(fb2.pos);

                std::memcpy(&txbuf[14], &tor2, 2);
                std::memcpy(&txbuf[16], &vel2, 4);
                std::memcpy(&txbuf[20], &pos2, 4);

                send(sock_fd, txbuf, 24, MSG_DONTWAIT);

                std::printf("A: tor=%d vel=%d pos=%d | L: tor=%d vel=%d pos=%d\n",
                            fb1.tor, fb1.vel, fb1.pos,
                            fb2.tor, fb2.vel, fb2.pos);
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