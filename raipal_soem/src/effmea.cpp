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

        // non-blocking 설정
        fcntl(sock_fd, F_SETFL, O_NONBLOCK);

        std::cout << "[TCP] Connected to Python\n";

        // ================= EtherCAT 초기화 =================
        const char* ifname = argv[1];

        EthercatMaster master(ifname);
        const int slave_count = master.slaveCount();
        if (slave_count <= 0)
            return 2;

        ActuatorPDOMap map{};
        std::vector<EthercatActuator> acts;
        acts.reserve(slave_count);
        for (int sid = 1; sid <= slave_count; ++sid)
            acts.emplace_back(sid, map);

        std::vector<ActuatorCommand>  cmds(slave_count);
        std::vector<ActuatorFeedback> fbs (slave_count);

        for (int k = 0; k < slave_count; ++k) {
            cmds[k].controlword = CW_FAULT_RESET;
            cmds[k].mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
            cmds[k].target_tor  = 0;
            cmds[k].target_vel  = 0;
            cmds[k].target_pos  = 0;
            cmds[k].kp          = 0;
            cmds[k].kd          = 0;
        }

        const auto period = std::chrono::microseconds(5000);

        // ================= Main Loop =================
        while (true)
        {
            auto loop_start = std::chrono::steady_clock::now();

            // ---- Python → C++ 명령 수신 (non-blocking)
            uint8_t rxbuf[8];
            ssize_t n = recv(sock_fd, rxbuf, sizeof(rxbuf), MSG_DONTWAIT);
            if (n == 8) {
                int16_t tor_net;
                int32_t vel_net;
                int16_t mode_net;

                std::memcpy(&tor_net,  rxbuf,     2);
                std::memcpy(&vel_net,  rxbuf + 2, 4);
                std::memcpy(&mode_net, rxbuf + 6, 2);

                int16_t new_tor = ntohs(tor_net);
                int32_t new_vel = ntohl(vel_net);
                int16_t new_mode= ntohs(mode_net);

                for (int k = 0; k < slave_count; ++k) {
                    cmds[k].target_tor = new_tor;
                    cmds[k].target_vel = new_vel;
                    cmds[k].mode       = new_mode;
                }
            }

            // ---- EtherCAT Write
            for (int k = 0; k < slave_count; ++k)
                acts[k].writeCommand(cmds[k]);

            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC())
            {
                for (int k = 0; k < slave_count; ++k)
                {
                    acts[k].readFeedback(fbs[k]);
                    acts[k].advanceCiA402(fbs[k], cmds[k]);

                    if (fbs[k].status == 0x27)
                        cmds[k].controlword = CW_ENABLE_OPERATION;
                }

                // ---- C++ → Python 피드백 송신 (slave 1 기준)
                if (slave_count > 0)
                {
                    uint8_t txbuf[12];

					uint8_t  status = fbs[0].status;
					uint8_t  error  = fbs[0].error;
					int16_t  tor    = htons(fbs[0].tor);
					int32_t  vel    = htonl(fbs[0].vel);
					int32_t  pos    = htonl(fbs[0].pos);

					txbuf[0] = status;
					txbuf[1] = error;

					std::memcpy(txbuf + 2, &tor, 2);
					std::memcpy(txbuf + 4, &vel, 4);
					std::memcpy(txbuf + 8, &pos, 4);

					send(sock_fd, txbuf, sizeof(txbuf), MSG_DONTWAIT);
                }

                std::printf("WKC=%d\r", wkc);
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
