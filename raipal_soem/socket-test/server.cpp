//
// Created by dongg on 25. 11. 5..
//
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>

#include <cstring>
#include <iostream>
#include <string>

#include <cstdint>
#include <cmath>
#include <tuple>

std::tuple<double, uint16_t, uint16_t> parse_frame(const uint8_t frame[6], int torque_decimal)
{
    // Combine D1, D2 → torque_raw
    uint16_t torque_raw_val = (frame[0] << 8) | frame[1];
    double torque_raw = torque_raw_val * std::pow(0.1, torque_decimal);
    torque_raw = std::round(torque_raw * std::pow(10, torque_decimal)) / std::pow(10, torque_decimal);

    // Combine D3, D4 → speed_raw (lowest 15 bits only)
    uint16_t speed_raw = ((frame[2] & 0x7F) << 8) | frame[3];

    // CRC from D5, D6
    uint16_t crc = (frame[4] << 8) | frame[5];

    // Check torque sign (D3 MSB)
    bool torque_sign = frame[2] & 0x80;
    double torque = torque_sign ? -torque_raw : torque_raw;

    return std::make_tuple(torque, speed_raw, crc);
}

int main() {
    ::signal(SIGPIPE, SIG_IGN);

    const int PORT = 8080;
    const size_t FRAME_SIZE = 6;
    int torque_decimal = 1;
    
    int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { std::perror("socket"); return 1; }

    int opt = 1;
    if (::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
        std::perror("setsockopt"); return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;   // 0.0.0.0
    addr.sin_port = htons(PORT);

    if (::bind(listen_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::perror("bind"); return 1;
    }
    if (::listen(listen_fd, 1) < 0) {
        std::perror("listen"); return 1;
    }

    std::cout << "C++ server listening on 0.0.0.0:" << PORT << std::endl;

    sockaddr_in cli{};
    socklen_t clen = sizeof(cli);
    int conn_fd = ::accept(listen_fd, (sockaddr*)&cli, &clen);
    if (conn_fd < 0) { std::perror("accept"); return 1; }

    char cli_ip[INET_ADDRSTRLEN]{};
    ::inet_ntop(AF_INET, &cli.sin_addr, cli_ip, sizeof(cli_ip));
    std::cout << "Client connected from " << cli_ip << ":" << ntohs(cli.sin_port) << "\n";

    std::string pending;
    char buf[4096];

    // Read bytes, split by '\n', print each complete line
    while (true) {
        ssize_t n = ::recv(conn_fd, buf, sizeof(buf), 0);
        if (n > 0) {
            pending.append(buf, buf + n);
            std::size_t pos;
            // Instead of '\n'-delimited, accumulate until FRAME_SIZE bytes in 'pending'
			while (pending.size() >= FRAME_SIZE) {
				uint8_t frame[FRAME_SIZE];
                std::memcpy(frame, pending.data(), FRAME_SIZE);
                pending.erase(pending.begin(), pending.begin() + FRAME_SIZE);
                
				std::cout << "[server] received: ";
				for (unsigned char c : frame) {
					printf("%02X ", c);
				}
				std::cout << std::endl;
				pending.erase(0, FRAME_SIZE);
				auto [torque, speed, crc] = parse_frame(frame, torque_decimal);

				std::cout << "Torque: " << torque << " Nm\n";
				std::cout << "Speed: " << speed << " RPM\n";
				std::cout << "CRC: 0x" << std::hex << crc << std::dec << "\n";
			}
        } else if (n == 0) {
            std::cout << "Client disconnected.\n";
            break;
        } else {
            std::perror("recv");
            break;
        }
    }

    ::close(conn_fd);
    ::close(listen_fd);
    return 0;
}
