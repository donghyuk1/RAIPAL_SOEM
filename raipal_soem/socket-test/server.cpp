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

int main() {
    ::signal(SIGPIPE, SIG_IGN);

    const int PORT = 8080;
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
            while ((pos = pending.find('\n')) != std::string::npos) {
                std::string line = pending.substr(0, pos);
                // drop optional '\r'
                if (!line.empty() && line.back() == '\r') line.pop_back();
                std::cout << "[server] received: " << line << std::endl;
                pending.erase(0, pos + 1);
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