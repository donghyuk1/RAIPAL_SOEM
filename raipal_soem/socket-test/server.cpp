#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <mutex>

static const int PY_PORT  = 8080;
static const int CPP_PORT = 8081;

// ======================= CSV LOGGER ===========================
std::ofstream csv_file("data_log.csv");
std::mutex csv_mutex;
auto start_time = std::chrono::steady_clock::now();

double now_ms()
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(now - start_time).count();
}

void log_target(int16_t target)
{
    std::lock_guard<std::mutex> lock(csv_mutex);
    csv_file << std::fixed << std::setprecision(3)
             << now_ms() << "," << target << "," << "" << "\n";
    csv_file.flush();
}

void log_measured(double measured)
{
    std::lock_guard<std::mutex> lock(csv_mutex);
    csv_file << std::fixed << std::setprecision(3)
             << now_ms() << "," << "" << "," << measured << "\n";
    csv_file.flush();
}
// =============================================================

// -------------------- PYTHON CLIENT HANDLER --------------------
void python_client_thread(int fd) {
    std::cout << "[PY] client connected.\n";

    std::vector<uint8_t> pending;
    pending.reserve(1024);

    uint8_t buf[1024];

    while (true) {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);

        if (n <= 0) {
            std::cout << "[PY] client disconnected.\n";
            break;
        }

        pending.insert(pending.end(), buf, buf + n);

        while (pending.size() >= 6) {
            uint8_t frame[6];
            memcpy(frame, pending.data(), 6);
            pending.erase(pending.begin(), pending.begin() + 6);

            uint16_t torque_raw = (frame[0] << 8) | frame[1];
            double measured_torque = (int16_t)torque_raw;  // You can scale if needed

            std::cout << "[PY] measured_torque = " << measured_torque << "\n";

            log_measured(measured_torque);
        }
    }

    close(fd);
}

// -------------------- C++ CLIENT HANDLER --------------------
void cpp_client_thread(int fd) {
    std::cout << "[CPP] client connected.\n";

    uint8_t buf[2];

    while (true) {
        ssize_t n = recv(fd, buf, 2, 0);

        if (n <= 0) {
            std::cout << "[CPP] client disconnected.\n";
            break;
        }
        if (n < 2) continue;  // partial packet → wait

        int16_t net;
        memcpy(&net, buf, 2);
        int16_t torque = ntohs(net);

        std::cout << "[CPP] target_torque = " << torque << "\n";

        log_target(torque);
    }

    close(fd);
}

// -------------------- MAKE LISTEN SOCKET --------------------
int make_listen_socket(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket");
        exit(1);
    }

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port);

    if (bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        exit(1);
    }

    if (listen(fd, 5) < 0) {
        perror("listen");
        exit(1);
    }

    return fd;
}

// -------------------- MAIN SERVER LOOP --------------------
int main() {

    // Create CSV header
    {
        std::lock_guard<std::mutex> lock(csv_mutex);
        csv_file << "time_ms,target_torque,measured_torque\n";
    }

    int py_fd  = make_listen_socket(PY_PORT);
    int cpp_fd = make_listen_socket(CPP_PORT);

    std::cout << "Server running.\n";
    std::cout << " - Python clients on port " << PY_PORT << "\n";
    std::cout << " - C++ clients on   port " << CPP_PORT << "\n";

    while (true) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(py_fd,  &fds);
        FD_SET(cpp_fd, &fds);

        int maxfd = std::max(py_fd, cpp_fd);

        if (select(maxfd + 1, &fds, nullptr, nullptr, nullptr) < 0) {
            perror("select");
            continue;
        }

        if (FD_ISSET(py_fd, &fds)) {
            sockaddr_in cli{};
            socklen_t clen = sizeof(cli);
            int cfd = accept(py_fd, (sockaddr*)&cli, &clen);
            if (cfd >= 0)
                std::thread(python_client_thread, cfd).detach();
        }

        if (FD_ISSET(cpp_fd, &fds)) {
            sockaddr_in cli{};
            socklen_t clen = sizeof(cli);
            int cfd = accept(cpp_fd, (sockaddr*)&cli, &clen);
            if (cfd >= 0)
                std::thread(cpp_client_thread, cfd).detach();
        }
    }

    close(py_fd);
    close(cpp_fd);
    return 0;
}

