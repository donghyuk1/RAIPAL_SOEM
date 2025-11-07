//
// Created by dongg on 25. 11. 7..
//
#include <cstdio>
#include <thread>
#include <chrono>
#include <csignal>
#include <limits>

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

int main(int argc, char** argv) {
    if (argc < 2) {
        std::puts("Usage: actuator_vibrate_one <ifname>\n  e.g., actuator_vibrate_one eth0");
        return 1;
    }
    std::signal(SIGINT, handle_sigint);

    try {
        const char* ifname = argv[1];

        // 1) Bring up EtherCAT master
        EthercatMaster master(ifname);
        const int slave_count = master.slaveCount();
        if (slave_count < 1) {
            std::fprintf(stderr, "No EtherCAT slaves discovered\n");
            return 2;
        }

        // 2) Single actuator (slave #1)
        constexpr int kSlaveId = 1;
        ActuatorPDOMap   map{};
        EthercatActuator act(kSlaveId, map);
        ActuatorCommand  cmd{};
        ActuatorFeedback fb{};

        // 3) Initialize
        cmd.controlword = CW_FAULT_RESET;
        cmd.mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;  // CST mode
        cmd.target_tor  = 0;

        // 4) Vibration parameters
        constexpr int16_t TORQUE_MAG = 20;                  // ±20
        const auto period  = std::chrono::microseconds(500); // loop in every 0.5ms
        const auto flip_dt = std::chrono::milliseconds(100); // every 0.01 s
        auto last_flip     = std::chrono::steady_clock::now();
        int16_t tor        = TORQUE_MAG;

        // 5) Min/max tracking
        bool have_minmax   = false;
        int32_t pos_min    = std::numeric_limits<int32_t>::max();
        int32_t pos_max    = std::numeric_limits<int32_t>::min();

        while (!g_stop) {
            act.writeCommand(cmd);
            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                act.readFeedback(fb);
                act.advanceCiA402(fb, cmd);

                if (fb.status == 0x27) { // OP-ENABLED
                    cmd.controlword = CW_ENABLE_OPERATION;
                    cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

                    if (!have_minmax) {
                        pos_min = pos_max = fb.pos;
                        have_minmax = true;
                    } else {
                        if (fb.pos < pos_min) pos_min = fb.pos;
                        if (fb.pos > pos_max) pos_max = fb.pos;
                    }
                }

                // Torque flip
                auto now = std::chrono::steady_clock::now();
                if (now - last_flip >= flip_dt) {
                    tor = (tor > 0) ? -TORQUE_MAG : TORQUE_MAG;
                    last_flip = now;

                    if (have_minmax) {
                        long long delta = static_cast<long long>(pos_max) - static_cast<long long>(pos_min);
                        // std::printf(
                        //     "\n[flip] tor=%+d | pos_min=%d pos_max=%d | delta=%lld counts\n",
                        //     tor, pos_min, pos_max, delta
                        // );
                    }
                }
                cmd.target_tor = tor;

                long long delta_live = have_minmax
                    ? static_cast<long long>(pos_max) - static_cast<long long>(pos_min)
                    : 0;
                std::printf("WKC=%d | tor=%+d | pos=%d vel=%d temp=%d | Δ=%lld\r",
                            wkc, tor, fb.pos, fb.vel, fb.temp, delta_live);
                std::fflush(stdout);
            }

            std::this_thread::sleep_for(period);
        }

        // Graceful stop
        cmd.target_tor  = 0;
        cmd.controlword = CW_DISABLE_OPERATION;
        act.writeCommand(cmd);
        master.tickOnce();

        if (have_minmax) {
            long long delta = static_cast<long long>(pos_max) - static_cast<long long>(pos_min);
            std::printf("\nFinal range: pos_min=%d pos_max=%d → Δ=%lld counts\n",
                        pos_min, pos_max, delta);
        }

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }

    std::puts("\nStopped.");
    return 0;
}