//
// Created by dongghk on 25. 9. 4.
//

// src/main.cpp
#include <cstdio>
#include <thread>
#include <chrono>
#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::puts("Usage: demo <ifname>   e.g., demo eth0");
        return 1;
    }
    try {
        EthercatMaster master(argv[1]);

        // One actuator on slave #1 with your PDO layout
        ActuatorPDOMap map{}; // uses defaults from header
        EthercatActuator act1(1, map);
        EthercatActuator act2(2, map);

        ActuatorCommand  cmd1{}, cmd2{};
        ActuatorFeedback fb1{},  fb2{};

        cmd1.controlword = CW_FAULT_RESET;
        cmd2.controlword = CW_FAULT_RESET;

        const auto period = std::chrono::microseconds(5000); // 5 ms

        for (int i = 0; i < 10000; ++i) {
            // Write desired command into PDO
            act1.writeCommand(cmd1);
            act2.writeCommand(cmd2);

            // Exchange once for whole bus
            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                act1.readFeedback(fb1);
                act2.readFeedback(fb2);

                act1.advanceCiA402(fb1, cmd1);
                act2.advanceCiA402(fb2, cmd2);

                // 5) When each drive reaches OP-ENABLED (0x27), set CST + torque
                if (fb1.status == 0x27) {
                    cmd1.mode        = CST;               // cyclic synchronous torque
                    cmd1.controlword = CW_ENABLE_OPERATION;
                    cmd1.target_tor  = 100;               // torque command for slave #1
                }
                if (fb2.status == 0x27) {
                    cmd2.mode        = CST;
                    cmd2.controlword = CW_ENABLE_OPERATION;
                    cmd2.target_tor  = 200;               // torque command for slave #2
                }

                // Optional: debug print (trim as needed)
                std::printf(
                  "WKC=%d | S1: sw=0x%02X tor=%d pos=%d vel=%d | S2: sw=0x%02X tor=%d pos=%d vel=%d \r",
                  wkc,
                  fb1.status, fb1.tor, fb1.pos, fb1.vel,
                  fb2.status, fb2.tor, fb2.pos, fb2.vel
                );
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
