//
// Created by dongg on 25. 11. 7.
//
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <chrono>
#include <csignal>
#include <limits>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

struct BacklashTestConfig {
    // vibration
    int torque_mag = 40;   // ± torque
    int flip_ms    = 10;   // flip interval (ms)
    int period_us  = 500;  // control loop (us)

    // sweep
    int N = 10;                           // number of measurement positions
    int position_error_threshold = 500;   // counts
    int position_error_timeout   = 10000; // micro seconds
    int measure_time_s = 3;               // seconds
    std::string csv_path = "backlash_results.csv";
};

static BacklashTestConfig load_cfg(const std::string& yaml_path) {
    BacklashTestConfig cfg;

    try {
        YAML::Node root = YAML::LoadFile(yaml_path);

        if (root["backlash_test"]) {
            auto t = root["backlash_test"];
            if (t["torque_mag"]) cfg.torque_mag = t["torque_mag"].as<int>();
            if (t["flip_dt_ms"]) cfg.flip_ms    = t["flip_dt_ms"].as<int>();
            if (t["period_us"])  cfg.period_us  = t["period_us"].as<int>();
        } else {
            std::fprintf(stderr, "WARN: missing 'backlash_test' in %s, using defaults.\n", yaml_path.c_str());
        }

        if (root["backlash_sweep"]) {
            auto s = root["backlash_sweep"];
            if (s["N"])                         cfg.N  = s["N"].as<int>();
            if (s["position_error_threshold"])  cfg.position_error_threshold = s["position_error_threshold"].as<int>();
            if (s["position_error_timeout_ms"]) cfg.position_error_timeout = s["position_error_timeout_ms"].as<int>();
            if (s["measure_time_s"])            cfg.measure_time_s = s["measure_time_s"].as<int>();
            if (s["csv_path"])                  cfg.csv_path = s["csv_path"].as<std::string>();
        } else {
            std::fprintf(stderr, "WARN: missing 'backlash_sweep' in %s, using defaults.\n", yaml_path.c_str());
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "WARN: cannot read %s (%s). Using defaults.\n", yaml_path.c_str(), e.what());
    }

    // clamps / sanity
    if (cfg.torque_mag > 32767) cfg.torque_mag = 32767;
    if (cfg.torque_mag < -32768) cfg.torque_mag = -32768;
    if (cfg.flip_ms < 1)  cfg.flip_ms = 1;
    if (cfg.period_us < 50) cfg.period_us = 50;
    if (cfg.N < 1) cfg.N = 1;
    if (cfg.position_error_threshold < 1) cfg.position_error_threshold = 1;
    if (cfg.measure_time_s < 1) cfg.measure_time_s = 1;

    return cfg;
}

// Wait in CSP until |pos - target| < threshold for stable_count consecutive cycles (to avoid bouncing)
static bool wait_until_at_position(EthercatMaster& master,
                                   EthercatActuator& act,
                                   ActuatorCommand& cmd,
                                   ActuatorFeedback& fb,
                                   int32_t target_pos,
                                   int threshold,
                                   int timeout_ms,
                                   std::chrono::microseconds period,
                                   int stable_required = 100
                                   )
{
    cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION; // 0x08
    cmd.target_pos = target_pos;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    int stable = 0;

    while (!g_stop) {
        act.writeCommand(cmd);
        int wkc = master.tickOnce();
        if (wkc >= master.expectedWKC()) {
            act.readFeedback(fb);
            act.advanceCiA402(fb, cmd);

            if (fb.status == 0x27) {
                cmd.controlword = CW_ENABLE_OPERATION;
                cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION;
            }

            int err = std::abs(fb.pos - target_pos);
            if (err < threshold) {
                ++stable;
                if (stable >= stable_required) return true;
            } else {
                stable = 0;
            }
        }

        if (std::chrono::steady_clock::now() > deadline) {
            std::fprintf(stderr, "Timeout waiting for position %d (last err=%d)\n", target_pos, std::abs(fb.pos - target_pos));
            return false;
        }
        std::this_thread::sleep_for(period);
    }
    return false;
}

// Measure backlash at current position by vibrating in CST and tracking max peak-to-peak over the window
static long long measure_backlash(EthercatMaster& master,
                                  EthercatActuator& act,
                                  ActuatorCommand& cmd,
                                  ActuatorFeedback& fb,
                                  int16_t torque_mag,
                                  std::chrono::milliseconds flip_dt,
                                  std::chrono::seconds measure_time,
                                  std::chrono::microseconds loop_period)
{
    auto start = std::chrono::steady_clock::now();
    auto last_flip = start;
    int16_t tor = torque_mag;

    bool have_minmax = false;
    int32_t pos_min = std::numeric_limits<int32_t>::max();
    int32_t pos_max = std::numeric_limits<int32_t>::min();

    long long max_delta_overall = 0;

    // Put CST mode
    cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
    cmd.target_tor = 0;

    while (!g_stop) {
        // End condition
        if (std::chrono::steady_clock::now() - start >= measure_time) break;

        // Write & exchange
        act.writeCommand(cmd);
        int wkc = master.tickOnce();

        if (wkc >= master.expectedWKC()) {
            act.readFeedback(fb);
            act.advanceCiA402(fb, cmd);

            if (fb.status == 0x27) {
                cmd.controlword = CW_ENABLE_OPERATION;
                cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

                // update per-window min/max
                if (!have_minmax) { pos_min = pos_max = fb.pos; have_minmax = true; }
                else {
                    if (fb.pos < pos_min) pos_min = fb.pos;
                    if (fb.pos > pos_max) pos_max = fb.pos;
                }
            }

            // flip torque
            auto now = std::chrono::steady_clock::now();
            if (now - last_flip >= flip_dt) {
                tor = (tor > 0) ? static_cast<int16_t>(-torque_mag) : torque_mag;
                last_flip = now;

                if (have_minmax) {
                    long long delta = static_cast<long long>(pos_max) - static_cast<long long>(pos_min);
                    if (delta > max_delta_overall) max_delta_overall = delta;
                    // reset per-cycle window
                    pos_min = pos_max = fb.pos;
                    have_minmax = true;
                }
            }
            cmd.target_tor = tor;
        }

        std::this_thread::sleep_for(loop_period);
    }

    // Zero torque
    cmd.target_tor = 0;
    act.writeCommand(cmd);
    master.tickOnce();

    return max_delta_overall;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::puts("Usage: check_backlash_sweep <ifname> [config.yaml]\n  e.g., check_backlash_sweep eth0 ../src/config.yaml");
        return 1;
    }
    std::signal(SIGINT, handle_sigint);

    const char* ifname = argv[1];
    const std::string yaml_path = (argc >= 3) ? argv[2] : "../src/config.yaml";
    BacklashTestConfig cfg = load_cfg(yaml_path);

    std::printf("Config:\n  torque_mag=%d, flip_dt_ms=%d, period_us=%d\n  N=%d, threshold=%d, measure_time_s=%d\n  csv=%s\n",
                cfg.torque_mag, cfg.flip_ms, cfg.period_us,
                cfg.N, cfg.position_error_threshold, cfg.measure_time_s,
                cfg.csv_path.c_str());

    try {
        // Master & single actuator (slave #1)
        EthercatMaster master(ifname);
        if (master.slaveCount() < 1) {
            std::fprintf(stderr, "No EtherCAT slaves found.\n");
            return 2;
        }

        constexpr int SLAVE_ID = 1;
        ActuatorPDOMap   map{};
        EthercatActuator act(SLAVE_ID, map);
        ActuatorCommand  cmd{};
        ActuatorFeedback fb{};

        const auto loop_period = std::chrono::microseconds(cfg.period_us);
        const auto flip_dt     = std::chrono::milliseconds(cfg.flip_ms);
        const auto measure_t   = std::chrono::seconds(cfg.measure_time_s);
        const int16_t TORQUE   = static_cast<int16_t>(cfg.torque_mag);

        // Fault reset and bring to OP
        cmd.controlword = CW_FAULT_RESET;
        cmd.mode        = MODE_CYCLIC_SYNCHRONOUS_POSITION;
        cmd.target_pos  = 0;

        // Step 1: go to position 0 (CSP) and wait until within threshold
        std::printf("Moving to origin (0) in CSP...\n");
        if (!wait_until_at_position(master, act, cmd, fb,
                                    /*target*/0,
                                    cfg.position_error_threshold,
                                    cfg.position_error_timeout,
                                    loop_period)) {
            std::fprintf(stderr, "Failed to reach origin.\n");
            return 3;
        }
        std::printf("At origin: pos=%d\n", fb.pos);

        // Step 2: Sweep N positions across range [0, RANGE]
        const long long RANGE = 65536LL * 22LL; // counts
        std::vector<long long> results;
        results.reserve(cfg.N);

        for (int i = 0; i < cfg.N && !g_stop; ++i) {
            long long target_ll = (cfg.N == 1) ? 0 : (RANGE * i) / (cfg.N - 1);
            int32_t target_pos = static_cast<int32_t>(std::llround(target_ll));

            // 2-1) Go to designated position in CSP
            std::printf("Position %d/%d: moving to %d ...\n", i+1, cfg.N, target_pos);
            if (!wait_until_at_position(master, act, cmd, fb,
                                        target_pos,
                                        cfg.position_error_threshold,
                                        cfg.position_error_timeout,
                                        loop_period)) {
                std::fprintf(stderr, "Failed to reach position %d (target=%d)\n", i+1, target_pos);
                results.push_back(0);
                continue;
            }

            // 2-2) Measure backlash in CST for measure_time_s
            std::printf("Measuring backlash for %d s (CST, ±%d, flip %d ms)...\n",
                        cfg.measure_time_s, cfg.torque_mag, cfg.flip_ms);

            long long delta = measure_backlash(master, act, cmd, fb,
                                               TORQUE, flip_dt, measure_t, loop_period);

            // 2-3) Store result
            results.push_back(delta);
            std::printf("  → Result Δ=%lld counts\n", delta);

            // Return to CSP at current target to re-hold position
            cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION;
            cmd.target_pos = target_pos;
        }

        // 3) Write CSV row (N values)
        {
            // Ensure directory exists if relative path includes dirs (left to user); we just write the file.
            std::ofstream ofs(cfg.csv_path, std::ios::app); // append row
            if (!ofs) {
                std::fprintf(stderr, "ERROR: cannot open CSV for write: %s\n", cfg.csv_path.c_str());
            } else {
                for (int i = 0; i < (int)results.size(); ++i) {
                    ofs << results[i];
                    if (i + 1 < (int)results.size()) ofs << ",";
                }
                ofs << "\n";
                ofs.close();
                std::printf("Saved CSV row to %s\n", cfg.csv_path.c_str());
            }
        }

        // Graceful stop: zero torque and disable
        cmd.target_tor  = 0;
        cmd.controlword = CW_DISABLE_OPERATION;
        act.writeCommand(cmd);
        master.tickOnce();

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }

    std::puts("Done.");
    return 0;
}