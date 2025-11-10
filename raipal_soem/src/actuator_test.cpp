//
// Created by dongghk on 25. 11. 3.
//
// Created by dongghk on 25. 9. 4.
// src/actuator_test.cpp
#include <cstdio>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <csignal>
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

// ---------- Test Parameters ----------
static constexpr auto    kControlPeriod     = std::chrono::microseconds(5000); // 5ms
static constexpr auto    kLegDurationActive       = std::chrono::seconds(10);//std::chrono::minutes(1);         // 5 minutes per velocity leg
static constexpr auto    kBacklashEveryActive = std::chrono::minutes(1);        // run sweep every 10 minutes of active run


// THERMAL WATCHDOG: thresholds with hysteresis
static constexpr int     kTempHighC = 60;    // disable when >= 60°C
static constexpr int     kTempLowC  = 50;    // re-enable when <= 50°C


static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

// ---------- YAML config (same shape & style as main.cpp) ----------
struct actuator_cfg_t {
    std::string name;
    uint8_t  mode;              // not strictly used (we force modes per stage)
    int16_t  target_torque;
    int32_t  target_velocity;   // used for +/− velocity legs
    int32_t  target_position;
    uint16_t kp;
    uint16_t kd;
};

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

static std::vector<actuator_cfg_t> load_actuator_cfg(const char* yaml_path, int expected_size) {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node seq = root["actuators"].IsDefined() ? root["actuators"] : root;
    if (!seq.IsSequence()) {
        std::fprintf(stderr, "Config error: top-level must be a sequence\n");
        std::exit(2);
    }
    if (expected_size > 0 && static_cast<int>(seq.size()) != expected_size) {
        std::fprintf(stderr, "Config error: YAML entries (%zu) != slave count (%d)\n",
                     seq.size(), expected_size);
        std::exit(3);
    }
    std::vector<actuator_cfg_t> v;
    v.reserve(seq.size());
    for (size_t i = 0; i < seq.size(); ++i) {
        const auto& n = seq[i];
        actuator_cfg_t c{};
        c.name            = n["name"]            ? n["name"].as<std::string>() : ("slave_" + std::to_string(i+1));
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

struct BacklashTestConfig {
    // vibration
    int torque_mag = 40;
    int flip_ms    = 10;
    int period_us  = 500;
    // sweep
    int N                           = 10;
    int position_error_threshold    = 500;
    int position_error_timeout_ms   = 10000;
    int measure_time_s              = 3;
    std::string csv_path            = "backlash_results.csv";
    int slave_id                    = 1; // optional selector
    int backlash_threshold          = 0;
};

static BacklashTestConfig load_backlash_cfg(const char* yaml_path) {
    BacklashTestConfig cfg;
    try{
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (root["backlash_test"]){
            auto t = root["backlash_test"];
            if (t["torque_mag"]) cfg.torque_mag = t["torque_mag"].as<int>();
            if (t["flip_dt_ms"]) cfg.flip_ms    = t["flip_dt_ms"].as<int>();
            if (t["period_us"])  cfg.period_us  = t["period_us"].as<int>();
            if (t["backlash_threshold"]) cfg.backlash_threshold = t["backlash_threshold"].as<int>();
        }
        if (root["backlash_sweep"]){
            auto s = root["backlash_sweep"];
            if (s["N"])                         cfg.N  = s["N"].as<int>();
            if (s["position_error_threshold"])  cfg.position_error_threshold = s["position_error_threshold"].as<int>();
            if (s["position_error_timeout_ms"]) cfg.position_error_timeout_ms = s["position_error_timeout_ms"].as<int>();
            if (s["measure_time_s"])            cfg.measure_time_s = s["measure_time_s"].as<int>();
            if (s["csv_path"])                  cfg.csv_path = s["csv_path"].as<std::string>();
            if (s["slave_id"])                  cfg.slave_id = s["slave_id"].as<int>();
        }
    }catch(const std::exception& e){
        std::fprintf(stderr,"WARN: cannot read %s (%s). Using defaults.\n", yaml_path, e.what());
    }
    // sanity
    if (cfg.torque_mag > 32767) cfg.torque_mag = 32767;
    if (cfg.torque_mag < -32768) cfg.torque_mag = -32768;
    if (cfg.flip_ms < 1) cfg.flip_ms = 1;
    if (cfg.period_us < 50) cfg.period_us = 50;
    if (cfg.N < 1) cfg.N = 1;
    if (cfg.position_error_threshold < 1) cfg.position_error_threshold = 1;
    if (cfg.measure_time_s < 1) cfg.measure_time_s = 1;
    if (cfg.slave_id < 1) cfg.slave_id = 1;
    return cfg;
}


// ---------- Helpers from check_backlash ----------
static bool wait_until_at_position(EthercatMaster& master,
                                   EthercatActuator& act,
                                   ActuatorCommand& cmd,
                                   ActuatorFeedback& fb,
                                   int32_t target_pos,
                                   int threshold,
                                   int timeout_ms,
                                   std::chrono::microseconds period,
                                   int stable_required = 100)
{
    cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION;
    cmd.target_pos = target_pos;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    int stable = 0;

    while (!g_stop) {
        act.writeCommand(cmd);
        int wkc = master.tickOnce();
        if (wkc >= master.expectedWKC()){
            act.readFeedback(fb);
            act.advanceCiA402(fb, cmd);

            if (fb.status == 0x27){
                cmd.controlword = CW_ENABLE_OPERATION;
                cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION;
            }
            int err = std::abs(fb.pos - target_pos);
            if (err < threshold){
                if (++stable >= stable_required) return true;
            }else{
                stable = 0;
            }
        }
        if (std::chrono::steady_clock::now() > deadline){
            std::fprintf(stderr, "Timeout waiting for pos %d (last err=%d)\n", target_pos, std::abs(fb.pos - target_pos));
            return false;
        }
        std::this_thread::sleep_for(period);
    }
    return false;
}


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

    cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
    cmd.target_tor = 0;

    while (!g_stop) {
        if (std::chrono::steady_clock::now() - start >= measure_time) break;

        act.writeCommand(cmd);
        int wkc = master.tickOnce();
        if (wkc >= master.expectedWKC()){
            act.readFeedback(fb);
            act.advanceCiA402(fb, cmd);

            if (fb.status == 0x27){
                cmd.controlword = CW_ENABLE_OPERATION;
                cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

                if (!have_minmax){ pos_min = pos_max = fb.pos; have_minmax = true; }
                else {
                    if (fb.pos < pos_min) pos_min = fb.pos;
                    if (fb.pos > pos_max) pos_max = fb.pos;
                }
            }

            auto now = std::chrono::steady_clock::now();
            if (now - last_flip >= flip_dt){
                tor = (tor > 0) ? static_cast<int16_t>(-torque_mag) : torque_mag;
                last_flip = now;

                if (have_minmax){
                    long long d = static_cast<long long>(pos_max) - static_cast<long long>(pos_min);
                    if (d > max_delta_overall) max_delta_overall = d;
                    pos_min = pos_max = fb.pos; // per-cycle reset
                    have_minmax = true;
                }
            }
            cmd.target_tor = tor;
        }
        std::this_thread::sleep_for(loop_period);
    }

    cmd.target_tor = 0;
    act.writeCommand(cmd);
    master.tickOnce();
    return max_delta_overall;
}


// Run the full backlash sweep (single selected slave), append one CSV row of N deltas.
static int run_backlash_sweep(EthercatMaster& master,
                               std::vector<EthercatActuator>& acts,
                               BacklashTestConfig& bc,
                               const std::string& yaml_path)
{
    const int sid = std::min(std::max(1, bc.slave_id), static_cast<int>(acts.size()));
    EthercatActuator& act = acts[sid-1];

    ActuatorCommand  cmd{};
    ActuatorFeedback fb{};

    const auto loop_period = std::chrono::microseconds(bc.period_us);
    const auto flip_dt     = std::chrono::milliseconds(bc.flip_ms);
    const auto measure_t   = std::chrono::seconds(bc.measure_time_s);
    const int16_t TORQUE   = static_cast<int16_t>(bc.torque_mag);

    std::printf("\n[BACKLASH] Starting sweep on slave %d using config %s\n", sid, yaml_path.c_str());

    // Move to zero (CSP)
    cmd.controlword = CW_FAULT_RESET;
    cmd.mode        = MODE_CYCLIC_SYNCHRONOUS_POSITION;
    cmd.target_pos  = 0;

    if (!wait_until_at_position(master, act, cmd, fb,
                                0,
                                bc.position_error_threshold,
                                bc.position_error_timeout_ms,
                                loop_period)) {
        std::fprintf(stderr, "[BACKLASH] Failed to reach origin. Aborting sweep.\n");
        return -1;
    }

    const long long RANGE = 65536LL * 22LL;
    std::vector<long long> results; results.reserve(bc.N);

    for (int i=0;i<bc.N && !g_stop;++i){
        long long target_ll = (bc.N==1) ? 0 : (RANGE * i) / (bc.N - 1);
        int32_t target_pos = static_cast<int32_t>(std::llround(target_ll));

        std::printf("[BACKLASH] (%d/%d) Move CSP to %d ...\n", i+1, bc.N, target_pos);
        if (!wait_until_at_position(master, act, cmd, fb,
                                    target_pos,
                                    bc.position_error_threshold,
                                    bc.position_error_timeout_ms,
                                    loop_period)) {
            std::fprintf(stderr, "[BACKLASH] Failed to reach pos=%d, writing 0.\n", target_pos);
            results.push_back(0);
            continue;
        }

        std::printf("[BACKLASH] Measure CST ±%d flip %d ms for %d s ...\n",
                    bc.torque_mag, bc.flip_ms, bc.measure_time_s);

        long long d = measure_backlash(master, act, cmd, fb,
                                       TORQUE, flip_dt, measure_t, loop_period);
        results.push_back(d);
        std::printf("[BACKLASH]   → Δ=%lld\n", d);

        // hold in CSP at the current target (optional)
        cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION;
        cmd.target_pos = target_pos;
    }

    // Append one CSV row
    std::ofstream ofs(bc.csv_path, std::ios::app);
    if (!ofs){
        std::fprintf(stderr, "[BACKLASH] ERROR: cannot open CSV: %s\n", bc.csv_path.c_str());
    }else{
        for (int i=0;i<(int)results.size();++i){
            ofs << results[i];
            if (i+1 < (int)results.size()) ofs << ",";
        }
        ofs << "\n";
        ofs.close();
        std::printf("[BACKLASH] Saved row to %s\n", bc.csv_path.c_str());
    }

    int exceed = 0;
    if (bc.backlash_threshold > 0) {
        for (auto d : results) {
            if (d > static_cast<long long>(bc.backlash_threshold)) ++exceed;
        }
    }
    return exceed;   // ADD
}


// ---------- State machine for the 2-stage cycle ----------
enum class Stage {
    VELOCITY_FWD = 0,
    VELOCITY_REV = 1,
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::puts("Usage: actuator_test <ifname> <config.yaml>\n  e.g., actuator_test eth0 config.yaml");
        return 1;
    }

    std::signal(SIGINT, handle_sigint);

    const char* ifname   = argv[1];
    const char* yamlpath = argv[2];

    try {
        EthercatMaster master(ifname);
        const int slave_count = master.slaveCount();
        if (slave_count <= 0) {
            std::fprintf(stderr, "No EtherCAT slaves discovered\n");
            return 2;
        }

        auto acfg = load_actuator_cfg(yamlpath, slave_count);
        BacklashTestConfig bcfg = load_backlash_cfg(yamlpath);

        ActuatorPDOMap map{};
        std::vector<EthercatActuator> acts;
        acts.reserve(slave_count);
        for (int sid = 1; sid <= slave_count; ++sid) {
            acts.emplace_back(sid, map);
        }

        std::vector<ActuatorCommand>  cmds(slave_count);
        std::vector<ActuatorFeedback> fbs (slave_count);

        for (int k = 0; k < slave_count; ++k) {
            cmds[k].controlword = CW_FAULT_RESET; // clear faults first
            cmds[k].kp          = acfg[k].kp;
            cmds[k].kd          = acfg[k].kd;
        }

        Stage stage = Stage::VELOCITY_FWD;

        // Timers that only advance when actively running (not overheated)
        auto last_tick = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration active_elapsed{0};
        std::chrono::steady_clock::duration leg_elapsed{0};

        bool paused = false;

        // Control loop
        while (!g_stop) {
            // 6) Write outputs for all slaves
            for (int k = 0; k < slave_count; ++k) {
                acts[k].writeCommand(cmds[k]);
            }

            // Exchange PD once
            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                // Refresh feedback and latch EO per-drive
                int hot_count = 0;
                for (int k = 0; k < slave_count; ++k) {
                    acts[k].readFeedback(fbs[k]);
                    acts[k].advanceCiA402(fbs[k], cmds[k]);

                    // If drive is in OP, keep EO latched (individually)
                    if (fbs[k].status == 0x27) {
                        cmds[k].controlword = CW_ENABLE_OPERATION;
                    }
                }

                // Determine global pause (if ANY hot ⇒ pause)
                for (int k=0;k<slave_count;++k){
                    if (fbs[k].temp >= kTempHighC) { paused = true; break; }
                }
                if (paused){
                    // check resume condition: ALL cooled
                    bool allCooled = true;
                    for (int k=0;k<slave_count;++k){
                        if (fbs[k].temp > kTempLowC){ allCooled = false; break; }
                    }
                    if (allCooled) paused = false;
                }
                                // 3) Apply stage commands if NOT paused; else force zero motion
                if (!paused){
                    for (int k=0;k<slave_count;++k){
                        cmds[k].mode = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                        if (stage == Stage::VELOCITY_FWD){
                            cmds[k].target_vel = acfg[k].target_velocity;
                        } else {
                            cmds[k].target_vel = (acfg[k].target_velocity > 0)
                                ? -acfg[k].target_velocity : acfg[k].target_velocity;
                        }
                        cmds[k].kp = acfg[k].kp; cmds[k].kd = acfg[k].kd;
                    }
                } else {
                    for (int k=0;k<slave_count;++k){
                        cmds[k].mode = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                        cmds[k].target_vel = 0;
                        cmds[k].target_tor = 0;
                        cmds[k].target_pos = fbs[k].pos;
                    }
                }

                // 4) Advance ACTIVE timers only if not paused
                auto now = std::chrono::steady_clock::now();
                auto dt  = now - last_tick;
                last_tick = now;

                if (!paused){
                    active_elapsed += dt;
                    leg_elapsed    += dt;

                    // 4a) Stage flip every 1 min of active run
                    if (leg_elapsed >= kLegDurationActive){
                        stage = (stage == Stage::VELOCITY_FWD) ? Stage::VELOCITY_REV : Stage::VELOCITY_FWD;
                        leg_elapsed = std::chrono::steady_clock::duration::zero();
                        // std::printf("\n[RUN] Flip direction: stage=%d\n", static_cast<int>(stage));
                    }

                    // 4b) Backlash every 10 min of active run
                    if (active_elapsed >= kBacklashEveryActive){
                        std::printf("\n[SCHED] Running backlash sweep (active run ≥ 10 min)\n");

                        // Freeze commands (zero motion) before measurement
                        for (int k=0;k<slave_count;++k){
                            cmds[k].mode = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                            cmds[k].target_vel = 0;
                            cmds[k].target_tor = 0;
                        }
                        // Push one cycle of zero before switching mode in backlash
                        for (int k=0;k<slave_count;++k) acts[k].writeCommand(cmds[k]);
                        master.tickOnce();

                        int exceed = run_backlash_sweep(master, acts, bcfg, yamlpath);

                        if (exceed >= 3) {
                            std::printf("\n[BACKLASH] %d positions exceeded threshold (%d) → terminating test.\n",
                                        exceed, bcfg.backlash_threshold);

                            // graceful stop
                            for (int k=0;k<slave_count;++k){
                                cmds[k].target_vel = 0;
                                cmds[k].target_tor = 0;
                                cmds[k].controlword = CW_DISABLE_OPERATION;
                                acts[k].writeCommand(cmds[k]);
                            }
                            master.tickOnce();
                            break; // exit the control loop
                        }

                        // Reset only the 10-min scheduler; keep leg timer as-is (we paused motion)
                        active_elapsed -= kBacklashEveryActive;
                        // After backlash, resume loop; stage continues from where it left.
                        last_tick = std::chrono::steady_clock::now();
                    }
                }


                // Debug line: show temp and overheat flag per actuator
                // std::printf("PAUSE=%d STG=%d WKC=%d |", paused?1:0, static_cast<int>(stage), wkc);
                // for (int k=0;k<slave_count;++k){
                //     std::printf(" S%d: sw=0x%02X vel=%d pos=%d T=%d",
                //                 k+1, fbs[k].status, fbs[k].vel, fbs[k].pos, fbs[k].temp);
                //     if (k != slave_count-1) std::printf(" |");
                // }
                // std::printf(" \r");
                // std::fflush(stdout);
            }

            std::this_thread::sleep_for(kControlPeriod);
        }

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }

    std::puts("\nStopped.");
    return 0;
}
