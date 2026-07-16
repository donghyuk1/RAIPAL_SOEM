//
// Created by dongghk on 25. 11. 3.
//
// Created by dongghk on 25. 9. 4.
// src/actuator_test.cpp
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <csignal>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>

#include <yaml-cpp/yaml.h>

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

// ---------- Test Parameters ----------
static constexpr auto    kControlPeriod     = std::chrono::microseconds(5000); // 5ms
static constexpr auto    kLegDurationActive = std::chrono::minutes(1);         // velocity leg length
static constexpr auto    kBacklashEveryActive = std::chrono::minutes(10);      // run sweep every 10 minutes of active run

static constexpr int     kStableTicks = 4000; // ticks the drive must stay in-threshold before "stable"

// THERMAL WATCHDOG: thresholds with hysteresis (per-actuator)
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
        }
        if (root["backlash_sweep"]){
            auto s = root["backlash_sweep"];
            if (s["N"])                         cfg.N  = s["N"].as<int>();
            if (s["position_error_threshold"])  cfg.position_error_threshold = s["position_error_threshold"].as<int>();
            if (s["position_error_timeout_ms"]) cfg.position_error_timeout_ms = s["position_error_timeout_ms"].as<int>();
            if (s["measure_time_s"])            cfg.measure_time_s = s["measure_time_s"].as<int>();
            if (s["csv_path"])                  cfg.csv_path = s["csv_path"].as<std::string>();
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
    return cfg;
}


struct FrictionTestConfig {
    int moving_threshold = 1000;          // counts to consider "moving"
    int torque_step      = 10;            // counts per step (CST command units)
    int dwell_ms         = 20;            // hold each torque level for this many ms
    int torque_limit     = 4000;          // absolute max torque to try
    int period_us        = 500;           // control loop period (matching your system)
    int position_error_threshold = 500;   // for CSP positioning before the test
    int position_error_timeout_ms = 10000;
};

static FrictionTestConfig load_friction_cfg(const char* yaml_path) {
    FrictionTestConfig fc;
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (root["friction_test"]) {
            auto f = root["friction_test"];
            if (f["moving_threshold"])           fc.moving_threshold = f["moving_threshold"].as<int>();
            if (f["torque_step"])                fc.torque_step = f["torque_step"].as<int>();
            if (f["dwell_ms"])                   fc.dwell_ms = f["dwell_ms"].as<int>();
            if (f["torque_limit"])               fc.torque_limit = f["torque_limit"].as<int>();
            if (f["period_us"])                  fc.period_us = f["period_us"].as<int>();
            if (f["position_error_threshold"])   fc.position_error_threshold = f["position_error_threshold"].as<int>();
            if (f["position_error_timeout_ms"])  fc.position_error_timeout_ms = f["position_error_timeout_ms"].as<int>();
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "WARN: cannot read friction_test from %s (%s). Using defaults.\n", yaml_path, e.what());
    }
    if (fc.torque_step < 1) fc.torque_step = 1;
    if (fc.dwell_ms < 5) fc.dwell_ms = 5;
    if (fc.torque_limit < fc.torque_step) fc.torque_limit = fc.torque_step;
    if (fc.period_us < 50) fc.period_us = 50;
    if (fc.moving_threshold < 1) fc.moving_threshold = 1;
    return fc;
}

// ---------- Stop-criteria thresholds (SEPARATE yaml so the user can tune easily) ----------
struct ThresholdConfig {
    int backlash_threshold         = 300;   // Δ above this counts as an "exceed"
    int backlash_outlier_threshold = 1000;  // Δ at/above this is ignored as an outlier
    int backlash_exceed_limit      = 3;     // stop actuator when exceed count > this within one sweep
    int friction_threshold         = 0;     // stop actuator when mean friction < this (<=0 disables)
};

static ThresholdConfig load_threshold_cfg(const char* yaml_path) {
    ThresholdConfig t;
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        // Accept keys either at top level or nested under "thresholds:"
        YAML::Node n = root["thresholds"] ? root["thresholds"] : root;
        if (n["backlash_threshold"])         t.backlash_threshold         = n["backlash_threshold"].as<int>();
        if (n["backlash_outlier_threshold"]) t.backlash_outlier_threshold = n["backlash_outlier_threshold"].as<int>();
        if (n["backlash_exceed_limit"])      t.backlash_exceed_limit      = n["backlash_exceed_limit"].as<int>();
        if (n["friction_threshold"])         t.friction_threshold         = n["friction_threshold"].as<int>();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "WARN: cannot read thresholds %s (%s). Using defaults.\n", yaml_path, e.what());
    }
    if (t.backlash_exceed_limit < 0) t.backlash_exceed_limit = 0;
    return t;
}


// ============================================================================
// Multi-actuator helpers.
//
// EtherCAT exchanges the WHOLE bus on every tickOnce(), so to drive several
// actuators "together" every helper commands ALL drives each cycle. An `active`
// mask selects which actuators participate; inactive (done) drives keep whatever
// disabled command the caller set and are never re-enabled here.
// ============================================================================

// Hold every ACTIVE actuator at its target position (CSP) until all of them
// have stayed within `threshold` continuously for kStableTicks, or timeout.
// Returns true only if every active actuator stabilized.
static bool wait_until_at_position_multi(EthercatMaster& master,
                                         std::vector<EthercatActuator>& acts,
                                         std::vector<ActuatorCommand>& cmds,
                                         std::vector<ActuatorFeedback>& fbs,
                                         const std::vector<char>& active,
                                         const std::vector<int32_t>& targets,
                                         int threshold,
                                         int timeout_ms,
                                         std::chrono::microseconds period,
                                         int stable_required = kStableTicks)
{
    const size_t n = acts.size();
    for (size_t k = 0; k < n; ++k) {
        if (!active[k]) continue;
        cmds[k].mode       = MODE_CYCLIC_SYNCHRONOUS_POSITION;
        cmds[k].target_pos = targets[k];
    }

    std::vector<int> stable(n, 0);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (!g_stop) {
        for (size_t k = 0; k < n; ++k) acts[k].writeCommand(cmds[k]);
        int wkc = master.tickOnce();
        if (wkc >= master.expectedWKC()) {
            bool all_ok = true;
            for (size_t k = 0; k < n; ++k) {
                acts[k].readFeedback(fbs[k]);
                if (!active[k]) continue;              // done drive: leave disabled, don't advance
                acts[k].advanceCiA402(fbs[k], cmds[k]);

                if (fbs[k].status == 0x27) cmds[k].controlword = CW_ENABLE_OPERATION;
                cmds[k].mode       = MODE_CYCLIC_SYNCHRONOUS_POSITION;
                cmds[k].target_pos = targets[k];       // keep holding the target (real stabilize)

                int err = std::abs(fbs[k].pos - targets[k]);
                if (err < threshold) ++stable[k];
                else                 stable[k] = 0;
                if (stable[k] < stable_required) all_ok = false;
            }
            if (all_ok) return true;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            for (size_t k = 0; k < n; ++k) {
                if (active[k] && stable[k] < stable_required)
                    std::fprintf(stderr, "Timeout: act%zu not at pos %d (err=%d)\n",
                                 k + 1, targets[k], std::abs(fbs[k].pos - targets[k]));
            }
            return false;
        }
        std::this_thread::sleep_for(period);
    }
    return false;
}


// Vibrate every ACTIVE actuator with ±torque_mag (flipping every flip_dt) for
// measure_time and record each drive's peak position span (backlash Δ).
static void measure_backlash_multi(EthercatMaster& master,
                                   std::vector<EthercatActuator>& acts,
                                   std::vector<ActuatorCommand>& cmds,
                                   std::vector<ActuatorFeedback>& fbs,
                                   const std::vector<char>& active,
                                   int16_t torque_mag,
                                   std::chrono::milliseconds flip_dt,
                                   std::chrono::seconds measure_time,
                                   std::chrono::microseconds loop_period,
                                   std::vector<long long>& out_delta)
{
    const size_t n = acts.size();
    auto start = std::chrono::steady_clock::now();
    auto last_flip = start;

    std::vector<int16_t> tor(n, torque_mag);
    std::vector<char>    have(n, 0);
    std::vector<int32_t> pmin(n, 0), pmax(n, 0);
    out_delta.assign(n, 0);

    for (size_t k = 0; k < n; ++k) {
        if (!active[k]) continue;
        cmds[k].mode       = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
        cmds[k].target_tor = 0;
    }

    while (!g_stop) {
        if (std::chrono::steady_clock::now() - start >= measure_time) break;

        for (size_t k = 0; k < n; ++k) acts[k].writeCommand(cmds[k]);
        int wkc = master.tickOnce();
        if (wkc >= master.expectedWKC()) {
            for (size_t k = 0; k < n; ++k) {
                acts[k].readFeedback(fbs[k]);
                if (!active[k]) continue;
                acts[k].advanceCiA402(fbs[k], cmds[k]);
                if (fbs[k].status == 0x27) {
                    cmds[k].controlword = CW_ENABLE_OPERATION;
                    cmds[k].mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
                    if (!have[k]) { pmin[k] = pmax[k] = fbs[k].pos; have[k] = 1; }
                    else {
                        if (fbs[k].pos < pmin[k]) pmin[k] = fbs[k].pos;
                        if (fbs[k].pos > pmax[k]) pmax[k] = fbs[k].pos;
                    }
                }
            }

            auto now = std::chrono::steady_clock::now();
            if (now - last_flip >= flip_dt) {
                for (size_t k = 0; k < n; ++k) {
                    if (!active[k]) continue;
                    tor[k] = (tor[k] > 0) ? static_cast<int16_t>(-torque_mag) : torque_mag;
                    if (have[k]) {
                        long long d = static_cast<long long>(pmax[k]) - static_cast<long long>(pmin[k]);
                        if (d > out_delta[k]) out_delta[k] = d;
                        pmin[k] = pmax[k] = fbs[k].pos; // per-cycle reset
                    }
                }
                last_flip = now;
            }
            for (size_t k = 0; k < n; ++k) if (active[k]) cmds[k].target_tor = tor[k];
        }
        std::this_thread::sleep_for(loop_period);
    }

    for (size_t k = 0; k < n; ++k) {
        if (active[k]) cmds[k].target_tor = 0;
        acts[k].writeCommand(cmds[k]);
    }
    master.tickOnce();
}


// Ramp torque in one direction on every ACTIVE actuator until each one breaks
// away (moves >= moving_threshold from where it started). Each drive records its
// own breakaway torque; drives that never move are clamped at ±torque_limit.
static void measure_friction_dir_multi(EthercatMaster& master,
                                       std::vector<EthercatActuator>& acts,
                                       std::vector<ActuatorCommand>& cmds,
                                       std::vector<ActuatorFeedback>& fbs,
                                       const std::vector<char>& active,
                                       int dir,
                                       const FrictionTestConfig& fc,
                                       std::vector<int16_t>& out)
{
    const size_t n = acts.size();
    const auto loop_period = std::chrono::microseconds(fc.period_us);

    std::vector<int32_t> start_pos(n, 0);
    std::vector<char>    found(n, 0);
    out.assign(n, 0);

    for (size_t k = 0; k < n; ++k) {
        if (!active[k]) continue;
        cmds[k].mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
        cmds[k].controlword = CW_ENABLE_OPERATION;
        cmds[k].target_tor  = 0;
        start_pos[k] = fbs[k].pos;
        out[k] = static_cast<int16_t>(dir > 0 ? fc.torque_limit : -fc.torque_limit); // clamp default
    }

    for (int level = 0; std::abs(level) <= fc.torque_limit && !g_stop; level += dir * fc.torque_step) {
        auto dwell_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(fc.dwell_ms);
        bool all_found = false;

        while (std::chrono::steady_clock::now() < dwell_end && !g_stop) {
            for (size_t k = 0; k < n; ++k) {
                if (active[k] && !found[k]) cmds[k].target_tor = static_cast<int16_t>(level);
                acts[k].writeCommand(cmds[k]);
            }
            int wkc = master.tickOnce();
            if (wkc >= master.expectedWKC()) {
                for (size_t k = 0; k < n; ++k) {
                    acts[k].readFeedback(fbs[k]);
                    if (!active[k] || found[k]) continue;
                    acts[k].advanceCiA402(fbs[k], cmds[k]);
                    if (fbs[k].status == 0x27) {
                        cmds[k].controlword = CW_ENABLE_OPERATION;
                        cmds[k].mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
                    }
                    long long delta = static_cast<long long>(fbs[k].pos) - static_cast<long long>(start_pos[k]);
                    if (std::llabs(delta) >= static_cast<long long>(fc.moving_threshold)) {
                        out[k]   = static_cast<int16_t>(level);
                        found[k] = 1;
                        cmds[k].target_tor = 0;
                    }
                }
            }
            all_found = true;
            for (size_t k = 0; k < n; ++k) if (active[k] && !found[k]) { all_found = false; break; }
            if (all_found) break;
            std::this_thread::sleep_for(loop_period);
        }
        if (all_found) break;
    }

    for (size_t k = 0; k < n; ++k) {
        if (active[k]) cmds[k].target_tor = 0;
        acts[k].writeCommand(cmds[k]);
    }
    master.tickOnce();
}


struct SweepResult {
    std::vector<int>    exceed;    // per-actuator: positions whose Δ exceeded the backlash threshold
    std::vector<double> mean_fric; // per-actuator: mean |breakaway torque| over the sweep
    std::vector<char>   valid;     // per-actuator: mean_fric has samples
};

// Run one full sweep on all ACTIVE actuators together. Starts by STABILIZING at
// the current position (no homing move -> the old "failed to reach origin, abort"
// path is gone), then walks the N range positions measuring backlash + friction.
static SweepResult run_backlash_sweep_multi(EthercatMaster& master,
                                            std::vector<EthercatActuator>& acts,
                                            std::vector<ActuatorCommand>& cmds,
                                            std::vector<ActuatorFeedback>& fbs,
                                            const std::vector<char>& active,
                                            BacklashTestConfig& bc,
                                            FrictionTestConfig& fc,
                                            const ThresholdConfig& th,
                                            const std::string& yaml_path,
                                            const std::vector<actuator_cfg_t>& acfg)
{
    const size_t n = acts.size();
    const auto loop_period = std::chrono::microseconds(bc.period_us);
    const auto flip_dt     = std::chrono::milliseconds(bc.flip_ms);
    const auto measure_t   = std::chrono::seconds(bc.measure_time_s);
    const int16_t TORQUE   = static_cast<int16_t>(bc.torque_mag);

    SweepResult res;
    res.exceed.assign(n, 0);
    res.mean_fric.assign(n, 0.0);
    res.valid.assign(n, 0);

    std::vector<long long> fric_sum(n, 0);
    std::vector<int>       fric_cnt(n, 0);
    std::vector<std::vector<long long>> rows(n); // per-actuator CSV row (Δ,+fric,-fric triplets)

    std::printf("\n[SWEEP] Starting sweep on %d actuator(s) using config %s\n",
                static_cast<int>(std::count(active.begin(), active.end(), static_cast<char>(1))),
                yaml_path.c_str());

    // Stabilize at the CURRENT position (with settle time). Never aborts the sweep.
    std::vector<int32_t> cur(n, 0);
    for (size_t k = 0; k < n; ++k) cur[k] = fbs[k].pos;
    if (!wait_until_at_position_multi(master, acts, cmds, fbs, active, cur,
                                      bc.position_error_threshold,
                                      bc.position_error_timeout_ms, loop_period)) {
        std::fprintf(stderr, "[SWEEP] Note: not all actuators settled at current position; continuing.\n");
    }

    const long long RANGE = 65536LL * 22LL;
    for (int i = 0; i < bc.N && !g_stop; ++i) {
        const int32_t tp = static_cast<int32_t>((RANGE * i) / bc.N);
        std::vector<int32_t> targets(n, tp);

        // (1) Move to designated position (CSP). Failure here is non-fatal.
        std::printf("[SWEEP] (%d/%d) Move CSP to %d ...\n", i + 1, bc.N, tp);
        if (!wait_until_at_position_multi(master, acts, cmds, fbs, active, targets,
                                          bc.position_error_threshold,
                                          bc.position_error_timeout_ms, loop_period)) {
            std::fprintf(stderr, "[SWEEP] Some actuators did not reach %d; measuring anyway.\n", tp);
        }

        // (2) Backlash (CST vibration)
        std::vector<long long> delta;
        measure_backlash_multi(master, acts, cmds, fbs, active,
                               TORQUE, flip_dt, measure_t, loop_period, delta);
        for (size_t k = 0; k < n; ++k) {
            if (!active[k]) continue;
            rows[k].push_back(delta[k]);
            if (th.backlash_threshold > 0 &&
                delta[k] > static_cast<long long>(th.backlash_threshold) &&
                delta[k] < static_cast<long long>(th.backlash_outlier_threshold)) {
                res.exceed[k]++;
            }
        }

        // (3) Re-hold CSP before friction
        wait_until_at_position_multi(master, acts, cmds, fbs, active, targets,
                                     bc.position_error_threshold,
                                     bc.position_error_timeout_ms, loop_period);

        // (4) Friction, both directions
        std::vector<int16_t> fp, fn;
        measure_friction_dir_multi(master, acts, cmds, fbs, active, +1, fc, fp);
        wait_until_at_position_multi(master, acts, cmds, fbs, active, targets,
                                     bc.position_error_threshold,
                                     bc.position_error_timeout_ms, loop_period);
        measure_friction_dir_multi(master, acts, cmds, fbs, active, -1, fc, fn);
        wait_until_at_position_multi(master, acts, cmds, fbs, active, targets,
                                     bc.position_error_threshold,
                                     bc.position_error_timeout_ms, loop_period);

        for (size_t k = 0; k < n; ++k) {
            if (!active[k]) continue;
            rows[k].push_back(static_cast<long long>(fp[k]));
            rows[k].push_back(static_cast<long long>(fn[k]));
            fric_sum[k] += std::llabs(static_cast<long long>(fp[k])) + std::llabs(static_cast<long long>(fn[k]));
            fric_cnt[k] += 2;
            std::printf("  %s → Δ=%lld, +fric=%d, -fric=%d\n",
                        acfg[k].name.c_str(), delta[k], fp[k], fn[k]);
        }
    }

    for (size_t k = 0; k < n; ++k) {
        if (!active[k] || fric_cnt[k] == 0) continue;
        res.mean_fric[k] = static_cast<double>(fric_sum[k]) / static_cast<double>(fric_cnt[k]);
        res.valid[k] = 1;
    }

    // Append one CSV row per active actuator (prefixed with its name).
    std::ofstream ofs(bc.csv_path, std::ios::app);
    if (!ofs) {
        std::fprintf(stderr, "[SWEEP] ERROR: cannot open CSV: %s\n", bc.csv_path.c_str());
    } else {
        for (size_t k = 0; k < n; ++k) {
            if (!active[k]) continue;
            ofs << acfg[k].name;
            for (size_t j = 0; j < rows[k].size(); ++j) ofs << "," << rows[k][j];
            ofs << "\n";
        }
        std::printf("[SWEEP] Saved rows to %s\n", bc.csv_path.c_str());
    }

    return res;
}


// ---------- State machine for the 2-stage cycle ----------
enum class Stage {
    VELOCITY_FWD = 0,
    VELOCITY_REV = 1,
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::puts("Usage: actuator_test <ifname> <config.yaml> [thresholds.yaml]\n"
                  "  e.g., actuator_test eth0 config.yaml thresholds.yaml");
        return 1;
    }

    std::signal(SIGINT, handle_sigint);

    const char* ifname   = argv[1];
    const char* yamlpath = argv[2];
    const char* thpath   = (argc >= 4) ? argv[3] : "thresholds.yaml";

    try {
        EthercatMaster master(ifname);
        const int slave_count = master.slaveCount();
        if (slave_count <= 0) {
            std::fprintf(stderr, "No EtherCAT slaves discovered\n");
            return 2;
        }

        auto acfg = load_actuator_cfg(yamlpath, slave_count);
        BacklashTestConfig bcfg = load_backlash_cfg(yamlpath);
        FrictionTestConfig fcfg = load_friction_cfg(yamlpath);
        ThresholdConfig    thr  = load_threshold_cfg(thpath);

        std::printf("[CFG] %d actuator(s). Stop when: backlash exceed > %d (Δ in %d..%d), "
                    "or mean friction < %d%s. Thresholds from %s\n",
                    slave_count, thr.backlash_exceed_limit,
                    thr.backlash_threshold, thr.backlash_outlier_threshold,
                    thr.friction_threshold,
                    (thr.friction_threshold > 0) ? "" : " (friction criterion disabled)",
                    thpath);

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

        // Per-actuator state
        std::vector<char> done  (slave_count, 0); // met a stop criterion -> stays disabled
        std::vector<char> paused(slave_count, 0); // thermal pause (hysteresis)

        Stage stage = Stage::VELOCITY_FWD;

        // Timers that only advance while the fleet is actively aging
        auto last_tick = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration active_elapsed{0};
        std::chrono::steady_clock::duration leg_elapsed{0};

        // Control loop
        while (!g_stop) {
            for (int k = 0; k < slave_count; ++k) acts[k].writeCommand(cmds[k]);

            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                for (int k = 0; k < slave_count; ++k) {
                    acts[k].readFeedback(fbs[k]);
                    acts[k].advanceCiA402(fbs[k], cmds[k]);
                    if (fbs[k].status == 0x27) cmds[k].controlword = CW_ENABLE_OPERATION;
                }

                // Per-actuator thermal hysteresis (only for not-done drives)
                for (int k = 0; k < slave_count; ++k) {
                    if (done[k]) continue;
                    if      (fbs[k].temp >= kTempHighC) paused[k] = 1;
                    else if (fbs[k].temp <= kTempLowC)  paused[k] = 0;
                }

                // Apply per-actuator commands
                for (int k = 0; k < slave_count; ++k) {
                    if (done[k]) {
                        cmds[k].controlword = CW_DISABLE_OPERATION;
                        cmds[k].mode        = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                        cmds[k].target_vel  = 0;
                        cmds[k].target_tor  = 0;
                        continue;
                    }
                    cmds[k].mode = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                    if (paused[k]) {
                        cmds[k].target_vel = 0;
                        cmds[k].target_tor = 0;
                        cmds[k].target_pos = fbs[k].pos;
                    } else {
                        if (stage == Stage::VELOCITY_FWD) {
                            cmds[k].target_vel = acfg[k].target_velocity;
                        } else {
                            cmds[k].target_vel = (acfg[k].target_velocity > 0)
                                ? -acfg[k].target_velocity : acfg[k].target_velocity;
                        }
                        cmds[k].kp = acfg[k].kp; cmds[k].kd = acfg[k].kd;
                    }
                }

                // Advance timers only while at least one actuator is actively aging
                bool any_active = false;
                for (int k = 0; k < slave_count; ++k)
                    if (!done[k] && !paused[k]) { any_active = true; break; }

                auto now = std::chrono::steady_clock::now();
                auto dt  = now - last_tick;
                last_tick = now;

                if (any_active) {
                    active_elapsed += dt;
                    leg_elapsed    += dt;

                    // Stage flip every leg duration of active run
                    if (leg_elapsed >= kLegDurationActive) {
                        stage = (stage == Stage::VELOCITY_FWD) ? Stage::VELOCITY_REV : Stage::VELOCITY_FWD;
                        leg_elapsed = std::chrono::steady_clock::duration::zero();
                    }

                    // Sweep every 10 min of active run
                    if (active_elapsed >= kBacklashEveryActive) {
                        std::printf("\n[SCHED] Running sweep (active run >= 10 min)\n");

                        // Freeze non-done actuators before measurement
                        for (int k = 0; k < slave_count; ++k) {
                            if (done[k]) continue;
                            cmds[k].mode       = MODE_CYCLIC_SYNCHRONOUS_VELOCITY;
                            cmds[k].target_vel = 0;
                            cmds[k].target_tor = 0;
                        }
                        for (int k = 0; k < slave_count; ++k) acts[k].writeCommand(cmds[k]);
                        master.tickOnce();

                        std::vector<char> active(slave_count, 0);
                        for (int k = 0; k < slave_count; ++k) active[k] = done[k] ? 0 : 1;

                        SweepResult sr = run_backlash_sweep_multi(master, acts, cmds, fbs, active,
                                                                  bcfg, fcfg, thr, yamlpath, acfg);

                        // Evaluate stop criteria PER actuator
                        for (int k = 0; k < slave_count; ++k) {
                            if (done[k]) continue;
                            bool backlash_stop = (thr.backlash_threshold > 0) &&
                                                 (sr.exceed[k] > thr.backlash_exceed_limit);
                            bool friction_stop = (thr.friction_threshold > 0) && sr.valid[k] &&
                                                 (sr.mean_fric[k] < static_cast<double>(thr.friction_threshold));
                            if (backlash_stop || friction_stop) {
                                done[k] = 1;
                                cmds[k].controlword = CW_DISABLE_OPERATION;
                                cmds[k].target_vel  = 0;
                                cmds[k].target_tor  = 0;
                                acts[k].writeCommand(cmds[k]);
                                std::printf("[DONE] %s stopped (%s%s%s): exceed=%d, mean_fric=%.1f\n",
                                            acfg[k].name.c_str(),
                                            backlash_stop ? "backlash" : "",
                                            (backlash_stop && friction_stop) ? "+" : "",
                                            friction_stop ? "friction" : "",
                                            sr.exceed[k], sr.valid[k] ? sr.mean_fric[k] : -1.0);
                            }
                        }
                        master.tickOnce();

                        // Terminate the WHOLE process only when every actuator is done
                        bool all_done = true;
                        for (int k = 0; k < slave_count; ++k) if (!done[k]) { all_done = false; break; }
                        if (all_done) {
                            std::printf("\n[TERMINATE] All actuators met their stop criteria.\n");
                            for (int k = 0; k < slave_count; ++k) {
                                cmds[k].controlword = CW_DISABLE_OPERATION;
                                cmds[k].target_vel  = 0;
                                cmds[k].target_tor  = 0;
                                acts[k].writeCommand(cmds[k]);
                            }
                            master.tickOnce();
                            break;
                        }

                        // Reset only the 10-min scheduler; keep the leg timer as-is
                        active_elapsed -= kBacklashEveryActive;
                        last_tick = std::chrono::steady_clock::now();
                    }
                }
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
