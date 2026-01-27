# RAIPAL_SOEM
RAIPAL EtherCAT master for low-level arm control. This repo includes:
- `raipal_soem/`: the RAIPAL master application and examples.
- `soem/`: a vendored copy of SOEM (Simple Open EtherCAT Master).

---

## Features
- EtherCAT master demos built on SOEM
- PDO read/write examples and actuator control
- Ready-to-build with CMake

---

## Requirements
- Ubuntu 20.04+
- CMake >= 3.16
- GCC / G++ compiler
- yaml-cpp (for `demo`/`actuator_test`)
- SOEM installed and discoverable by CMake

---

## Install SOEM
This builds SOEM as a static library and installs headers + CMake package files.
By default it installs into `$HOME/.local`:
- Library: `$HOME/.local/lib/libsoem.a`
- Headers: `$HOME/.local/include/soem/`
- CMake package: `$HOME/.local/share/soem/cmake/soemConfig.cmake`

`raipal_soem/CMakeLists.txt` appends `$HOME/.local` to `CMAKE_PREFIX_PATH` and
uses `find_package(soem CONFIG REQUIRED)` to locate this package.

```bash
# cd to soem
cd <RAIPAL_SOEM_PROJECT_DIRECTORY>/soem

# Create build folder
mkdir build && cd build

# Run CMake
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_TESTS=OFF ..
cmake --build . -j
cmake --install .

```

---

## Build raipal_soem
The main application is `raipal_soem/src/main.cpp` (built as `demo`), which
connects to EtherCAT slaves and drives them based on `config.yaml`.

```bash
# cd to raipal_soem
cd <RAIPAL_SOEM_PROJECT_DIRECTORY>/raipal_soem

# Create build folder
mkdir build && cd build

# Build raipal_soem
cmake -S .. -B . 
cmake --build . -j

```

### Usage

```bash
# List available network adapters (no args)
./simple_test

# Print slave info; -map dumps PDO mappings
./slaveinfo <adapter> -map

# Minimal EtherCAT communication test (state changes + PDO exchange)
./simple_test <adapter>

# Run actuator control defined in config.yaml
./demo <adapter> <config.yaml>
```

### What the tools do
- `simple_test`: a minimal SOEM-based test; verifies NIC access and that slaves
  can reach OP state while exchanging process data.
- `slaveinfo`: enumerates slaves and shows SII/PDO configuration. Use `-map`
  to print PDO mapping for each slave (helps validate IO layout).

### How `main.cpp` uses `config.yaml`
`demo` expects one YAML entry per discovered slave in `config.yaml`:
```yaml
actuators:
  - name: joint_1
    mode: 8            # e.g., CSP = 0x08
    target_torque: 200
    target_velocity: 1000000
    target_position: 0
```
At runtime:
1. The master discovers slaves and enters OP state.
2. `config.yaml` is loaded; the number of `actuators` must match the slave count.
3. Each slave gets a mode and initial setpoints; commands update every 5 ms.
4. When a drive reaches “Operation Enabled,” the command for its mode is applied.

Other YAML sections (`backlash_test`, `friction_test`) are used by
`actuator_test`/`backlash_test`, not by `demo`.

## Actuator Bring-Up (from zero knowledge)
This project uses the `EthercatActuator` class (`raipal_soem/include/ethercat_actuator.hpp`)
to map PDOs and drive CiA-402 state transitions. Follow this sequence to safely
bring actuators online:

### 1) Verify the EtherCAT connection
Run `slaveinfo` first to confirm discovery and mapping:
```bash
./slaveinfo <adapter> -map
```
Check the output:
- The number of slaves reported must match the number of actuators physically connected.
- PDO mapping should match your actuator’s expected layout (targets, mode, controlword, feedback).

### 2) Commutation mode (MODE_PROFILE_COMMUTATION = 0x0F)
Before commanding motion, switch all actuators into commutation mode:
1. Edit `config.yaml` so every actuator uses `mode: 15` (0x0F).
2. Run the demo:
```bash
./demo <adapter> <config.yaml>
```
This step ensures each actuator is enabled and ready under a known drive mode.

### 3) Run target control (CSP/CSV/CST)
After commutation is verified, set your desired mode and targets:
- Allowed modes: **0x08 (CSP)**, **0x09 (CSV)**, **0x0A (CST)**
- Set the matching target field:
  - CSP (0x08): `target_position`
  - CSV (0x09): `target_velocity`
  - CST (0x0A): `target_torque`

Example:
```yaml
actuators:
  - name: joint_1
    mode: 8
    target_position: 0
  - name: joint_2
    mode: 9
    target_velocity: 500000
```
Important:
- The number of `actuators` entries **must equal** the number of connected slaves.
- Mismatched counts cause the demo to exit with a config error.
