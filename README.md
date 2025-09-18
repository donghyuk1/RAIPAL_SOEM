# RAIPAL_SOEM
RAIPAL ethercat master for low level arm control
This project demonstrates how to connect to EtherCAT slaves, send commands, and read process data.

---

## Features
- Simple EtherCAT master example with SOEM
- Demonstrates reading and writing PDO data
- Ready-to-build with CMake

---

## Requirements
- Ubuntu 20.04+
- CMake >= 3.16
- GCC / G++ compiler
- SOEM library (installed and available in CMAKE_PREFIX_PATH)

---

## Install SOEM
This will install SOEM to your machine

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
# Check your network adapters
./simple_test

# Slave info (PDO mappings)
./slaveinfo <adapter> -map

# Simple ethercat communication
./simple_test <adapter>

# Run the motor 
./demo <adapter>
```
