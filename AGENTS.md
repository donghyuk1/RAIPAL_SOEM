# Repository Guidelines

## Project Structure & Module Organization
- `raipal_soem/` contains the RAIPAL EtherCAT master code.
  - `raipal_soem/src/`: C/C++ sources (e.g., `ethercat_master.cpp`, `actuator_test.cpp`, `simple_test*.c`).
  - `raipal_soem/include/`: public headers.
  - `raipal_soem/output/`: runtime outputs such as CSV results.
  - `raipal_soem/build/`: out-of-tree build directory (generated).
- `soem/` is a vendored SOEM dependency with its own `CMakeLists.txt`, `soem/`, `osal/`, `oshw/`, and `test/`.
- Root `README.md` documents the primary build and run flows.

## Build, Test, and Development Commands
- Build SOEM (installs to `$HOME/.local` by default):
  ```bash
  cd soem
  mkdir -p build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_TESTS=OFF ..
  cmake --build . -j
  cmake --install .
  ```
- Build RAIPAL master:
  ```bash
  cd raipal_soem
  mkdir -p build && cd build
  cmake -S .. -B .
  cmake --build . -j
  ```
- Run examples (from `raipal_soem/build/`):
  - `./simple_test <adapter>`, `./slaveinfo <adapter> -map`, `./demo <adapter>`

## Coding Style & Naming Conventions
- Languages: C and C++ (CMake build).
- Indentation is 4 spaces; braces follow existing style in file (C files and C++ methods both use K&R-like layout with braces on next line).
- Keep naming consistent with existing patterns: `snake_case` for C functions/files (e.g., `simple_test.c`), `CamelCase` for C++ classes (e.g., `EthercatMaster`).
- No formatter or linter is configured; avoid large style-only diffs.

## Testing Guidelines
- There is no unit-test framework in `raipal_soem`.
- SOEM provides sample test executables under `soem/test/` (e.g., `test/simple_ng`, `test/linux/simple_test`).
  - When building SOEM standalone, `BUILD_TESTS` defaults to ON; tests are built as binaries, not CTest targets.
  - Run the generated executables directly from the SOEM build tree.

## Commit & Pull Request Guidelines
- Recent commits use short, descriptive subjects without prefixes (e.g., “actuator test …”).
- Keep commit titles concise, lowercase, and focused on a single change.
- PRs should include: purpose, hardware/network assumptions (NIC name, slave type), and run instructions; add screenshots/log snippets for test runs when applicable.

## Configuration & Safety Notes
- Hardware-specific configuration lives in `raipal_soem/src/config.yaml`; keep test parameters and output paths updated.
- EtherCAT tests interact with real hardware; call out any changes that affect device states or motion control.
