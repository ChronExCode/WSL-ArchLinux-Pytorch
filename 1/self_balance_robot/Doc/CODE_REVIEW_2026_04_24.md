# Code Review 2026-04-24

## Scope

Reviewed the uploaded `self_balance_robot` project with emphasis on build consistency, runtime safety, parameter/logging traceability, and documentation accuracy.

## Changes made in this revision

### Build and executable naming

The public docs and helper scripts expect these executable names:

- Raspberry Pi robot process: `robot`
- MSI Claw controller process: `robot_controller`

CMake previously produced targets named `robot` and `controller`. This revision keeps the internal target names but sets explicit output names in both CMake projects, so the generated binaries match the README, Zed tasks, and `build.sh` output.

Files changed:

- `robot/CMakeLists.txt`
- `controller/CMakeLists.txt`

### Logging reproducibility

`CsvLogger` already had `log_config_snapshot()`, but startup never called it. This revision writes the initial config snapshot immediately after logger startup succeeds. That makes a CSV log more useful for replay and tuning because the run parameters are captured before the first control samples.

File changed:

- `robot/src/robot.cpp`

### README cleanup

The README had a wrong controller build path and a malformed HTML-style heading. This revision fixes the controller build command to use `cd controller`, documents the expected binary output names, and adds the root-level `./build.sh Release` path.

File changed:

- `README.md`

## Findings not changed in code yet

### 1. Build verification is blocked in this environment

The local environment used for review does not have the required development packages installed:

- `libusb-1.0` for `robot/`
- `wayland-client` for `controller/`

Because of that, I could verify CMake reaches dependency checks, but I could not complete a full compile here. On the target machines, run:

```bash
./build.sh Release
```

or build each side separately as shown in the README.

### 2. Signal handler is not async-signal-safe

`robot/src/main.cpp` calls C++ I/O and `Robot::shutdown()` from a POSIX signal handler. This is common in prototypes, but it is not async-signal-safe. A safer pattern is to set an atomic shutdown flag or write to an `eventfd`, then let the main loop perform shutdown.

Priority: medium, because the current version is probably usable but can deadlock or behave unexpectedly under unlucky signal timing.

### 3. Command-line parsing should validate ranges before casting

`--tcp-port` uses `std::stoi()` and then casts to `uint16_t`. Negative or oversized values can wrap. Recommended fix: parse to `int`, validate `1..65535`, then cast.

Priority: medium.

### 4. Remote config parsing is still hand-written

The robot-side parameter callback uses string search plus `std::stod()`. It is acceptable for a trusted, flat, local protocol, but it is fragile for malformed input and difficult to audit as the parameter list grows. The next improvement should be a small typed config parser or a generated table shared by UI, robot config, logging, and docs.

Priority: high for maintainability, medium for immediate runtime risk if the network is trusted.

### 5. Controller UI parameter list is still duplicated by hand

`WaylandApp::initialize_params()` manually repeats keys, labels, ranges, and groups. This is easy to drift from `Config`, logging, and robot-side apply logic. Recommended long-term fix: make a single parameter descriptor table and generate the UI, parsing, validation, logging, and docs from it.

Priority: medium.

## Recommended next steps

1. Build on the actual Raspberry Pi and MSI Claw machines after installing dependencies.
2. Add a tiny unit or smoke test for config JSON round-trip and `Config::sanitize()/validate()`.
3. Replace the long remote-config lambda with table-driven field application.
4. Replace signal-handler shutdown with atomic/eventfd-driven shutdown.
5. Add CI or a simple `scripts/check_build.sh` that configures both CMake projects and fails on target-name/doc drift.
