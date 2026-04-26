# Runtime Config, Logging, Replay, and Test Notes

## What changed

This update adds three infrastructure features intended to improve robustness and maintainability:

1. A lightweight JSON helper (`common/mini_json.h`) shared by `controller` and `robot`
2. Centralized runtime configuration sanitization and validation in `robot::Config`
3. CSV logging, replay tooling, and a minimal test target

## Lightweight JSON helper

File: `common/mini_json.h`

Provided functionality:
- `minijson::escape()` / `unescape()`
- `minijson::get_number_or()`
- `minijson::get_string_or()`
- `minijson::get_bool()`
- `minijson::ObjectBuilder`

Current usage:
- Controller parameter serialization
- Controller config persistence
- Robot TCP command parsing
- Robot telemetry JSON generation

Important limitation:
- This is a deliberately small helper for flat JSON objects used by this project.
- It is not a general-purpose standards-complete JSON parser.
- If the protocol grows to nested objects / arrays, replace it with a real JSON library.

## Config sanitize / validate

File: `robot/include/types.h`

New methods:
- `Config::sanitize()`
- `Config::validate(std::string* reason)`

Behavior:
- `sanitize()` clamps dangerous or non-finite values into safe ranges.
- `validate()` checks cross-field invariants and reports a reason string.

Current integration points:
- Robot initialization sanitizes and validates the startup config
- Remote parameter updates sanitize and validate before applying to the controller

Recommended future extension:
- Add a `Config::diff()` helper for auditing remote changes
- Add a `Config::to_json()` helper for export / snapshots

## CSV logging

Files:
- `robot/include/logger.h`
- `robot/src/logger.cpp`

Current behavior:
- Logs a CSV file into `logs/`
- Default filename format: `run_YYYYMMDD_HHMMSS.csv`
- Logs are recorded from the robot side during balancing

Current fields include:
- time
- state
- IMU pitch / roll / yaw
- gyro / acceleration subset
- remote command values
- target pitch / filtered pitch / target speed
- displacement / command outputs
- NMPC usage and cost

Current config fields:
- `logging_enable`
- `logging_decimation`

## Replay tool

File: `tools/replay_csv.py`

Usage:

```bash
python tools/replay_csv.py logs/run_YYYYMMDD_HHMMSS.csv
python tools/replay_csv.py logs/run_YYYYMMDD_HHMMSS.csv --plot
```

Behavior:
- Always prints summary statistics
- Optionally plots pitch / target pitch / velocity if `matplotlib` is installed

## Test target

Files:
- `robot/tests/test_json_config.cpp`
- `robot/CMakeLists.txt`

What is tested now:
- JSON string parsing / unescaping for the small shared helper
- Basic `Config::sanitize()` / `Config::validate()` invariants

Build notes:
- The test target is enabled through CTest in the robot CMake file
- It does not depend on libusb or the hardware drivers

Typical commands:

```bash
cd robot/build
cmake ..
make -j
ctest --output-on-failure
```

## Recommended debug workflow

1. Run robot with logging enabled
2. Perform a short controlled balancing test
3. Inspect the CSV with `tools/replay_csv.py`
4. Tune one parameter family at a time
5. Re-run the test and compare traces

## Why these changes matter

These changes improve the project in ways that are more important than adding yet another control feature:
- Safer runtime parameter updates
- More reliable text protocol handling
- Better post-run visibility through logs
- A starting point for automated regression testing

These are foundational steps toward an industrial-grade control software stack.
