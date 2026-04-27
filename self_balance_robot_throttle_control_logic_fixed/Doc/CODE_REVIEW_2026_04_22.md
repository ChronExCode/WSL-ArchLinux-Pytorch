# Code Review 2026-04-22

## Summary
This review focused on build hygiene, interface consistency, controller/robot parameter flow, telemetry safety, and control-path robustness.

## Fixed in this revision
- Removed unused `mul_diag_right()` helper from `robot/src/ekf.cpp`.
- Wired `json_escape(status_msg)` into telemetry output in `robot/src/tcp_server.cpp`.
- Added EKF parameters to the controller UI and to robot-side parameter application.

## High-priority findings still worth addressing
1. **Manual JSON parsing is fragile** in both controller and robot. Escaped strings and malformed payloads are not handled robustly.
2. **`json_get_string()` is not escape-aware** and will stop at the next raw quote.
3. **Telemetry assembly uses fixed-size `char buf[4096]`** and unchecked accumulation. Long messages or future field growth can truncate output silently.
4. **`TcpServer::send_to_client()` assumes one `send()` completes the entire payload**. A loop around partial sends would be safer.
5. **`Config` values are not centrally validated** after remote updates. Invalid but parseable values can still destabilize control.
6. **`poll()`/thread lifecycle is workable but not cancellation-safe under all races**. A future refactor to a single event loop would simplify shutdown.

## Medium-priority findings
- `WaylandApp::save_config()` writes JSON-like text without escaping string values.
- `main.cpp` forces `EVENT4 EVENT5 EVENT6`, which is intentional for this hardware, but it should be documented prominently in user docs and usage output.
- Many `.cpp` files are dense single-line implementations. Readability can be improved significantly by formatting and extracting helpers.

## Recommended next steps
1. Replace ad-hoc JSON parsing/serialization with a small JSON library.
2. Add `Config::sanitize()` and call it after every remote update.
3. Add structured logging / replay support.
4. Add unit tests for MPC model building, EKF predict/update, and parameter deserialization.
