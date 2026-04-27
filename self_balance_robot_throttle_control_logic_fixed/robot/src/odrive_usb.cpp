// ═══════════════════════════════════════════════════════════════
//  ODrive v3.6 — libusb Bulk Transfer Implementation
//
//  Architecture (same pattern as the proven MSP reader):
//    • One async bulk IN read, re-submitted in callback
//    • sync bulk_write for all outgoing commands
//    • send_query: write → poll resp_ready_ → return response
//    • All writes serialized by io_mtx_
//    • No fire-and-forget async writes — everything is synchronous
// ═══════════════════════════════════════════════════════════════

#include "odrive_usb.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

namespace robot {

ODriveUsb::ODriveUsb() = default;

ODriveUsb::~ODriveUsb() {
    detach();
}

// ── Attach / Detach ─────────────────────────────────────────

bool ODriveUsb::attach(std::unique_ptr<UsbDevice> dev) {
    if (!dev || !dev->is_open()) return false;

    detach();  // clean up previous connection

    dev_ = std::move(dev);

    // CDC init: set line coding, then DTR/RTS toggle
    dev_->set_line_coding(115200);
    dev_->set_control_line_state(false, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dev_->set_control_line_state(true, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Start the single async read
    rx_running_.store(true);
    submit_async_read();

    connected_.store(true);
    std::cout << "[ODrive] Attached: " << dev_->info() << "\n";
    return true;
}

void ODriveUsb::detach() {
    connected_.store(false);
    rx_running_.store(false);

    if (dev_) {
        dev_->cancel_async();
        // Small delay for async cancellation to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Lock io_mtx_ to wait for any in-progress IO to finish
    {
        std::lock_guard<std::mutex> lock(io_mtx_);
        dev_.reset();
    }

    // Clear response state
    {
        std::lock_guard<std::mutex> lock(resp_mtx_);
        resp_partial_.clear();
        resp_line_.clear();
        resp_ready_.store(false);
    }

    std::cout << "[ODrive] Detached\n";
}

// ── Single Async Read ───────────────────────────────────────
//
// Modeled after user's proven libusb example:
//   • One async bulk read, re-submitted on completion
//   • Callback parses received data into response lines
//   • Sets resp_ready_ flag when a complete line arrives

void ODriveUsb::submit_async_read() {
    if (!dev_ || !rx_running_.load() || dev_->is_detaching()) return;

    dev_->async_read(
        rx_buf_, RX_BUF_SIZE,
        [this](const uint8_t* data, int length, int status) {
            if (status == 0 /* LIBUSB_TRANSFER_COMPLETED */ && length > 0) {
                on_rx_data(data, length);
            }

            // Re-submit on non-fatal status
            // COMPLETED=0, ERROR=1, TIMED_OUT=2 → re-submit
            // CANCELLED=3, NO_DEVICE=5 → stop
            if (status <= 2 && rx_running_.load() && dev_ && !dev_->is_detaching()) {
                submit_async_read();
            }
        },
        0  // no timeout
    );
}

void ODriveUsb::on_rx_data(const uint8_t* data, int length) {
    std::lock_guard<std::mutex> lock(resp_mtx_);

    for (int i = 0; i < length; ++i) {
        char c = static_cast<char>(data[i]);
        if (c == '\n') {
            if (!resp_partial_.empty()) {
                // Complete line received — store it and set flag
                resp_line_ = std::move(resp_partial_);
                resp_partial_.clear();
                resp_ready_.store(true);
            }
        } else if (c != '\r') {
            resp_partial_ += c;
        }
    }
}

// ── IO Primitives ───────────────────────────────────────────

bool ODriveUsb::sync_write(const std::string& data, int timeout_ms) {
    if (!dev_ || !dev_->is_open() || !connected_.load()) return false;

    int n = dev_->bulk_write(
        reinterpret_cast<const uint8_t*>(data.data()),
        static_cast<int>(data.size()),
        timeout_ms);

    return n == static_cast<int>(data.size());
}

bool ODriveUsb::send_command(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(io_mtx_);
    return sync_write(cmd + "\n");
}

std::string ODriveUsb::send_query(const std::string& cmd, int timeout_ms) {
    std::lock_guard<std::mutex> lock(io_mtx_);

    // Clear any stale response
    resp_ready_.store(false);

    // Send query
    if (!sync_write(cmd + "\n")) return {};

    // Wait for response (poll resp_ready_ flag, like user's `while(!next)` pattern)
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (!resp_ready_.load()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            return {};  // timeout
        }
        // Brief sleep to avoid busy-spin, but responsive enough for 200Hz control
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    // Grab the response
    std::lock_guard<std::mutex> rlock(resp_mtx_);
    resp_ready_.store(false);
    return resp_line_;
}

// ── Motor Control ───────────────────────────────────────────

bool ODriveUsb::set_velocities(double left_vel, double right_vel,
                                double left_ff, double right_ff) {
    std::lock_guard<std::mutex> lock(io_mtx_);
    char buf[256];
    // axis1 is mirror-mounted: negate right velocity at ODrive IO boundary
    snprintf(buf, sizeof(buf), "v 0 %.4f %.4f\nv 1 %.4f %.4f\n",
             left_vel, left_ff, -right_vel, right_ff);
    return sync_write(buf);
}

bool ODriveUsb::set_velocity_control(int axis) {
    char buf[128];
    snprintf(buf, sizeof(buf), "w axis%d.controller.config.control_mode 2", axis);
    return send_command(buf);
}

bool ODriveUsb::set_closed_loop(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "w axis%d.requested_state 8", axis);
    return send_command(buf);
}

bool ODriveUsb::set_idle(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "w axis%d.requested_state 1", axis);
    return send_command(buf);
}

bool ODriveUsb::clear_errors(int axis) {
    char buf[64];
    const int delay_ms = 10;

    snprintf(buf, sizeof(buf), "w axis%d.error 0", axis);
    if (!send_command(buf)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    snprintf(buf, sizeof(buf), "w axis%d.motor.error 0", axis);
    if (!send_command(buf)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    snprintf(buf, sizeof(buf), "w axis%d.encoder.error 0", axis);
    if (!send_command(buf)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    snprintf(buf, sizeof(buf), "w axis%d.controller.error 0", axis);
    if (!send_command(buf)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    return true;
}

bool ODriveUsb::clear_all_errors() {
    if (!clear_errors(0)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return clear_errors(1);
}

void ODriveUsb::emergency_stop() {
    // Best-effort, but still serialize with normal ODrive IO.  This avoids
    // concurrent access to dev_ while another thread is in send_query(),
    // set_velocities(), or detach().  Use short USB timeouts so fault handling
    // does not block for long.
    std::lock_guard<std::mutex> lock(io_mtx_);
    if (!dev_ || !dev_->is_open()) return;

    // Stop velocity (28 bytes)
    const char* stop_vel = "v 0 0.0 0.0\nv 1 0.0 0.0\n";
    dev_->bulk_write(reinterpret_cast<const uint8_t*>(stop_vel),
                     static_cast<int>(strlen(stop_vel)), 20);

    // Set idle state (48 bytes)
    const char* set_idle = "w axis0.requested_state 1\nw axis1.requested_state 1\n";
    dev_->bulk_write(reinterpret_cast<const uint8_t*>(set_idle),
                     static_cast<int>(strlen(set_idle)), 20);
}

// ── Telemetry ───────────────────────────────────────────────

double ODriveUsb::get_position(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "r axis%d.encoder.pos_estimate", axis);
    std::string resp = send_query(buf);
    if (resp.empty()) return 0.0;
    try { 
			if (axis == 0)
			{
				return std::stod(resp); 
			}
			else
			{
				return -std::stod(resp); 
			}
	} catch (...) { return 0.0; }
}

double ODriveUsb::get_velocity(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "r axis%d.encoder.vel_estimate", axis);
    std::string resp = send_query(buf);
    if (resp.empty()) return 0.0;
    try {
        if (axis == 0)
        {
            return std::stod(resp);
        }
        else
        {
            return -std::stod(resp);
        }
    } catch (...) { return 0.0; }
}

double ODriveUsb::get_vbus_voltage() {
    std::string resp = send_query("r vbus_voltage");
    if (resp.empty()) return 0.0;
    try { return std::stod(resp); } catch (...) { return 0.0; }
}

int ODriveUsb::get_axis_error(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "r axis%d.error", axis);
    std::string resp = send_query(buf);
    if (resp.empty()) return -1;
    try { return std::stoi(resp); } catch (...) { return -1; }
}

bool ODriveUsb::is_closed_loop(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "r axis%d.current_state", axis);
    std::string resp = send_query(buf);
    if (resp.empty()) return false;
    try { return std::stoi(resp) == 8; } catch (...) { return false; }
}

} // namespace robot
