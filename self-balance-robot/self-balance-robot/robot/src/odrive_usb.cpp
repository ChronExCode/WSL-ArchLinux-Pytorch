// ═══════════════════════════════════════════════════════════════
//  ODrive v3.5 — libusb Bulk Transfer Implementation
//
//  Architecture:
//    • Commands (set_velocity etc.) → async bulk OUT (fire-and-forget)
//    • Queries (get_velocity etc.) → sync bulk OUT + wait on RX queue
//    • Background: N async bulk IN transfers continuously receive data
//      into a line queue, parsed by newline delimiters
// ═══════════════════════════════════════════════════════════════

#include "odrive_usb.h"

#include <cstdio>
#include <cstring>
#include <iostream>

namespace robot {

ODriveUsb::ODriveUsb() = default;

ODriveUsb::~ODriveUsb() {
    detach();
}

bool ODriveUsb::attach(std::unique_ptr<UsbDevice> dev) {
    if (!dev || !dev->is_open()) return false;

    detach();
    dev_ = std::move(dev);

    // Set CDC line coding: 115200 8N1
    dev_->set_line_coding(115200);
    dev_->set_control_line_state(true, true);  // DTR=1, RTS=1

    std::cout << "[ODrive] Attached: " << dev_->info() << "\n";
    return true;
}

void ODriveUsb::detach() {
    stop_rx_listener();

    if (dev_) {
        // Try to idle motors before releasing
        if (dev_->is_open()) {
            dev_->write_string("w axis0.requested_state 1\n");
            dev_->write_string("w axis1.requested_state 1\n");
        }
        dev_.reset();
    }

    // Drain RX queue
    {
        std::lock_guard<std::mutex> lock(rx_mtx_);
        rx_queue_.clear();
        rx_partial_.clear();
    }

    std::cout << "[ODrive] Detached\n";
}

// ── Async RX Listener ────────────────────────────────────────

void ODriveUsb::on_rx_data(const uint8_t* data, int length) {
    std::lock_guard<std::mutex> lock(rx_mtx_);

    for (int i = 0; i < length; ++i) {
        char c = static_cast<char>(data[i]);
        if (c == '\n') {
            if (!rx_partial_.empty()) {
                rx_queue_.push_back(std::move(rx_partial_));
                rx_partial_.clear();
                rx_cv_.notify_one();
            }
        } else if (c != '\r') {
            rx_partial_ += c;
        }
    }

    // Prevent unbounded queue growth
    while (rx_queue_.size() > 64) {
        rx_queue_.pop_front();
    }
}

void ODriveUsb::submit_async_read(int slot_index) {
    if (!dev_ || !rx_running_.load()) return;

    dev_->async_read(
        rx_slots_[slot_index].buffer, RX_BUF_SIZE,
        [this, slot_index](const uint8_t* data, int length) {
            on_rx_data(data, length);
            // Re-submit this slot for continuous reading
            submit_async_read(slot_index);
        },
        0  // no timeout — wait indefinitely for data
    );
}

void ODriveUsb::start_rx_listener() {
    if (rx_running_.load() || !dev_) return;
    rx_running_.store(true);

    // Submit multiple async reads for pipeline depth
    for (int i = 0; i < RX_QUEUE_DEPTH; ++i) {
        submit_async_read(i);
    }

    std::cout << "[ODrive] RX listener started (" << RX_QUEUE_DEPTH
              << " async transfers)\n";
}

void ODriveUsb::stop_rx_listener() {
    rx_running_.store(false);
    if (dev_) {
        dev_->cancel_async();
    }
}

// ── Command / Query ──────────────────────────────────────────

bool ODriveUsb::send_command(const std::string& cmd) {
    if (!dev_ || !dev_->is_open()) return false;

    std::string data = cmd + "\n";

    // Fire-and-forget async write
    return dev_->async_write(
        reinterpret_cast<const uint8_t*>(data.data()),
        static_cast<int>(data.size()),
        [](int status, int) {
            if (status != 0) {
                // LIBUSB_TRANSFER_COMPLETED == 0
                std::cerr << "[ODrive] Async write failed, status="
                          << status << "\n";
            }
        },
        50  // 50ms timeout
    );
}

std::string ODriveUsb::send_query(const std::string& cmd, int timeout_ms) {
    std::lock_guard<std::mutex> io_lock(io_mtx_);

    // Drain stale responses
    {
        std::lock_guard<std::mutex> lock(rx_mtx_);
        rx_queue_.clear();
    }

    // Send query synchronously (to ensure ordering)
    if (!dev_ || !dev_->is_open()) return {};
    std::string data = cmd + "\n";
    if (dev_->bulk_write(reinterpret_cast<const uint8_t*>(data.data()),
                         static_cast<int>(data.size()), 50) < 0) {
        return {};
    }

    // Wait for response
    std::unique_lock<std::mutex> lock(rx_mtx_);
    if (rx_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                        [this] { return !rx_queue_.empty(); })) {
        std::string resp = rx_queue_.front();
        rx_queue_.pop_front();
        return resp;
    }

    return {};  // timeout
}

// ── Motor Control ────────────────────────────────────────────

bool ODriveUsb::set_velocity(int axis, double velocity, double torque_ff) {
    char buf[128];
    snprintf(buf, sizeof(buf), "v %d %.4f %.4f", axis, velocity, torque_ff);
    return send_command(buf);
}

bool ODriveUsb::set_velocities(double left_vel, double right_vel,
                                double left_ff, double right_ff) {
    char buf[256];
    snprintf(buf, sizeof(buf), "v 0 %.4f %.4f\nv 1 %.4f %.4f",
             left_vel, left_ff, right_vel, right_ff);

    if (!dev_ || !dev_->is_open()) return false;
    std::string data(buf);
    data += "\n";

    return dev_->async_write(
        reinterpret_cast<const uint8_t*>(data.data()),
        static_cast<int>(data.size()),
        nullptr,  // no completion callback needed
        50);
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
    snprintf(buf, sizeof(buf), "w axis%d.error 0", axis);
    send_command(buf);
    snprintf(buf, sizeof(buf), "w axis%d.motor.error 0", axis);
    send_command(buf);
    snprintf(buf, sizeof(buf), "w axis%d.encoder.error 0", axis);
    send_command(buf);
    snprintf(buf, sizeof(buf), "w axis%d.controller.error 0", axis);
    return send_command(buf);
}

bool ODriveUsb::clear_all_errors() {
    return clear_errors(0) && clear_errors(1);
}

void ODriveUsb::emergency_stop() {
    // Use sync writes for E-stop — must be reliable
    if (!dev_ || !dev_->is_open()) return;

    const char* cmds[] = {
        "v 0 0.0 0.0\n",
        "v 1 0.0 0.0\n",
        "w axis0.requested_state 1\n",
        "w axis1.requested_state 1\n"
    };

    for (auto& cmd : cmds) {
        dev_->bulk_write(reinterpret_cast<const uint8_t*>(cmd),
                         static_cast<int>(strlen(cmd)), 20);
    }
}

bool ODriveUsb::run_calibration(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "w axis%d.requested_state 3", axis);
    return send_command(buf);
}

// ── Telemetry ────────────────────────────────────────────────

double ODriveUsb::get_velocity(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "r axis%d.encoder.vel_estimate", axis);
    std::string resp = send_query(buf);
    if (resp.empty()) return 0.0;
    try { return std::stod(resp); } catch (...) { return 0.0; }
}

double ODriveUsb::get_position(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "r axis%d.encoder.pos_estimate", axis);
    std::string resp = send_query(buf);
    if (resp.empty()) return 0.0;
    try { return std::stod(resp); } catch (...) { return 0.0; }
}

double ODriveUsb::get_vbus_voltage() {
    std::string resp = send_query("r vbus_voltage");
    if (resp.empty()) return 0.0;
    try { return std::stod(resp); } catch (...) { return 0.0; }
}

double ODriveUsb::get_current(int axis) {
    char buf[64];
    snprintf(buf, sizeof(buf), "r axis%d.motor.current_control.Iq_measured", axis);
    std::string resp = send_query(buf);
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
