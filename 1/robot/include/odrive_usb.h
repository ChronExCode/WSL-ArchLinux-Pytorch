#pragma once
// ═══════════════════════════════════════════════════════════════
//  ODrive v3.6 — libusb Bulk Transfer Driver
//  ASCII protocol over USB CDC bulk endpoints
//
//  Architecture (modeled after proven MSP/libusb pattern):
//    • One async bulk IN transfer, continuously re-submitted
//    • All writes use synchronous libusb_bulk_transfer
//    • Response lines parsed by newline delimiter
//    • send_query: sync write → wait for resp_ready_ flag
//    • Thread-safe via io_mtx_ (one write at a time)
// ═══════════════════════════════════════════════════════════════

#include "types.h"
#include "usb_manager.h"

#include <string>
#include <mutex>
#include <atomic>
#include <memory>

namespace robot {

class ODriveUsb {
public:
    ODriveUsb();
    ~ODriveUsb();

    ODriveUsb(const ODriveUsb&) = delete;
    ODriveUsb& operator=(const ODriveUsb&) = delete;

    /// Attach to an already-opened UsbDevice (takes ownership)
    bool attach(std::unique_ptr<UsbDevice> dev);

    /// Detach and release device
    void detach();

    /// Check if connected
    bool is_connected() const { return connected_.load(); }

    // ── Motor Control ────────────────────────────────────────

    /// Set both motors (unified sign: positive = forward)
    /// axis1 negation for mirror mount is handled internally
    bool set_velocities(double left_vel, double right_vel,
                        double left_ff = 0.0, double right_ff = 0.0);

    /// Set velocity control mode
    bool set_velocity_control(int axis);

    /// Set closed loop state
    bool set_closed_loop(int axis);

    /// Set idle state
    bool set_idle(int axis);

    /// Clear axis errors
    bool clear_errors(int axis);
    bool clear_all_errors();

    /// Emergency stop — best effort, no error check
    void emergency_stop();

    // ── Telemetry (query: write request, await response) ─────

    double get_position(int axis);
    double get_velocity(int axis);
    double get_vbus_voltage();
    int    get_axis_error(int axis);
    bool   is_closed_loop(int axis);

private:
    std::unique_ptr<UsbDevice> dev_;
    std::atomic<bool> connected_{false};

    // ── Single async read ────────────────────────────────────
    static constexpr int RX_BUF_SIZE = 512;
    uint8_t rx_buf_[RX_BUF_SIZE];
    std::atomic<bool> rx_running_{false};

    // Response handling
    std::mutex  resp_mtx_;
    std::string resp_partial_;       // accumulates bytes until newline
    std::string resp_line_;          // complete response line
    std::atomic<bool> resp_ready_{false};

    void submit_async_read();
    void on_rx_data(const uint8_t* data, int length);

    // ── IO serialization ─────────────────────────────────────
    std::mutex io_mtx_;  // ensures one USB transaction at a time

    /// Synchronous bulk write
    bool sync_write(const std::string& data, int timeout_ms = 100);

    /// Send command (no response expected)
    bool send_command(const std::string& cmd);

    /// Send query and wait for response line
    std::string send_query(const std::string& cmd, int timeout_ms = 200);
};

} // namespace robot
