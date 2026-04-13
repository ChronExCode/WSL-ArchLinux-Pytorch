#pragma once
// ═══════════════════════════════════════════════════════════════
//  ODrive v3.5 — libusb Bulk Transfer Driver
//  ASCII protocol over USB CDC bulk endpoints
//  Uses async write + sync read for command/response pattern
// ═══════════════════════════════════════════════════════════════

#include "types.h"
#include "usb_manager.h"

#include <string>
#include <mutex>
#include <memory>
#include <deque>
#include <condition_variable>

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
    bool is_connected() const { return dev_ && dev_->is_open(); }

    /// Start async RX listener (fills response queue)
    void start_rx_listener();

    /// Stop RX listener
    void stop_rx_listener();

    // ── Motor Control (fire-and-forget async writes) ─────────

    /// Set velocity for axis (0=left, 1=right)
    bool set_velocity(int axis, double velocity, double torque_ff = 0.0);

    /// Set both motors in one call
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

    /// Emergency stop
    void emergency_stop();

    /// Run calibration
    bool run_calibration(int axis);

    // ── Telemetry (sync query: write request, await response) ─

    double get_velocity(int axis);
    double get_position(int axis);
    double get_vbus_voltage();
    double get_current(int axis);
    int    get_axis_error(int axis);
    bool   is_closed_loop(int axis);

private:
    std::unique_ptr<UsbDevice> dev_;
    std::mutex io_mtx_;  // serializes command/response pairs

    // Async RX response queue
    std::mutex rx_mtx_;
    std::condition_variable rx_cv_;
    std::deque<std::string> rx_queue_;
    std::atomic<bool> rx_running_{false};

    // RX buffer pool for async reads
    static constexpr int RX_BUF_SIZE = 512;
    static constexpr int RX_QUEUE_DEPTH = 4;
    struct RxSlot {
        uint8_t buffer[RX_BUF_SIZE];
    };
    RxSlot rx_slots_[RX_QUEUE_DEPTH];
    std::string rx_partial_;  // partial line accumulator

    /// Send command (no response expected) — async write
    bool send_command(const std::string& cmd);

    /// Send query and wait for response line — sync pattern
    std::string send_query(const std::string& cmd, int timeout_ms = 100);

    /// Process received data into line queue
    void on_rx_data(const uint8_t* data, int length);

    /// Submit one async read
    void submit_async_read(int slot_index);
};

} // namespace robot
