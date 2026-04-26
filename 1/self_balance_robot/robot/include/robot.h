#pragma once
// ═══════════════════════════════════════════════════════════════
//  Robot — Main Orchestrator with USB Hotplug
//  Auto-detects ODrive and SpeedyBee F405 via libusb VID/PID
//  Handles connect/disconnect/reconnect automatically
// ═══════════════════════════════════════════════════════════════

#include "types.h"
#include "usb_manager.h"
#include "odrive_usb.h"
#include "msp_imu_usb.h"
#include "balance_controller.h"
#include "tcp_server.h"
#include "logger.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace robot {

class Robot {
public:
    Robot();
    ~Robot();

    /// Initialize subsystems (USB manager, TCP, etc.)
    bool initialize(const Config& cfg);

    /// Run the main control loop (blocks until shutdown)
    void run();

    /// Request shutdown
    void shutdown();

    /// Check if still running
    bool is_running() const { return running_.load(); }

    SystemState get_state() const {
        return static_cast<SystemState>(state_.load());
    }

private:
    Config cfg_;
    std::atomic<int> state_{static_cast<int>(SystemState::INITIALIZING)};
    std::atomic<bool> running_{false};

    // USB
    UsbManager usb_mgr_;

    // Subsystems
    ODriveUsb        odrive_;
    MspImuUsb        imu_;
    BalanceController controller_;
    TcpServer        tcp_;

    // Device connection state
    std::atomic<bool> odrive_connected_{false};
    std::atomic<bool> imu_connected_{false};
    std::mutex reconnect_mtx_;   // guards device open/close

    // Status message for display on controller HUD
    std::mutex msg_mtx_;
    std::string status_msg_;
    std::chrono::steady_clock::time_point msg_time_;

    void set_status_msg(const std::string& msg) {
        std::lock_guard<std::mutex> lock(msg_mtx_);
        status_msg_ = msg;
        msg_time_ = std::chrono::steady_clock::now();
    }
    std::string get_status_msg() {
        std::lock_guard<std::mutex> lock(msg_mtx_);
        // Auto-clear after 5 seconds
        auto age = std::chrono::steady_clock::now() - msg_time_;
        if (std::chrono::duration_cast<std::chrono::seconds>(age).count() > 5) {
            status_msg_.clear();
        }
        return status_msg_;
    }

    // Threads
    std::thread telemetry_thread_;
    std::thread reconnect_thread_;  // periodic reconnect attempts

    MotorOutput last_output_;
    double enc_offset_l_ = 0, enc_offset_r_ = 0;  // encoder position at ARM time
    double yaw_offset_ = 0;                         // yaw reading at ARM time (zero point)
    std::atomic<double> enc_displacement_{0};       // average displacement from ARM point (turns)
    std::atomic<double> enc_pos_l_{0};               // left encoder position relative to ARM (turns)
    std::atomic<double> enc_pos_r_{0};               // right encoder position relative to ARM (turns, sign-corrected)
    std::atomic<double> enc_vel_l_{0};                 // measured left wheel velocity from ODrive (turns/s)
    std::atomic<double> enc_vel_r_{0};                 // measured right wheel velocity from ODrive (turns/s)
    std::atomic<double> enc_velocity_{0};              // average wheel velocity from ODrive (turns/s)
    int enc_tick_counter_ = 0;                      // for 100Hz encoder query in 200Hz loop
    CsvLogger logger_;
    uint64_t log_counter_ = 0;

    enum class PendingStateCmd : int { None = 0, Arm, Disarm, Estop };
    std::atomic<int> pending_state_cmd_{static_cast<int>(PendingStateCmd::None)};

    void set_state(SystemState s);

    // ── USB Hotplug ──────────────────────────────────────────

    /// Called by hotplug or reconnect thread
    void on_odrive_attached();
    void on_odrive_detached();
    void on_imu_attached();
    void on_imu_detached();

    /// Try to open and attach a device
    bool try_connect_odrive();
    bool try_connect_imu();

    /// Periodic reconnect loop (fallback if hotplug unavailable)
    void reconnect_loop();

    // ── Control ──────────────────────────────────────────────

    /// Initialize ODrive motors
    bool init_odrive_motors();

    /// Main control loop tick
    void control_tick(double dt);

    /// Handle arm/disarm/estop
    void handle_state_command(const std::string& cmd);

    /// Telemetry sender loop
    void telemetry_loop();

    void enter_fault(const std::string& reason);

    /// Check if both devices are ready
    bool devices_ready() const {
        return odrive_connected_.load() && imu_connected_.load();
    }
};

} // namespace robot
