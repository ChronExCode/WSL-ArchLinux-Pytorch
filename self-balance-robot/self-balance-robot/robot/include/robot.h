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

#include <atomic>
#include <thread>
#include <mutex>

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

    // Threads
    std::thread telemetry_thread_;
    std::thread reconnect_thread_;  // periodic reconnect attempts

    MotorOutput last_output_;
    double enc_offset_l_ = 0, enc_offset_r_ = 0;  // encoder position at ARM time
    std::atomic<double> enc_displacement_{0};       // average displacement from ARM point (turns)

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
