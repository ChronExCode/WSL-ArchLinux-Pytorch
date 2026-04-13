#pragma once
// ═══════════════════════════════════════════════════════════════
//  Self-Balance Robot — Common Types & Configuration
//  Target: Raspberry Pi 5 + ODrive v3.5 + SpeedyBee F405 V5
//  USB:    Direct libusb bulk transfers, no /dev/ttyACM*
// ═══════════════════════════════════════════════════════════════

#include <cstdint>
#include <cmath>
#include <string>
#include <atomic>
#include <chrono>
#include <functional>

namespace robot {

// ── USB Device Identifiers ───────────────────────────────────
//  ODrive v3.5 (native USB CDC):  VID=0x1209  PID=0x0D32
//  SpeedyBee F405 V5 (STM32 VCP): VID=0x0483  PID=0x5740
struct UsbDeviceId {
    uint16_t vid;
    uint16_t pid;
    const char* name;
};

static constexpr UsbDeviceId ODRIVE_USB_ID = { 0x1209, 0x0D32, "ODrive v3.5" };
static constexpr UsbDeviceId F405_USB_ID   = { 0x0483, 0x5740, "SpeedyBee F405 V5" };

// CDC ACM class/subclass/protocol for interface matching
static constexpr uint8_t USB_CLASS_CDC_DATA = 0x0A;
static constexpr uint8_t USB_CLASS_CDC      = 0x02;
static constexpr uint8_t USB_SUBCLASS_ACM   = 0x02;

// ── Physical & Tuning Constants ──────────────────────────────
struct Config {
    // PID gains for pitch balance (inner loop: angle → motor velocity)
    double pitch_kp         = 0.30;
    double pitch_ki         = 0.005;
    double pitch_kd         = 0.025;

    // PID gains for yaw (heading lock)
    double yaw_kp           = 1.0;
    double yaw_ki           = 0.0;
    double yaw_kd           = 0.0;

    // Outer loop: speed → target pitch (anti-drift)
    double speed_kp         = 0.6;
    double speed_ki         = 0.0;
    double speed_kd         = 0.0;

    // Complementary filter alpha (0..1, higher = trust gyro more)
    double comp_filter_alpha = 0.98;

    // Balance setpoint offsets
    double pitch_offset     = 0.0;   // degrees, mechanical CoG trim

    // Motor limits
    double max_velocity     = 2.0;   // turns/s ODrive velocity units (safety limit)
    double max_torque       = 6.0;   // Nm

    // Deadband for joystick input
    double joystick_deadband = 0.05;

    // Control loop frequency
    int    control_rate_hz  = 200;

    // Speed ramp rate (units/s per second)
    double speed_ramp_rate  = 10.0;

    // TCP server port for remote control
    uint16_t tcp_port       = 9000;

    // IMU MSP request interval (microseconds)
    // Both ATTITUDE and RAW_IMU are requested every cycle.
    // 1000μs (1kHz) gives ~5 fresh samples per 200Hz control tick.
    int msp_request_interval_us = 1000;  // 1000 Hz

    // Safety: max tilt angle before motor cutoff (degrees)
    double max_tilt_angle   = 45.0;

    // Wheel geometry
    double wheel_base_m     = 0.45;  // distance between wheels

    // USB async transfer settings
    int usb_bulk_timeout_ms = 50;
    int usb_rx_queue_depth  = 4;     // number of concurrent async RX transfers
};

// ── IMU Data ─────────────────────────────────────────────────
struct ImuData {
    double pitch = 0.0;
    double roll  = 0.0;
    double yaw   = 0.0;

    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;

    double acc_x = 0.0;
    double acc_y = 0.0;
    double acc_z = 0.0;

    std::chrono::steady_clock::time_point timestamp;
    bool valid = false;
};

// ── Remote Control Command ───────────────────────────────────
struct RemoteCommand {
    double throttle    = 0.0;
    double steering    = 0.0;
    double speed_limit = 1.0;
    bool   emergency_stop = false;
    bool   arm         = false;

    std::chrono::steady_clock::time_point timestamp;
};

// ── Motor Output ─────────────────────────────────────────────
struct MotorOutput {
    double left_velocity  = 0.0;
    double right_velocity = 0.0;
};

// ── PID Debug Data (for telemetry) ──────────────────────────
struct PidDebugData {
    double target_pitch    = 0;   // pitch setpoint
    double filtered_pitch  = 0;   // actual pitch measurement
    double pitch_error     = 0;   // target_pitch - filtered_pitch
    double pitch_p = 0, pitch_i = 0, pitch_d = 0;  // control terms
    double pitch_output    = 0;   // total motor command (p+i+d)
    double target_speed    = 0;   // desired speed from joystick
    double speed_estimate  = 0;   // estimated speed
    double speed_error     = 0;   // target_speed - speed_estimate
    double gyro_y          = 0;   // angular velocity (deg/s)
    double gyro_y_dot      = 0;   // angular acceleration (deg/s²)
    double pos_left        = 0;   // left encoder position (turns)
    double pos_right       = 0;   // right encoder position (turns)
};

// ── System State ─────────────────────────────────────────────
enum class SystemState : int {
    INITIALIZING = 0,
    WAITING_USB  = 1,   // waiting for USB devices
    CALIBRATING  = 2,
    IDLE         = 3,
    ARMED        = 4,
    BALANCING    = 5,
    FAULT        = 6,
    SHUTDOWN     = 7
};

inline const char* state_name(SystemState s) {
    switch (s) {
        case SystemState::INITIALIZING: return "INITIALIZING";
        case SystemState::WAITING_USB:  return "WAITING_USB";
        case SystemState::CALIBRATING:  return "CALIBRATING";
        case SystemState::IDLE:         return "IDLE";
        case SystemState::ARMED:        return "ARMED";
        case SystemState::BALANCING:    return "BALANCING";
        case SystemState::FAULT:        return "FAULT";
        case SystemState::SHUTDOWN:     return "SHUTDOWN";
    }
    return "UNKNOWN";
}

// ── Utility ──────────────────────────────────────────────────
inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
inline double deg_to_rad(double d) { return d * M_PI / 180.0; }
inline double rad_to_deg(double r) { return r * 180.0 / M_PI; }

inline double apply_deadband(double val, double band) {
    if (std::abs(val) < band) return 0.0;
    double sign = (val > 0) ? 1.0 : -1.0;
    return sign * (std::abs(val) - band) / (1.0 - band);
}

} // namespace robot
