#pragma once
// ═══════════════════════════════════════════════════════════════
//  Self-Balance Robot — Common Types & Configuration
//  Target: Raspberry Pi 5 + ODrive v3.5 + SpeedyBee F405 V5
// ═══════════════════════════════════════════════════════════════

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <algorithm>
#include <sstream>
#include <string>

namespace robot {

struct UsbDeviceId {
    uint16_t vid;
    uint16_t pid;
    const char* name;
};

static constexpr UsbDeviceId ODRIVE_USB_ID = {0x1209, 0x0D32, "ODrive v3.5"};
static constexpr UsbDeviceId F405_USB_ID = {0x0483, 0x5740, "SpeedyBee F405 V5"};

static constexpr uint8_t USB_CLASS_CDC_DATA = 0x0A;
static constexpr uint8_t USB_CLASS_CDC = 0x02;
static constexpr uint8_t USB_SUBCLASS_ACM = 0x02;

struct Config {
    double pitch_kp = 0.30;
    double pitch_ki = 0.005;
    double pitch_kd = 0.025;
    double pitch_d_alpha = 0.2;
    double yaw_kp = 1.0;
    double yaw_ki = 0.0;
    double yaw_kd = 0.0;
    double speed_kp = 0.6;
    double speed_ki = 0.0;
    double speed_kd = 0.0;
    double pos_kp = 0.5;
    double pos_ki = 0.1;
    double pos_kd = 0.2;

    double pos_out_max_fwd = 5.0;
    double pos_out_max_bwd = -5.0;
    double target_pitch_max_fwd = 15.0;
    double target_pitch_max_bwd = -15.0;

    double comp_filter_alpha = 0.98;
    double pitch_lpf_alpha = 0.50;
    double wheel_velocity_lpf_alpha = 0.25;
    double command_lpf_alpha = 0.18;
    double steering_lpf_alpha = 0.20;

    double pitch_offset = 0.0;
    double max_velocity = 2.0;
    double max_torque = 6.0;
    double joystick_deadband = 0.05;
    int control_rate_hz = 200;
    double speed_ramp_rate = 10.0;
    uint16_t tcp_port = 9000;
    int msp_request_interval_us = 1000;
    double max_tilt_angle = 45.0;
    int rc_timeout_ms = 500;
    double wheel_base_m = 0.45;
    int usb_bulk_timeout_ms = 50;
    int usb_rx_queue_depth = 4;
    bool logging_enable = true;
    int logging_decimation = 4;

    bool adaptive_balance_enable = true;
    double adaptive_balance_rate = 0.08;
    double adaptive_balance_max_trim_deg = 6.0;
    double adaptive_balance_pitch_window_deg = 8.0;
    double adaptive_balance_gyro_window_dps = 25.0;
    double adaptive_balance_speed_window_tps = 0.35;
    double adaptive_balance_cmd_window = 0.08;

    double target_speed_rate_limit = 3.0;
    double target_pitch_rate_limit_dps = 80.0;
    double target_displacement_catchup_rate = 0.35;
    double stop_speed_threshold_tps = 0.12;
    double stop_hold_capture_window_tps = 0.20;
    double reference_position_gain = 0.10;
    double reference_velocity_damping_gain = 0.06;
    double speed_to_pitch_ff = 1.0;
    double accel_to_pitch_ff = 0.05;
    double velocity_feedforward_gain = 0.25;

    bool auto_tune_enable = false;
    double auto_tune_rate = 0.015;
    double auto_tune_min_scale = 0.65;
    double auto_tune_max_scale = 1.60;
    double auto_tune_pitch_error_target_deg = 2.0;
    double auto_tune_speed_error_target_tps = 0.15;

    double friction_comp_tps = 0.0;
    double friction_comp_deadband_tps = 0.03;
    double friction_comp_fade_tps = 0.35;

    double observer_pos_alpha = 0.55;
    double observer_vel_beta = 0.18;

    bool ekf_enable = true;
    double ekf_q_position = 1e-4;
    double ekf_q_velocity = 2e-2;
    double ekf_q_pitch = 4e-2;
    double ekf_q_pitch_rate = 1.5;
    double ekf_r_position = 2e-4;
    double ekf_r_velocity = 4e-2;
    double ekf_r_pitch = 0.8;
    double ekf_r_pitch_rate = 2.0;
    double ekf_init_pos_var = 5e-3;
    double ekf_init_vel_var = 5e-2;
    double ekf_init_pitch_var = 4.0;
    double ekf_init_pitch_rate_var = 16.0;

    double body_mass_kg = 9.0;
    double wheel_mass_kg = 0.6;
    double com_height_m = 0.22;
    double wheel_radius_m = 0.085;
    double drivetrain_time_constant_s = 0.08;
    double gravity_mps2 = 9.81;
    double auto_lqr_enable = 0.0;
    double lqr_q_x = 20.0;
    double lqr_q_v = 6.0;
    double lqr_q_theta = 220.0;
    double lqr_q_theta_dot = 18.0;
    double lqr_r_u = 1.4;

    double lqr_k_theta = 1.2;
    double lqr_k_theta_d = 0.06;
    double lqr_k_x = 0.12;
    double lqr_k_v = 0.28;
    double lqr_speed_gain_scale = 0.18;
    double lqr_gain_scale_max = 1.6;
    double lqr_integral_k = 0.0;
    double lqr_integral_limit = 3.0;

    bool nmpc_enabled = false;
    int nmpc_horizon_steps = 12;
    int nmpc_candidate_count = 16;
    double nmpc_pitch_min_deg = -12.0;
    double nmpc_pitch_max_deg = 12.0;
    double nmpc_pitch_slew_dps = 180.0;
    double nmpc_model_accel_gain = 7.5;
    double nmpc_model_accel_damping = 2.0;
    double nmpc_model_theta_stiffness = 18.0;
    double nmpc_model_theta_damping = 3.5;
    double nmpc_model_couple_gain = 0.11;
    double nmpc_w_theta = 8.0;
    double nmpc_w_theta_rate = 0.25;
    double nmpc_w_x = 2.5;
    double nmpc_w_v = 0.8;
    double nmpc_w_u = 0.05;
    double nmpc_w_du = 0.15;
    double nmpc_w_terminal_theta = 6.0;
    double nmpc_w_terminal_x = 4.0;
    double nmpc_w_terminal_v = 2.0;
    double nmpc_w_terminal_u = 0.10;
    double nmpc_reference_velocity_blend = 0.65;
    double nmpc_reference_position_preview_gain = 1.0;
    double nmpc_terminal_lqr_scale = 1.0;
    double nmpc_ltv_velocity_scale = 0.15;
    double nmpc_ltv_pitch_scale = 0.10;
    double nmpc_use_physical_model = 1.0;
    double nmpc_use_auto_lqr_terminal = 1.0;
    double nmpc_stage_lqr_tail_mix = 0.35;

    double stale_result_max_age_s = 0.05;
    double nmpc_state_mismatch_pitch_deg = 6.0;
    double nmpc_state_mismatch_vel_tps = 0.8;
    double nmpc_state_mismatch_disp_turns = 0.35;


    void sanitize() {
        auto clampd = [](double& v, double lo, double hi) {
            if (!std::isfinite(v)) v = lo;
            v = std::clamp(v, lo, hi);
        };
        auto clampi = [](int& v, int lo, int hi) { v = std::clamp(v, lo, hi); };
        pitch_kp = std::max(0.0, pitch_kp); pitch_ki = std::max(0.0, pitch_ki); pitch_kd = std::max(0.0, pitch_kd);
        clampd(pitch_d_alpha, 0.001, 1.0);
        yaw_kp = std::max(0.0, yaw_kp); yaw_ki = std::max(0.0, yaw_ki); yaw_kd = std::max(0.0, yaw_kd);
        speed_kp = std::max(0.0, speed_kp); speed_ki = std::max(0.0, speed_ki); speed_kd = std::max(0.0, speed_kd);
        pos_kp = std::max(0.0, pos_kp); pos_ki = std::max(0.0, pos_ki); pos_kd = std::max(0.0, pos_kd);
        clampd(pos_out_max_bwd, -45.0, -0.01); clampd(pos_out_max_fwd, 0.01, 45.0);
        if (pos_out_max_bwd >= pos_out_max_fwd) pos_out_max_bwd = -std::abs(pos_out_max_fwd);
        clampd(target_pitch_max_bwd, -45.0, -0.01); clampd(target_pitch_max_fwd, 0.01, 45.0);
        clampd(comp_filter_alpha, 0.0, 1.0); clampd(pitch_lpf_alpha, 0.0, 1.0); clampd(wheel_velocity_lpf_alpha, 0.0, 1.0); clampd(command_lpf_alpha, 0.0, 1.0); clampd(steering_lpf_alpha, 0.0, 1.0);
        clampd(pitch_offset, -20.0, 20.0); clampd(max_velocity, 0.05, 100.0); clampd(max_torque, 0.0, 100.0); clampd(joystick_deadband, 0.0, 0.95); clampi(control_rate_hz, 20, 2000); clampd(speed_ramp_rate, 0.01, 1000.0); clampd(max_tilt_angle, 5.0, 89.0); clampd(wheel_base_m, 0.05, 5.0); clampi(rc_timeout_ms, 10, 100000);
        if (tcp_port == 0) tcp_port = 9000;
        clampi(msp_request_interval_us, 100, 1000000);
        clampi(usb_bulk_timeout_ms, 1, 5000);
        clampi(usb_rx_queue_depth, 1, 128);
        clampi(logging_decimation, 1, 1000);
        clampd(adaptive_balance_rate, 0.0, 10.0); clampd(adaptive_balance_max_trim_deg, 0.0, 45.0); clampd(adaptive_balance_pitch_window_deg, 0.0, 45.0); clampd(adaptive_balance_gyro_window_dps, 0.0, 5000.0); clampd(adaptive_balance_speed_window_tps, 0.0, 100.0); clampd(adaptive_balance_cmd_window, 0.0, 1.0);
        clampd(target_speed_rate_limit, 0.01, 1000.0); clampd(target_pitch_rate_limit_dps, 0.01, 5000.0); clampd(target_displacement_catchup_rate, 0.0, 100.0); clampd(stop_speed_threshold_tps, 0.0, 10.0); clampd(stop_hold_capture_window_tps, 0.0, 10.0); clampd(reference_position_gain, 0.0, 50.0); clampd(reference_velocity_damping_gain, 0.0, 50.0); clampd(speed_to_pitch_ff, -100.0, 100.0); clampd(accel_to_pitch_ff, -100.0, 100.0); clampd(velocity_feedforward_gain, 0.0, 2.0);
        clampd(auto_tune_rate, 0.0, 1.0); clampd(auto_tune_min_scale, 0.1, 10.0); clampd(auto_tune_max_scale, 0.1, 10.0); if (auto_tune_min_scale > auto_tune_max_scale) std::swap(auto_tune_min_scale, auto_tune_max_scale); clampd(auto_tune_pitch_error_target_deg, 0.1, 30.0); clampd(auto_tune_speed_error_target_tps, 0.01, 50.0);
        clampd(friction_comp_tps, 0.0, 10.0); clampd(friction_comp_deadband_tps, 0.0, 10.0); clampd(friction_comp_fade_tps, 0.01, 100.0);
        clampd(observer_pos_alpha, 0.0, 1.0); clampd(observer_vel_beta, 0.0, 1.0);
        clampd(ekf_q_position, 1e-12, 1e6); clampd(ekf_q_velocity, 1e-12, 1e6); clampd(ekf_q_pitch, 1e-12, 1e6); clampd(ekf_q_pitch_rate, 1e-12, 1e6);
        clampd(ekf_r_position, 1e-12, 1e6); clampd(ekf_r_velocity, 1e-12, 1e6); clampd(ekf_r_pitch, 1e-12, 1e6); clampd(ekf_r_pitch_rate, 1e-12, 1e6);
        clampd(ekf_init_pos_var, 1e-12, 1e6); clampd(ekf_init_vel_var, 1e-12, 1e6); clampd(ekf_init_pitch_var, 1e-12, 1e6); clampd(ekf_init_pitch_rate_var, 1e-12, 1e6);
        clampd(body_mass_kg, 0.05, 500.0); clampd(wheel_mass_kg, 0.0, 100.0); clampd(com_height_m, 0.01, 5.0); clampd(wheel_radius_m, 0.005, 1.0); clampd(drivetrain_time_constant_s, 1e-4, 10.0); clampd(gravity_mps2, 1.0, 30.0);
        clampd(auto_lqr_enable, 0.0, 1.0); clampd(lqr_q_x, 0.0, 1e6); clampd(lqr_q_v, 0.0, 1e6); clampd(lqr_q_theta, 0.0, 1e6); clampd(lqr_q_theta_dot, 0.0, 1e6); clampd(lqr_r_u, 1e-9, 1e6);
        clampd(lqr_k_theta, 0.0, 1e3); clampd(lqr_k_theta_d, 0.0, 1e3); clampd(lqr_k_x, 0.0, 1e3); clampd(lqr_k_v, 0.0, 1e3); clampd(lqr_speed_gain_scale, 0.0, 1e3); clampd(lqr_gain_scale_max, 0.0, 1e3); clampd(lqr_integral_k, 0.0, 1e3); clampd(lqr_integral_limit, 0.0, 1e6);
        clampi(nmpc_horizon_steps, 1, 256); clampi(nmpc_candidate_count, 1, 256); clampd(nmpc_pitch_min_deg, -89.0, 0.0); clampd(nmpc_pitch_max_deg, 0.0, 89.0); if (nmpc_pitch_min_deg >= nmpc_pitch_max_deg) { nmpc_pitch_min_deg = -12.0; nmpc_pitch_max_deg = 12.0; } clampd(nmpc_pitch_slew_dps, 0.01, 1e4);
        clampd(nmpc_model_accel_gain, -1e4, 1e4); clampd(nmpc_model_accel_damping, -1e4, 1e4); clampd(nmpc_model_theta_stiffness, -1e4, 1e4); clampd(nmpc_model_theta_damping, -1e4, 1e4); clampd(nmpc_model_couple_gain, -1e4, 1e4);
        clampd(nmpc_w_theta, 0.0, 1e9); clampd(nmpc_w_theta_rate, 0.0, 1e9); clampd(nmpc_w_x, 0.0, 1e9); clampd(nmpc_w_v, 0.0, 1e9); clampd(nmpc_w_u, 0.0, 1e9); clampd(nmpc_w_du, 0.0, 1e9); clampd(nmpc_w_terminal_theta, 0.0, 1e9); clampd(nmpc_w_terminal_x, 0.0, 1e9); clampd(nmpc_w_terminal_v, 0.0, 1e9); clampd(nmpc_w_terminal_u, 0.0, 1e9);
        clampd(nmpc_reference_velocity_blend, 0.0, 1.0); clampd(nmpc_reference_position_preview_gain, 0.0, 1e3); clampd(nmpc_terminal_lqr_scale, 0.0, 1e3); clampd(nmpc_ltv_velocity_scale, 0.0, 1e3); clampd(nmpc_ltv_pitch_scale, 0.0, 1e3);
        clampd(nmpc_use_physical_model, 0.0, 1.0); clampd(nmpc_use_auto_lqr_terminal, 0.0, 1.0); clampd(nmpc_stage_lqr_tail_mix, 0.0, 1.0); clampd(stale_result_max_age_s, 0.0, 10.0); clampd(nmpc_state_mismatch_pitch_deg, 0.0, 180.0); clampd(nmpc_state_mismatch_vel_tps, 0.0, 1e3); clampd(nmpc_state_mismatch_disp_turns, 0.0, 1e3);
    }

    bool validate(std::string* reason = nullptr) const {
        auto fail = [&](const std::string& r) { if (reason) *reason = r; return false; };
        if (!std::isfinite(max_velocity) || max_velocity <= 0.0) return fail("max_velocity must be positive and finite");
        if (!std::isfinite(max_tilt_angle) || max_tilt_angle <= 0.0 || max_tilt_angle >= 89.0) return fail("max_tilt_angle must be in (0,89)");
        if (target_pitch_max_bwd >= 0.0 || target_pitch_max_fwd <= 0.0) return fail("target pitch limits must straddle zero");
        if (nmpc_pitch_min_deg >= nmpc_pitch_max_deg) return fail("nmpc pitch min must be less than max");
        if (control_rate_hz <= 0) return fail("control_rate_hz must be positive");
        if (tcp_port == 0) return fail("tcp_port must be non-zero");
        if (usb_bulk_timeout_ms <= 0 || usb_rx_queue_depth <= 0) return fail("USB timing parameters must be positive");
        if (wheel_radius_m <= 0.0 || com_height_m <= 0.0 || body_mass_kg <= 0.0) return fail("physical parameters must be positive");
        if (ekf_enable && (ekf_q_position <= 0.0 || ekf_r_pitch <= 0.0)) return fail("EKF covariances must be positive");
        return true;
    }
};

struct ImuData {
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;
    double acc_x = 0.0;
    double acc_y = 0.0;
    double acc_z = 0.0;
    std::chrono::steady_clock::time_point timestamp;
    bool valid = false;
};

struct RemoteCommand {
    double throttle = 0.0;
    double steering = 0.0;
    double speed_limit = 1.0;
    bool emergency_stop = false;
    bool arm = false;
    std::chrono::steady_clock::time_point timestamp;
};

struct MotorOutput {
    double left_velocity = 0.0;
    double right_velocity = 0.0;
};

struct EstimatedState {
    double pitch_deg = 0.0;
    double pitch_rate_dps = 0.0;
    double displacement_turns = 0.0;
    double velocity_tps = 0.0;
    double balance_trim_deg = 0.0;
    bool valid = false;
};

struct ReferenceState {
    double target_speed_tps = 0.0;
    double target_displacement_turns = 0.0;
    double nominal_pitch_deg = 0.0;
    double constrained_pitch_deg = 0.0;
};

struct PidDebugData {
    double target_pitch = 0.0;
    double actual_pitch = 0.0;
    double filtered_pitch = 0.0;
    double filtered_pitch_rate = 0.0;
    double pitch_error = 0.0;
    double pitch_p = 0.0;
    double pitch_i = 0.0;
    double pitch_d = 0.0;
    double pitch_output = 0.0;
    double target_speed = 0.0;
    double speed_estimate = 0.0;
    double speed_error = 0.0;
    double gyro_y = 0.0;
    double gyro_y_dot = 0.0;
    double pos_left = 0.0;
    double pos_right = 0.0;
    double pos_disp = 0.0;
    double observed_displacement = 0.0;
    double filtered_velocity = 0.0;
    double pos_target = 0.0;
    double pos_out = 0.0;
    double cmd_left = 0.0;
    double cmd_right = 0.0;

    double balance_point = 0.0;
    double balance_trim = 0.0;
    double throttle_filtered = 0.0;
    double steering_filtered = 0.0;
    double nominal_pitch = 0.0;
    double theta_error = 0.0;
    double theta_d_error = 0.0;
    double x_error = 0.0;
    double v_error = 0.0;
    double fb_theta = 0.0;
    double fb_theta_d = 0.0;
    double fb_x = 0.0;
    double fb_v = 0.0;
    double ff_term = 0.0;
    double integral_term = 0.0;
    double base_cmd = 0.0;
    double nmpc_cost = 0.0;
    double nmpc_age_ms = 0.0;
    bool nmpc_used = false;
};

enum class SystemState : int {
    INITIALIZING = 0,
    WAITING_USB = 1,
    CALIBRATING = 2,
    IDLE = 3,
    ARMED = 4,
    BALANCING = 5,
    FAULT = 6,
    SHUTDOWN = 7
};

inline const char* state_name(SystemState s) {
    switch (s) {
        case SystemState::INITIALIZING: return "INITIALIZING";
        case SystemState::WAITING_USB: return "WAITING_USB";
        case SystemState::CALIBRATING: return "CALIBRATING";
        case SystemState::IDLE: return "IDLE";
        case SystemState::ARMED: return "ARMED";
        case SystemState::BALANCING: return "BALANCING";
        case SystemState::FAULT: return "FAULT";
        case SystemState::SHUTDOWN: return "SHUTDOWN";
    }
    return "UNKNOWN";
}

inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
inline double deg_to_rad(double d) { return d * M_PI / 180.0; }
inline double rad_to_deg(double r) { return r * 180.0 / M_PI; }

inline double apply_deadband(double val, double band) {
    if (std::abs(val) < band) return 0.0;
    const double sign = (val > 0.0) ? 1.0 : -1.0;
    return sign * (std::abs(val) - band) / (1.0 - band);
}

inline double lpf(double prev, double value, double alpha) {
    alpha = clamp(alpha, 0.0, 1.0);
    return alpha * value + (1.0 - alpha) * prev;
}

inline double ramp_limit(double prev, double target, double rate, double dt) {
    const double max_step = std::max(0.0, rate) * std::max(0.0, dt);
    return prev + clamp(target - prev, -max_step, max_step);
}

}  // namespace robot
