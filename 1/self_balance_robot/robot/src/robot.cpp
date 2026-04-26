// ═══════════════════════════════════════════════════════════════
//  Robot — Main Orchestrator with libusb Hotplug
//
//  Startup flow:
//    1. Init libusb, start event loop
//    2. Register hotplug callbacks for ODrive & F405
//    3. Enumerate existing USB devices, try to connect
//    4. Enter WAITING_USB until both devices are found
//    5. Once both connected → IDLE, wait for 'arm' command
//    6. On device disconnect → auto-disarm, re-enter WAITING_USB
//    7. On device reconnect → auto-transition back to IDLE
// ═══════════════════════════════════════════════════════════════

#include "robot.h"
#include "mini_json.h"
#include "logger.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace robot {

Robot::Robot() = default;

Robot::~Robot() {
    shutdown();
}

void Robot::set_state(SystemState s) {
    auto old = static_cast<SystemState>(state_.load());
    if (old == s) return;
    state_.store(static_cast<int>(s));
    const std::string old_name = state_name(old);
    const std::string new_name = state_name(s);
    std::cout << "[Robot] State: " << old_name
              << " → " << new_name << "\n";
    logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                      "STATE", old_name + " -> " + new_name);
}

// ── USB Connection Management ────────────────────────────────

bool Robot::try_connect_odrive() {
    std::lock_guard<std::mutex> lock(reconnect_mtx_);

    if (odrive_connected_.load()) return true;

    auto dev = usb_mgr_.open_device(ODRIVE_USB_ID);
    if (!dev) return false;

    if (!odrive_.attach(std::move(dev))) {
        std::cerr << "[Robot] ODrive attach failed\n";
        return false;
    }

    // Liveness check: verify ODrive actually responds to commands
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    double vbus = odrive_.get_vbus_voltage();
    if (vbus <= 0) {
        std::cerr << "[Robot] ODrive not responding after attach (vbus=" << vbus << ")\n";
        odrive_.detach();
        return false;
    }
    std::cout << "[Robot] ODrive liveness OK (vbus=" << vbus << "V)\n";

    odrive_connected_.store(true);
    std::cout << "[Robot] ✓ ODrive connected\n";
    logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                      "USB_ATTACH", "ODrive attached and responsive");
    return true;
}

bool Robot::try_connect_imu() {
    std::lock_guard<std::mutex> lock(reconnect_mtx_);

    if (imu_connected_.load()) return true;

    auto dev = usb_mgr_.open_device(F405_USB_ID);
    if (!dev) return false;

    if (!imu_.attach(std::move(dev))) {
        std::cerr << "[Robot] IMU attach failed\n";
        return false;
    }

    // Start MSP polling
    imu_.start_polling(cfg_.msp_request_interval_us);

    imu_connected_.store(true);
    std::cout << "[Robot] ✓ IMU connected\n";
    logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                      "USB_ATTACH", "IMU attached and polling started");
    return true;
}

void Robot::on_odrive_attached() {
    std::cout << "[Robot] ODrive USB attached event\n";

    // ODrive v3.6 needs significant settle time after USB enumeration
    // before it can reliably accept CDC commands
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (try_connect_odrive()) {
        if (devices_ready() && get_state() == SystemState::WAITING_USB) {
            set_state(SystemState::IDLE);
            std::cout << "[Robot] Both devices ready. Send 'arm' to start.\n";
        }
    }
}

void Robot::on_odrive_detached() {
    std::cout << "[Robot] !! ODrive USB detached !!\n";
    logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                      "USB_DETACH", "ODrive detached");

    // Mark disconnected FIRST to stop telemetry_loop from querying
    odrive_connected_.store(false);

    auto state = get_state();
    if (state == SystemState::BALANCING || state == SystemState::ARMED) {
        // Emergency: we were actively balancing
        controller_.reset();
        set_state(SystemState::WAITING_USB);
    } else if (state != SystemState::SHUTDOWN) {
        set_state(SystemState::WAITING_USB);
    }

    {
        std::lock_guard<std::mutex> lock(reconnect_mtx_);
        odrive_.detach();
    }
}

void Robot::on_imu_attached() {
    std::cout << "[Robot] IMU USB attached event\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (try_connect_imu()) {
        // Wait for first valid reading
        for (int i = 0; i < 50; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (imu_.is_data_fresh()) {
                auto data = imu_.get_imu_data();
                std::cout << "[Robot] IMU online — pitch=" << data.pitch
                          << "° roll=" << data.roll << "°\n";
                break;
            }
        }

        if (devices_ready() && get_state() == SystemState::WAITING_USB) {
            set_state(SystemState::IDLE);
            std::cout << "[Robot] Both devices ready. Send 'arm' to start.\n";
        }
    }
}

void Robot::on_imu_detached() {
    std::cout << "[Robot] !! IMU USB detached !!\n";
    logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                      "USB_DETACH", "IMU detached");

    // Mark disconnected FIRST
    imu_connected_.store(false);

    auto state = get_state();
    if (state == SystemState::BALANCING || state == SystemState::ARMED) {
        odrive_.emergency_stop();
        controller_.reset();
        set_state(SystemState::WAITING_USB);
    } else if (state != SystemState::SHUTDOWN) {
        set_state(SystemState::WAITING_USB);
    }

    {
        std::lock_guard<std::mutex> lock(reconnect_mtx_);
        imu_.detach();
    }
}

void Robot::reconnect_loop() {
    // Fallback reconnect for platforms without hotplug support.
    // Also serves as initial enumeration on startup.
    while (running_.load()) {
        if (!odrive_connected_.load()) {
            try_connect_odrive();
        }
        if (!imu_connected_.load()) {
            try_connect_imu();
        }

        // Check if we should transition from WAITING_USB to IDLE
        if (devices_ready() && get_state() == SystemState::WAITING_USB) {
            // Wait for IMU data
            bool imu_ok = false;
            for (int i = 0; i < 50 && running_.load(); ++i) {
                if (imu_.is_data_fresh()) { imu_ok = true; break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (imu_ok) {
                set_state(SystemState::IDLE);
                std::cout << "[Robot] Both devices ready. Send 'arm' to start.\n";
            }
        }

        // Also detect disconnects by trying a simple operation
        if (odrive_connected_.load() && odrive_.is_connected()) {
            // Periodic liveness check: read vbus voltage
            // (get_vbus_voltage returns 0 on failure, but that's also
            //  a valid value for no-power, so check is_connected after)
        }

        for (int i = 0; i < 20 && running_.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

// ── Initialization ───────────────────────────────────────────

bool Robot::initialize(const Config& cfg) {
    cfg_ = cfg;
    cfg_.sanitize();
    std::string cfg_reason;
    if (!cfg_.validate(&cfg_reason)) {
        std::cerr << "[Robot] Invalid config at initialize: " << cfg_reason << "\n";
        return false;
    }
    set_state(SystemState::INITIALIZING);

    // 1. Initialize libusb
    if (!usb_mgr_.init()) {
        std::cerr << "[Robot] ERROR: libusb init failed\n";
        return false;
    }

    // Print all connected USB devices
    std::cout << "\n[Robot] USB Device Enumeration:\n";
    std::cout << "  ┌──────────┬────────────────────────────────────────┐\n";
    auto devices = usb_mgr_.list_devices();
    for (auto& d : devices) {
        char vid_pid[16];
        snprintf(vid_pid, sizeof(vid_pid), "%04X:%04X", d.vid, d.pid);

        std::string label;
        if (d.vid == ODRIVE_USB_ID.vid && d.pid == ODRIVE_USB_ID.pid) {
            label = " ← ODrive v3.5";
        } else if (d.vid == F405_USB_ID.vid && d.pid == F405_USB_ID.pid) {
            label = " ← SpeedyBee F405";
        }

        std::cout << "  │ " << vid_pid << " │ "
                  << d.manufacturer << " " << d.product;
        if (!d.serial.empty()) std::cout << " [" << d.serial << "]";
        std::cout << label << "\n";
    }
    std::cout << "  └──────────┴────────────────────────────────────────┘\n\n";

    // 2. Start libusb event loop (for async transfers + hotplug)
    usb_mgr_.start_event_loop();

    // 3. Register hotplug callbacks
    usb_mgr_.register_hotplug(ODRIVE_USB_ID,
        [this](UsbManager::HotplugEvent ev, uint16_t, uint16_t) {
            if (ev == UsbManager::HotplugEvent::ATTACHED) {
                on_odrive_attached();
            } else {
                on_odrive_detached();
            }
        });

    usb_mgr_.register_hotplug(F405_USB_ID,
        [this](UsbManager::HotplugEvent ev, uint16_t, uint16_t) {
            if (ev == UsbManager::HotplugEvent::ATTACHED) {
                on_imu_attached();
            } else {
                on_imu_detached();
            }
        });

    // 4. Initialize balance controller
    controller_.set_config(cfg_);
    controller_.reset();

    // 5. Start CSV logger (best-effort; does not fail robot startup)
    if (!logger_.start("logs", cfg_.logging_enable)) {
        std::cerr << "[Robot] Warning: failed to start CSV logger\n";
    } else {
        logger_.log_config_snapshot(cfg_);
    }

    // 6. Start TCP server
    tcp_.set_state_callback([this](const std::string& cmd) {
        PendingStateCmd pending = PendingStateCmd::None;
        if (cmd == "arm") pending = PendingStateCmd::Arm;
        else if (cmd == "disarm") pending = PendingStateCmd::Disarm;
        else if (cmd == "estop") pending = PendingStateCmd::Estop;
        if (pending == PendingStateCmd::None) return;
        if (pending == PendingStateCmd::Estop) {
            pending_state_cmd_.store(static_cast<int>(PendingStateCmd::Estop), std::memory_order_release);
            return;
        }
        PendingStateCmd current = static_cast<PendingStateCmd>(pending_state_cmd_.load(std::memory_order_acquire));
        if (current != PendingStateCmd::Estop) {
            pending_state_cmd_.store(static_cast<int>(pending), std::memory_order_release);
        }
    });
    tcp_.set_pid_callback([this](const std::string& json) {
        auto getd = [&](const char* key, double current) {
            const double sentinel = -1.23456789e300;
            std::string search = std::string("\"") + key + "\":";
            size_t pos = json.find(search);
            if (pos == std::string::npos) return current;
            pos += search.size();
            while (pos < json.size() && json[pos] == ' ') ++pos;
            double v = sentinel;
            try { v = std::stod(json.substr(pos)); } catch (...) { return current; }
            return (v == sentinel) ? current : v;
        };
        auto geti = [&](const char* key, int current) {
            return static_cast<int>(std::round(getd(key, static_cast<double>(current))));
        };
        auto getb = [&](const char* key, bool current) {
            return getd(key, current ? 1.0 : 0.0) >= 0.5;
        };
        cfg_.pitch_kp = getd("pitch_kp", cfg_.pitch_kp); cfg_.pitch_ki = getd("pitch_ki", cfg_.pitch_ki); cfg_.pitch_kd = getd("pitch_kd", cfg_.pitch_kd); cfg_.pitch_d_alpha = getd("pitch_d_alpha", cfg_.pitch_d_alpha);
        cfg_.yaw_kp = getd("yaw_kp", cfg_.yaw_kp); cfg_.yaw_ki = getd("yaw_ki", cfg_.yaw_ki); cfg_.yaw_kd = getd("yaw_kd", cfg_.yaw_kd);
        cfg_.speed_kp = getd("speed_kp", cfg_.speed_kp); cfg_.speed_ki = getd("speed_ki", cfg_.speed_ki); cfg_.speed_kd = getd("speed_kd", cfg_.speed_kd);
        cfg_.pos_kp = getd("pos_kp", cfg_.pos_kp); cfg_.pos_ki = getd("pos_ki", cfg_.pos_ki); cfg_.pos_kd = getd("pos_kd", cfg_.pos_kd);
        cfg_.pos_out_max_fwd = getd("pos_out_max_fwd", cfg_.pos_out_max_fwd); cfg_.pos_out_max_bwd = getd("pos_out_max_bwd", cfg_.pos_out_max_bwd);
        cfg_.target_pitch_max_fwd = getd("target_pitch_max_fwd", cfg_.target_pitch_max_fwd); cfg_.target_pitch_max_bwd = getd("target_pitch_max_bwd", cfg_.target_pitch_max_bwd);
        cfg_.comp_filter_alpha = getd("comp_filter_alpha", cfg_.comp_filter_alpha); cfg_.pitch_lpf_alpha = getd("pitch_lpf_alpha", cfg_.pitch_lpf_alpha); cfg_.wheel_velocity_lpf_alpha = getd("wheel_velocity_lpf_alpha", cfg_.wheel_velocity_lpf_alpha); cfg_.command_lpf_alpha = getd("command_lpf_alpha", cfg_.command_lpf_alpha); cfg_.steering_lpf_alpha = getd("steering_lpf_alpha", cfg_.steering_lpf_alpha);
        cfg_.pitch_offset = getd("pitch_offset", cfg_.pitch_offset); cfg_.max_velocity = getd("max_velocity", cfg_.max_velocity); cfg_.max_torque = getd("max_torque", cfg_.max_torque); cfg_.joystick_deadband = getd("joystick_deadband", cfg_.joystick_deadband); cfg_.speed_ramp_rate = getd("speed_ramp_rate", cfg_.speed_ramp_rate); cfg_.max_tilt_angle = getd("max_tilt_angle", cfg_.max_tilt_angle); cfg_.wheel_base_m = getd("wheel_base_m", cfg_.wheel_base_m);
        cfg_.adaptive_balance_enable = getb("adaptive_balance_enable", cfg_.adaptive_balance_enable); cfg_.adaptive_balance_rate = getd("adaptive_balance_rate", cfg_.adaptive_balance_rate); cfg_.adaptive_balance_max_trim_deg = getd("adaptive_balance_max_trim_deg", cfg_.adaptive_balance_max_trim_deg); cfg_.adaptive_balance_pitch_window_deg = getd("adaptive_balance_pitch_window_deg", cfg_.adaptive_balance_pitch_window_deg); cfg_.adaptive_balance_gyro_window_dps = getd("adaptive_balance_gyro_window_dps", cfg_.adaptive_balance_gyro_window_dps); cfg_.adaptive_balance_speed_window_tps = getd("adaptive_balance_speed_window_tps", cfg_.adaptive_balance_speed_window_tps); cfg_.adaptive_balance_cmd_window = getd("adaptive_balance_cmd_window", cfg_.adaptive_balance_cmd_window);
        cfg_.target_speed_rate_limit = getd("target_speed_rate_limit", cfg_.target_speed_rate_limit); cfg_.target_pitch_rate_limit_dps = getd("target_pitch_rate_limit_dps", cfg_.target_pitch_rate_limit_dps); cfg_.target_displacement_catchup_rate = getd("target_displacement_catchup_rate", cfg_.target_displacement_catchup_rate); cfg_.stop_speed_threshold_tps = getd("stop_speed_threshold_tps", cfg_.stop_speed_threshold_tps); cfg_.stop_hold_capture_window_tps = getd("stop_hold_capture_window_tps", cfg_.stop_hold_capture_window_tps); cfg_.reference_position_gain = getd("reference_position_gain", cfg_.reference_position_gain); cfg_.reference_velocity_damping_gain = getd("reference_velocity_damping_gain", cfg_.reference_velocity_damping_gain); cfg_.speed_to_pitch_ff = getd("speed_to_pitch_ff", cfg_.speed_to_pitch_ff); cfg_.accel_to_pitch_ff = getd("accel_to_pitch_ff", cfg_.accel_to_pitch_ff);
        cfg_.observer_pos_alpha = getd("observer_pos_alpha", cfg_.observer_pos_alpha); cfg_.observer_vel_beta = getd("observer_vel_beta", cfg_.observer_vel_beta);
        cfg_.ekf_enable = getb("ekf_enable", cfg_.ekf_enable); cfg_.ekf_q_position = getd("ekf_q_position", cfg_.ekf_q_position); cfg_.ekf_q_velocity = getd("ekf_q_velocity", cfg_.ekf_q_velocity); cfg_.ekf_q_pitch = getd("ekf_q_pitch", cfg_.ekf_q_pitch); cfg_.ekf_q_pitch_rate = getd("ekf_q_pitch_rate", cfg_.ekf_q_pitch_rate); cfg_.ekf_r_position = getd("ekf_r_position", cfg_.ekf_r_position); cfg_.ekf_r_velocity = getd("ekf_r_velocity", cfg_.ekf_r_velocity); cfg_.ekf_r_pitch = getd("ekf_r_pitch", cfg_.ekf_r_pitch); cfg_.ekf_r_pitch_rate = getd("ekf_r_pitch_rate", cfg_.ekf_r_pitch_rate); cfg_.ekf_init_pos_var = getd("ekf_init_pos_var", cfg_.ekf_init_pos_var); cfg_.ekf_init_vel_var = getd("ekf_init_vel_var", cfg_.ekf_init_vel_var); cfg_.ekf_init_pitch_var = getd("ekf_init_pitch_var", cfg_.ekf_init_pitch_var); cfg_.ekf_init_pitch_rate_var = getd("ekf_init_pitch_rate_var", cfg_.ekf_init_pitch_rate_var);
        cfg_.body_mass_kg = getd("body_mass_kg", cfg_.body_mass_kg); cfg_.wheel_mass_kg = getd("wheel_mass_kg", cfg_.wheel_mass_kg); cfg_.com_height_m = getd("com_height_m", cfg_.com_height_m); cfg_.wheel_radius_m = getd("wheel_radius_m", cfg_.wheel_radius_m); cfg_.drivetrain_time_constant_s = getd("drivetrain_time_constant_s", cfg_.drivetrain_time_constant_s); cfg_.gravity_mps2 = getd("gravity_mps2", cfg_.gravity_mps2); cfg_.auto_lqr_enable = getd("auto_lqr_enable", cfg_.auto_lqr_enable); cfg_.lqr_q_x = getd("lqr_q_x", cfg_.lqr_q_x); cfg_.lqr_q_v = getd("lqr_q_v", cfg_.lqr_q_v); cfg_.lqr_q_theta = getd("lqr_q_theta", cfg_.lqr_q_theta); cfg_.lqr_q_theta_dot = getd("lqr_q_theta_dot", cfg_.lqr_q_theta_dot); cfg_.lqr_r_u = getd("lqr_r_u", cfg_.lqr_r_u);
        cfg_.lqr_k_theta = getd("lqr_k_theta", cfg_.lqr_k_theta); cfg_.lqr_k_theta_d = getd("lqr_k_theta_d", cfg_.lqr_k_theta_d); cfg_.lqr_k_x = getd("lqr_k_x", cfg_.lqr_k_x); cfg_.lqr_k_v = getd("lqr_k_v", cfg_.lqr_k_v); cfg_.lqr_speed_gain_scale = getd("lqr_speed_gain_scale", cfg_.lqr_speed_gain_scale); cfg_.lqr_gain_scale_max = getd("lqr_gain_scale_max", cfg_.lqr_gain_scale_max); cfg_.lqr_integral_k = getd("lqr_integral_k", cfg_.lqr_integral_k); cfg_.lqr_integral_limit = getd("lqr_integral_limit", cfg_.lqr_integral_limit);
        cfg_.nmpc_enabled = getb("nmpc_enabled", cfg_.nmpc_enabled); cfg_.nmpc_horizon_steps = geti("nmpc_horizon_steps", cfg_.nmpc_horizon_steps); cfg_.nmpc_candidate_count = geti("nmpc_candidate_count", cfg_.nmpc_candidate_count); cfg_.nmpc_pitch_min_deg = getd("nmpc_pitch_min_deg", cfg_.nmpc_pitch_min_deg); cfg_.nmpc_pitch_max_deg = getd("nmpc_pitch_max_deg", cfg_.nmpc_pitch_max_deg); cfg_.nmpc_pitch_slew_dps = getd("nmpc_pitch_slew_dps", cfg_.nmpc_pitch_slew_dps); cfg_.nmpc_model_accel_gain = getd("nmpc_model_accel_gain", cfg_.nmpc_model_accel_gain); cfg_.nmpc_model_accel_damping = getd("nmpc_model_accel_damping", cfg_.nmpc_model_accel_damping); cfg_.nmpc_model_theta_stiffness = getd("nmpc_model_theta_stiffness", cfg_.nmpc_model_theta_stiffness); cfg_.nmpc_model_theta_damping = getd("nmpc_model_theta_damping", cfg_.nmpc_model_theta_damping); cfg_.nmpc_model_couple_gain = getd("nmpc_model_couple_gain", cfg_.nmpc_model_couple_gain); cfg_.nmpc_w_theta = getd("nmpc_w_theta", cfg_.nmpc_w_theta); cfg_.nmpc_w_theta_rate = getd("nmpc_w_theta_rate", cfg_.nmpc_w_theta_rate); cfg_.nmpc_w_x = getd("nmpc_w_x", cfg_.nmpc_w_x); cfg_.nmpc_w_v = getd("nmpc_w_v", cfg_.nmpc_w_v); cfg_.nmpc_w_u = getd("nmpc_w_u", cfg_.nmpc_w_u); cfg_.nmpc_w_du = getd("nmpc_w_du", cfg_.nmpc_w_du); cfg_.nmpc_w_terminal_theta = getd("nmpc_w_terminal_theta", cfg_.nmpc_w_terminal_theta); cfg_.nmpc_w_terminal_x = getd("nmpc_w_terminal_x", cfg_.nmpc_w_terminal_x); cfg_.nmpc_w_terminal_v = getd("nmpc_w_terminal_v", cfg_.nmpc_w_terminal_v); cfg_.nmpc_w_terminal_u = getd("nmpc_w_terminal_u", cfg_.nmpc_w_terminal_u); cfg_.nmpc_reference_velocity_blend = getd("nmpc_reference_velocity_blend", cfg_.nmpc_reference_velocity_blend); cfg_.nmpc_reference_position_preview_gain = getd("nmpc_reference_position_preview_gain", cfg_.nmpc_reference_position_preview_gain); cfg_.nmpc_terminal_lqr_scale = getd("nmpc_terminal_lqr_scale", cfg_.nmpc_terminal_lqr_scale); cfg_.nmpc_use_physical_model = getd("nmpc_use_physical_model", cfg_.nmpc_use_physical_model); cfg_.nmpc_use_auto_lqr_terminal = getd("nmpc_use_auto_lqr_terminal", cfg_.nmpc_use_auto_lqr_terminal); cfg_.nmpc_stage_lqr_tail_mix = getd("nmpc_stage_lqr_tail_mix", cfg_.nmpc_stage_lqr_tail_mix); cfg_.nmpc_ltv_velocity_scale = getd("nmpc_ltv_velocity_scale", cfg_.nmpc_ltv_velocity_scale); cfg_.nmpc_ltv_pitch_scale = getd("nmpc_ltv_pitch_scale", cfg_.nmpc_ltv_pitch_scale); cfg_.stale_result_max_age_s = getd("stale_result_max_age_s", cfg_.stale_result_max_age_s); cfg_.nmpc_state_mismatch_pitch_deg = getd("nmpc_state_mismatch_pitch_deg", cfg_.nmpc_state_mismatch_pitch_deg); cfg_.nmpc_state_mismatch_vel_tps = getd("nmpc_state_mismatch_vel_tps", cfg_.nmpc_state_mismatch_vel_tps); cfg_.nmpc_state_mismatch_disp_turns = getd("nmpc_state_mismatch_disp_turns", cfg_.nmpc_state_mismatch_disp_turns);
        cfg_.sanitize();
        std::string cfg_reason;
        if (!cfg_.validate(&cfg_reason)) {
            set_status_msg("Config adjusted: " + cfg_reason);
        }
        controller_.set_config(cfg_);
        const double now_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        logger_.log_config_update(now_s, cfg_, "tcp_set_pid");
        logger_.log_event(now_s, "config_update", cfg_reason.empty() ? "remote config applied" : ("remote config applied with adjustments: " + cfg_reason));
    });

    if (!tcp_.start(cfg_.tcp_port)) {
        std::cerr << "[Robot] ERROR: Cannot start TCP server\n";
        return false;
    }

    // 7. Try initial device connections
    set_state(SystemState::WAITING_USB);
    try_connect_odrive();
    try_connect_imu();

    if (devices_ready()) {
        // Wait for IMU data
        for (int i = 0; i < 50; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (imu_.is_data_fresh()) {
                auto data = imu_.get_imu_data();
                std::cout << "[Robot] IMU online — pitch=" << data.pitch
                          << "° roll=" << data.roll << "°\n";
                set_state(SystemState::IDLE);
                break;
            }
        }
    }

    if (get_state() == SystemState::WAITING_USB) {
        std::cout << "[Robot] Waiting for USB devices...\n";
        if (!odrive_connected_.load())
            std::cout << "  ✗ ODrive (VID:PID = 1209:0D32)\n";
        if (!imu_connected_.load())
            std::cout << "  ✗ SpeedyBee F405 (VID:PID = 0483:5740)\n";
    }

    std::cout << "[Robot] ✓ Initialization complete\n";
    return true;
}

bool Robot::init_odrive_motors() {
    if (!odrive_.is_connected()) return false;

    std::cout << "[Robot] Setting up ODrive motors...\n";

    // Clear any existing errors first
    if (!odrive_.clear_all_errors()) {
        std::cerr << "[Robot] Failed to clear ODrive errors\n";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Try up to 2 times
    for (int attempt = 0; attempt < 2; attempt++) {
        if (attempt > 0) {
            std::cout << "[Robot] Retry " << attempt << ": clearing errors...\n";
            odrive_.set_idle(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            odrive_.set_idle(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            odrive_.clear_all_errors();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Set velocity control mode
        std::cout << "[ARM] ODrive init: set_velocity_control axis0 (attempt " << (attempt + 1) << ")\n";
        if (!odrive_.set_velocity_control(0)) {
            std::cerr << "[Robot] Failed to set velocity control axis0\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::cout << "[ARM] ODrive init: set_velocity_control axis1 (attempt " << (attempt + 1) << ")\n";
        if (!odrive_.set_velocity_control(1)) {
            std::cerr << "[Robot] Failed to set velocity control axis1\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Enter closed loop
        std::cout << "[ARM] ODrive init: set_closed_loop axis0 (attempt " << (attempt + 1) << ")\n";
        if (!odrive_.set_closed_loop(0)) {
            std::cerr << "[Robot] Failed to set closed loop axis0\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "[ARM] ODrive init: set_closed_loop axis1 (attempt " << (attempt + 1) << ")\n";
        if (!odrive_.set_closed_loop(1)) {
            std::cerr << "[Robot] Failed to set closed loop axis1\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Poll for closed loop state (up to 500ms)
        std::cout << "[ARM] ODrive init: poll closed-loop state (attempt " << (attempt + 1) << ")\n";
        bool ready = false;
        for (int i = 0; i < 10; i++) {
            if (odrive_.is_closed_loop(0) && odrive_.is_closed_loop(1)) {
                ready = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (ready) {
            odrive_.set_velocities(0, 0);
            std::cout << "[Robot] ✓ ODrive ready\n";
            return true;
        }

        int err0 = odrive_.get_axis_error(0);
        int err1 = odrive_.get_axis_error(1);
        std::cerr << "[Robot] ODrive not in closed loop (attempt " << attempt + 1
                  << ") axis0 err=" << err0 << " axis1 err=" << err1 << "\n";
    }

    return false;
}

void Robot::handle_state_command(const std::string& cmd) {
    auto state = get_state();

    if (cmd == "arm") {
        std::cout << "[ARM] step 1: entered arm handler\n";
        if (!devices_ready()) {
            std::string msg = "Cannot arm: devices not connected (odrive="
                + std::to_string(odrive_connected_.load())
                + " imu=" + std::to_string(imu_connected_.load()) + ")";
            std::cout << "[Robot] " << msg << "\n";
            set_status_msg(msg);
            return;
        }
        if (state == SystemState::IDLE || state == SystemState::FAULT) {
            std::cout << "[ARM] step 2: entering CALIBRATING\n";
            set_state(SystemState::CALIBRATING);

            // Clear any stale emergency stop flag
            {
                auto cmd_data = tcp_.get_command();
                if (cmd_data.emergency_stop) {
                    std::cout << "[Robot] Clearing stale E-STOP flag\n";
                }
            }

            auto imu_data = imu_.get_imu_data();
            if (!imu_data.valid) {
                set_status_msg("Cannot arm: IMU data not valid");
                std::cout << "[Robot] Cannot arm: IMU data not valid\n";
                set_state(SystemState::IDLE);
                return;
            }
            if (std::abs(imu_data.pitch) > 20.0) {
                char buf[128];
                snprintf(buf, sizeof(buf), "Cannot arm: tilt too large (%.1f deg)", imu_data.pitch);
                set_status_msg(buf);
                std::cout << "[Robot] " << buf << "\n";
                set_state(SystemState::IDLE);
                return;
            }
            std::cout << "[Robot] Arming... pitch=" << imu_data.pitch << "°\n";
            if (!init_odrive_motors()) {
                set_status_msg("Cannot arm: ODrive init failed");
                enter_fault("ODrive init failed");
                return;
            }
            std::cout << "[ARM] step 6: init_odrive_motors done\n";
            controller_.reset();
            controller_.set_balance_point(imu_data.pitch);
            // Record encoder positions at ARM time for relative display
            enc_offset_l_ = odrive_.get_position(0);
            enc_offset_r_ = odrive_.get_position(1);
            enc_displacement_.store(0, std::memory_order_relaxed);
            // Record yaw as zero point
            yaw_offset_ = imu_data.yaw;
            std::cout << "[Robot] Yaw zero point: " << yaw_offset_ << "°\n";
            set_state(SystemState::BALANCING);
            set_status_msg("Armed and balancing");
            std::cout << "[Robot] ✓ Armed and balancing!\n";
            logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                              "ARM", "Armed and balancing");
        } else {
            char buf[64];
            snprintf(buf, sizeof(buf), "Cannot arm in state %s", state_name(state));
            set_status_msg(buf);
            std::cout << "[Robot] " << buf << "\n";
        }
    } else if (cmd == "disarm") {
        std::cout << "[Robot] Disarm requested\n";
        set_state(devices_ready() ? SystemState::IDLE : SystemState::WAITING_USB);
        controller_.reset();
        last_output_ = MotorOutput{};
        if (odrive_.is_connected()) {
            odrive_.emergency_stop();  // sets idle + zero velocity in one call
        }
        std::cout << "[Robot] Disarmed, motors stopped\n";
        logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                          "DISARM", "User disarm");
    } else if (cmd == "estop") {
        std::cout << "[Robot] !! EMERGENCY STOP !!\n";
        set_state(SystemState::FAULT);
        controller_.reset();
        last_output_ = MotorOutput{};
        if (odrive_.is_connected()) {
            odrive_.emergency_stop();
        }
        logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                          "ESTOP", "Remote emergency stop");
    }
}

void Robot::enter_fault(const std::string& reason) {
    std::cerr << "[Robot] FAULT: " << reason << "\n";
    set_status_msg("FAULT: " + reason);
    logger_.log_event(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                      "FAULT", reason);
    // Only attempt emergency_stop if ODrive is still connected
    // (avoids accessing freed device handle during detach race)
    if (odrive_connected_.load()) {
        odrive_.emergency_stop();
    }
    controller_.reset();
    last_output_ = MotorOutput{};
    set_state(SystemState::FAULT);
}

void Robot::control_tick(double dt) {
    auto state = get_state();
    if (state != SystemState::BALANCING) return;

    // Check device connectivity
    if (!odrive_.is_connected()) {
        enter_fault("ODrive disconnected during balancing");
        return;
    }

    if (!imu_.is_data_fresh(50)) {
        enter_fault("IMU data stale");
        return;
    }

    // ── RC timeout safety: auto disarm if no control message ──
    if (!tcp_.has_client() || !tcp_.is_command_fresh(cfg_.rc_timeout_ms)) {
        std::cout << "[Robot] RC timeout (" << cfg_.rc_timeout_ms
                  << "ms), auto disarm\n";
        handle_state_command("disarm");
        return;
    }

    ImuData imu_data = imu_.get_imu_data();
    RemoteCommand cmd = tcp_.get_command();

    if (cmd.emergency_stop) {
        enter_fault("Remote emergency stop");
        return;
    }

    // ── Encoder query at 100Hz (every other tick of 200Hz loop) ──
    enc_tick_counter_++;
    if (enc_tick_counter_ >= 2) {
        enc_tick_counter_ = 0;
        if (odrive_connected_.load() && odrive_.is_connected()) {
            double raw_l = odrive_.get_position(0) - enc_offset_l_;
            double raw_r = odrive_.get_position(1) - enc_offset_r_;  // axis1 sign handled inside get_position()
            double disp = (raw_l + raw_r) * 0.5;
            enc_displacement_.store(disp, std::memory_order_relaxed);
            enc_pos_l_.store(raw_l, std::memory_order_relaxed);
            enc_pos_r_.store(raw_r, std::memory_order_relaxed);

            // Read actual wheel velocity from ODrive
            double vel_l = odrive_.get_velocity(0);
            double vel_r = odrive_.get_velocity(1);
            enc_vel_l_.store(vel_l, std::memory_order_relaxed);
            enc_vel_r_.store(vel_r, std::memory_order_relaxed);
            enc_velocity_.store((vel_l + vel_r) * 0.5, std::memory_order_relaxed);
        }
    }

    controller_.set_displacement(enc_displacement_.load(std::memory_order_relaxed));
    controller_.set_wheel_velocity(enc_velocity_.load(std::memory_order_relaxed));

    MotorOutput output = controller_.update(imu_data, cmd, dt);

    // Tilt safety
    if (!controller_.is_tilt_safe(imu_data.pitch)) {
        enter_fault("Tilt exceeded safety limit");
        return;
    }

    odrive_.set_velocities(output.left_velocity, output.right_velocity);
    last_output_ = output;

    if ((log_counter_++ % static_cast<uint64_t>(std::max(1, cfg_.logging_decimation))) == 0) {
        logger_.log(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                    get_state(), imu_data, cmd, controller_.get_debug_data(), last_output_,
                    enc_displacement_.load(std::memory_order_relaxed), enc_velocity_.load(std::memory_order_relaxed));
    }
}

void Robot::telemetry_loop() {
    int vbus_counter = 0;
    double cached_vbus = 0;

    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!running_.load()) break;
        if (!tcp_.has_client()) continue;

        try {
            auto imu_data = imu_.get_imu_data();
            auto dbg = controller_.get_debug_data();

            // Query vbus at 2Hz (every 10th tick) — low priority
            if (odrive_connected_.load() && odrive_.is_connected()) {
                vbus_counter++;
                if (vbus_counter >= 10) {
                    vbus_counter = 0;
                    double v = odrive_.get_vbus_voltage();
                    if (v > 0) cached_vbus = v;
                }
            }

            // Encoder display data (updated by control_tick at 100Hz)
            //double disp = enc_displacement_.load(std::memory_order_relaxed);
            double actual_velocity = enc_velocity_.load(std::memory_order_relaxed);
            double actual_left_velocity = enc_vel_l_.load(std::memory_order_relaxed);
            double actual_right_velocity = enc_vel_r_.load(std::memory_order_relaxed);
            dbg.speed_estimate = actual_velocity;
            dbg.pos_left  = enc_pos_l_.load(std::memory_order_relaxed);
            dbg.pos_right = enc_pos_r_.load(std::memory_order_relaxed);

            // Motor command velocities (already in unified sign: positive=forward)
            dbg.cmd_left  = last_output_.left_velocity;
            dbg.cmd_right = last_output_.right_velocity;

            double pitch = imu_data.pitch;
            double yaw_rel = imu_data.yaw - yaw_offset_;

            tcp_.send_telemetry(
                pitch, imu_data.roll, yaw_rel,
                actual_velocity, cached_vbus,
                get_state(),
                actual_left_velocity, actual_right_velocity,
                odrive_connected_.load(), imu_connected_.load(),
                get_status_msg(),
                imu_data, dbg);
        } catch (...) {
        }
    }
}

void Robot::run() {
    running_.store(true);

    telemetry_thread_ = std::thread(&Robot::telemetry_loop, this);
    reconnect_thread_ = std::thread(&Robot::reconnect_loop, this);

    const auto loop_period = std::chrono::microseconds(1000000 / cfg_.control_rate_hz);
    auto prev_time = std::chrono::steady_clock::now();

    std::cout << "[Robot] Control loop running @ " << cfg_.control_rate_hz << " Hz\n";

    while (running_.load()) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - prev_time).count();
        prev_time = now;

        PendingStateCmd pending = static_cast<PendingStateCmd>(pending_state_cmd_.exchange(
            static_cast<int>(PendingStateCmd::None), std::memory_order_acq_rel));
        switch (pending) {
            case PendingStateCmd::Arm:    handle_state_command("arm"); break;
            case PendingStateCmd::Disarm: handle_state_command("disarm"); break;
            case PendingStateCmd::Estop:  handle_state_command("estop"); break;
            case PendingStateCmd::None:   break;
        }

        control_tick(dt);

        auto elapsed = std::chrono::steady_clock::now() - now;
        auto sleep_time = loop_period -
            std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    // Cleanup — order matters!
    set_state(SystemState::SHUTDOWN);
    std::cout << "[Robot] cleanup: stop motors\n";
    // 1. Stop motors first (while USB is still alive)
    if (odrive_.is_connected()) {
        odrive_.set_velocities(0, 0);
        odrive_.emergency_stop();
    }

    // 2. Stop IMU polling (stops its async transfers)
    std::cout << "[Robot] cleanup: stop IMU polling\n";
    imu_.stop_polling();

    // 3. Wait for background threads to exit
    //    (they check running_ flag and will stop)
    std::cout << "[Robot] cleanup: join telemetry\n";
    if (telemetry_thread_.joinable()) telemetry_thread_.join();
    std::cout << "[Robot] cleanup: join reconnect\n";
    if (reconnect_thread_.joinable()) reconnect_thread_.join();

    // 4. Stop logger and TCP (closes sockets, joins TCP threads)
    std::cout << "[Robot] cleanup: stop logger\n";
    logger_.stop();
    std::cout << "[Robot] cleanup: stop tcp\n";
    tcp_.stop();

    // 5. Detach devices (cancels async transfers, releases USB interfaces)
    std::cout << "[Robot] cleanup: detach odrive\n";
    odrive_.detach();
    std::cout << "[Robot] cleanup: detach imu\n";
    imu_.detach();

    // 6. Finally shutdown libusb (now safe — no threads using USB)
    std::cout << "[Robot] cleanup: shutdown usb_mgr\n";
    usb_mgr_.shutdown();

    std::cout << "[Robot] Shutdown complete\n";
}

void Robot::shutdown() {
    running_.store(false);
}

} // namespace robot
