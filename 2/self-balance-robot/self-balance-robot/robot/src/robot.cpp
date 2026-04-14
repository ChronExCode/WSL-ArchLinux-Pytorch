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
#include <iostream>
#include <chrono>

namespace robot {

Robot::Robot() = default;

Robot::~Robot() {
    shutdown();
}

void Robot::set_state(SystemState s) {
    auto old = static_cast<SystemState>(state_.load());
    state_.store(static_cast<int>(s));
    std::cout << "[Robot] State: " << state_name(old)
              << " → " << state_name(s) << "\n";
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

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// ── Initialization ───────────────────────────────────────────

bool Robot::initialize(const Config& cfg) {
    cfg_ = cfg;
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
    controller_ = BalanceController(cfg_);

    // 5. Start TCP server
    tcp_.set_state_callback([this](const std::string& cmd) {
        handle_state_command(cmd);
    });
    tcp_.set_pid_callback([this](const Config& partial) {
        if (partial.pitch_kp >= 0) cfg_.pitch_kp = partial.pitch_kp;
        if (partial.pitch_ki >= 0) cfg_.pitch_ki = partial.pitch_ki;
        if (partial.pitch_kd >= 0) cfg_.pitch_kd = partial.pitch_kd;
        if (partial.pitch_d_alpha >= 0) cfg_.pitch_d_alpha = partial.pitch_d_alpha;
        if (partial.yaw_kp >= 0)   cfg_.yaw_kp   = partial.yaw_kp;
        if (partial.speed_kp >= 0) cfg_.speed_kp  = partial.speed_kp;
        if (partial.pos_kp >= 0)   cfg_.pos_kp    = partial.pos_kp;
        if (partial.pos_ki >= 0)   cfg_.pos_ki    = partial.pos_ki;
        if (partial.pos_kd >= 0)   cfg_.pos_kd    = partial.pos_kd;
        if (partial.pos_out_max_fwd > -998) cfg_.pos_out_max_fwd = partial.pos_out_max_fwd;
        if (partial.pos_out_max_bwd < 998)  cfg_.pos_out_max_bwd = partial.pos_out_max_bwd;
        if (partial.target_pitch_max_fwd > -998) cfg_.target_pitch_max_fwd = partial.target_pitch_max_fwd;
        if (partial.target_pitch_max_bwd < 998)  cfg_.target_pitch_max_bwd = partial.target_pitch_max_bwd;
        if (partial.pitch_offset > -998) cfg_.pitch_offset = partial.pitch_offset;
        if (partial.max_velocity >= 0)   cfg_.max_velocity = partial.max_velocity;
        controller_.set_config(cfg_);
        std::cout << "[Robot] PID updated: pitch(" << cfg_.pitch_kp
                  << "/" << cfg_.pitch_ki << "/" << cfg_.pitch_kd
                  << ") pos(" << cfg_.pos_kp << "/" << cfg_.pos_ki
                  << "/" << cfg_.pos_kd << ")"
                  << " pos_lim(" << cfg_.pos_out_max_bwd << "," << cfg_.pos_out_max_fwd << ")"
                  << " tgt_lim(" << cfg_.target_pitch_max_bwd << "," << cfg_.target_pitch_max_fwd << ")"
                  << " offset=" << cfg_.pitch_offset
                  << " max_vel=" << cfg_.max_velocity << "\n";
    });

    if (!tcp_.start(cfg_.tcp_port)) {
        std::cerr << "[Robot] ERROR: Cannot start TCP server\n";
        return false;
    }

    // 6. Try initial device connections
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
        if (!odrive_.set_velocity_control(0)) {
            std::cerr << "[Robot] Failed to set velocity control axis0\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (!odrive_.set_velocity_control(1)) {
            std::cerr << "[Robot] Failed to set velocity control axis1\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Enter closed loop
        if (!odrive_.set_closed_loop(0)) {
            std::cerr << "[Robot] Failed to set closed loop axis0\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!odrive_.set_closed_loop(1)) {
            std::cerr << "[Robot] Failed to set closed loop axis1\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Poll for closed loop state (up to 500ms)
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
        if (!devices_ready()) {
            std::string msg = "Cannot arm: devices not connected (odrive="
                + std::to_string(odrive_connected_.load())
                + " imu=" + std::to_string(imu_connected_.load()) + ")";
            std::cout << "[Robot] " << msg << "\n";
            set_status_msg(msg);
            return;
        }
        if (state == SystemState::IDLE || state == SystemState::FAULT) {
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
    } else if (cmd == "estop") {
        std::cout << "[Robot] !! EMERGENCY STOP !!\n";
        set_state(SystemState::FAULT);
        controller_.reset();
        last_output_ = MotorOutput{};
        if (odrive_.is_connected()) {
            odrive_.emergency_stop();
        }
    }
}

void Robot::enter_fault(const std::string& reason) {
    std::cerr << "[Robot] FAULT: " << reason << "\n";
    set_status_msg("FAULT: " + reason);
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
            double raw_r = -(odrive_.get_position(1) - enc_offset_r_);  // negate: axis1 mirror-mounted
            double disp = (raw_l + raw_r) * 0.5;
            enc_displacement_.store(disp, std::memory_order_relaxed);
            enc_pos_l_.store(raw_l, std::memory_order_relaxed);
            enc_pos_r_.store(raw_r, std::memory_order_relaxed);
        }
    }

    controller_.set_displacement(enc_displacement_.load(std::memory_order_relaxed));

    MotorOutput output = controller_.update(imu_data, cmd, dt);

    // Tilt safety
    if (!controller_.is_tilt_safe(imu_data.pitch)) {
        enter_fault("Tilt exceeded safety limit");
        return;
    }

    odrive_.set_velocities(output.left_velocity, output.right_velocity);
    last_output_ = output;
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
            double disp = enc_displacement_.load(std::memory_order_relaxed);
            dbg.speed_estimate = disp;
            dbg.pos_left  = enc_pos_l_.load(std::memory_order_relaxed);
            dbg.pos_right = enc_pos_r_.load(std::memory_order_relaxed);

            // Motor command velocities (already in unified sign: positive=forward)
            dbg.cmd_left  = last_output_.left_velocity;
            dbg.cmd_right = last_output_.right_velocity;

            double pitch = imu_data.pitch;
            double yaw_rel = imu_data.yaw - yaw_offset_;

            tcp_.send_telemetry(
                pitch, imu_data.roll, yaw_rel,
                controller_.get_speed_estimate(), cached_vbus,
                get_state(),
                last_output_.left_velocity, last_output_.right_velocity,
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

        control_tick(dt);

        auto elapsed = std::chrono::steady_clock::now() - now;
        auto sleep_time = loop_period -
            std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    // Cleanup — order matters!
    // 1. Stop motors first (while USB is still alive)
    if (odrive_.is_connected()) {
        odrive_.set_velocities(0, 0);
        odrive_.emergency_stop();
    }

    // 2. Stop IMU polling (stops its async transfers)
    imu_.stop_polling();

    // 3. Wait for background threads to exit
    //    (they check running_ flag and will stop)
    if (telemetry_thread_.joinable()) telemetry_thread_.join();
    if (reconnect_thread_.joinable()) reconnect_thread_.join();

    // 4. Stop TCP (closes sockets, joins TCP threads)
    tcp_.stop();

    // 5. Detach devices (cancels async transfers, releases USB interfaces)
    odrive_.detach();
    imu_.detach();

    // 6. Finally shutdown libusb (now safe — no threads using USB)
    usb_mgr_.shutdown();

    std::cout << "[Robot] Shutdown complete\n";
}

void Robot::shutdown() {
    running_.store(false);
}

} // namespace robot
