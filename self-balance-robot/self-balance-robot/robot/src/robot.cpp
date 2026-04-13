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

    // Start async RX listener for telemetry responses
    odrive_.start_rx_listener();

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

    // Small delay to let the device settle after plug-in
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (try_connect_odrive()) {
        if (devices_ready() && get_state() == SystemState::WAITING_USB) {
            set_state(SystemState::IDLE);
            std::cout << "[Robot] Both devices ready. Send 'arm' to start.\n";
        }
    }
}

void Robot::on_odrive_detached() {
    std::cout << "[Robot] !! ODrive USB detached !!\n";

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
        odrive_connected_.store(false);
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
        imu_connected_.store(false);
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
        if (partial.yaw_kp >= 0)   cfg_.yaw_kp   = partial.yaw_kp;
        if (partial.speed_kp >= 0) cfg_.speed_kp  = partial.speed_kp;
        if (partial.pitch_offset > -998) cfg_.pitch_offset = partial.pitch_offset;
        if (partial.max_velocity >= 0)   cfg_.max_velocity = partial.max_velocity;
        controller_.set_config(cfg_);
        std::cout << "[Robot] PID updated: pitch(" << cfg_.pitch_kp
                  << "/" << cfg_.pitch_ki << "/" << cfg_.pitch_kd
                  << ") yaw_kp=" << cfg_.yaw_kp
                  << " speed_kp=" << cfg_.speed_kp
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
    odrive_.clear_all_errors();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Try up to 2 times
    for (int attempt = 0; attempt < 2; attempt++) {
        if (attempt > 0) {
            std::cout << "[Robot] Retry " << attempt << ": clearing errors...\n";
            odrive_.set_idle(0);
            odrive_.set_idle(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            odrive_.clear_all_errors();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        odrive_.set_velocity_control(0);
        odrive_.set_velocity_control(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        odrive_.set_closed_loop(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        odrive_.set_closed_loop(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (odrive_.is_closed_loop(0) && odrive_.is_closed_loop(1)) {
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
            std::cout << "[Robot] Cannot arm: devices not connected"
                      << " (odrive=" << odrive_connected_.load()
                      << " imu=" << imu_connected_.load() << ")\n";
            return;
        }
        if (state == SystemState::IDLE || state == SystemState::FAULT) {
            set_state(SystemState::CALIBRATING);

            // Clear any stale emergency stop flag
            {
                auto cmd_data = tcp_.get_command();
                // The arm command in tcp_server already clears estop,
                // but log it for diagnostics
                if (cmd_data.emergency_stop) {
                    std::cout << "[Robot] Clearing stale E-STOP flag\n";
                }
            }

            auto imu_data = imu_.get_imu_data();
            if (!imu_data.valid) {
                std::cout << "[Robot] Cannot arm: IMU data not valid\n";
                set_state(SystemState::IDLE);
                return;
            }
            if (std::abs(imu_data.pitch) > 20.0) {
                std::cout << "[Robot] Cannot arm: tilt too large ("
                          << imu_data.pitch << "°). Hold upright.\n";
                set_state(SystemState::IDLE);
                return;
            }
            std::cout << "[Robot] Arming... pitch=" << imu_data.pitch << "°\n";
            if (!init_odrive_motors()) {
                enter_fault("ODrive init failed");
                return;
            }
            controller_.reset();
            controller_.set_balance_point(imu_data.pitch);
            // Record encoder positions at ARM time for relative display
            enc_offset_l_ = odrive_.get_position(0);
            enc_offset_r_ = odrive_.get_position(1);
            enc_displacement_.store(0, std::memory_order_relaxed);
            set_state(SystemState::BALANCING);
            std::cout << "[Robot] ✓ Armed and balancing!\n";
        } else {
            std::cout << "[Robot] Cannot arm in state "
                      << state_name(state) << "\n";
        }
    } else if (cmd == "disarm") {
        std::cout << "[Robot] Disarm requested\n";
        // Set state FIRST so control_tick stops sending velocities
        set_state(devices_ready() ? SystemState::IDLE : SystemState::WAITING_USB);
        controller_.reset();
        last_output_ = MotorOutput{};  // clear stale telemetry
        // Then stop motors — send multiple times for reliability
        // (async writes from control_tick may have been in flight)
        if (odrive_.is_connected()) {
            odrive_.set_velocities(0, 0);
            odrive_.emergency_stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            odrive_.set_velocities(0, 0);
            odrive_.emergency_stop();
            odrive_.clear_all_errors();
        }
        std::cout << "[Robot] Disarmed, motors stopped\n";
    } else if (cmd == "estop") {
        std::cout << "[Robot] !! EMERGENCY STOP !!\n";
        set_state(SystemState::FAULT);
        controller_.reset();
        last_output_ = MotorOutput{};
        if (odrive_.is_connected()) {
            odrive_.set_velocities(0, 0);
            odrive_.emergency_stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            odrive_.set_velocities(0, 0);
            odrive_.emergency_stop();
            odrive_.clear_all_errors();
        }
    }
}

void Robot::enter_fault(const std::string& reason) {
    std::cerr << "[Robot] FAULT: " << reason << "\n";
    if (odrive_.is_connected()) odrive_.emergency_stop();
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

    ImuData imu_data = imu_.get_imu_data();
    RemoteCommand cmd = tcp_.get_command();

    if (!tcp_.has_client() || !tcp_.is_command_fresh(500)) {
        cmd.throttle = 0;
        cmd.steering = 0;
    }

    if (cmd.emergency_stop) {
        enter_fault("Remote emergency stop");
        return;
    }

    // Pass encoder displacement to controller (updated at 20Hz by telemetry thread)
    controller_.set_displacement(enc_displacement_.load(std::memory_order_relaxed));

    MotorOutput output = controller_.update(imu_data, cmd, dt);

    // Tilt safety: use IMU pitch directly (controller's filtered_pitch_
    // now stores angular velocity, not pitch angle)
    if (!controller_.is_tilt_safe(imu_data.pitch)) {
        enter_fault("Tilt exceeded safety limit");
        return;
    }

    odrive_.set_velocities(output.left_velocity, output.right_velocity);
    last_output_ = output;
}

void Robot::telemetry_loop() {
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!running_.load()) break;
        if (!tcp_.has_client()) continue;

        try {
            auto imu_data = imu_.get_imu_data();
            double vbus = 0;
            auto dbg = controller_.get_debug_data();

            if (odrive_.is_connected() && running_.load()) {
                vbus = odrive_.get_vbus_voltage();
                double raw_l = odrive_.get_position(0) - enc_offset_l_;
                double raw_r = odrive_.get_position(1) - enc_offset_r_;
                dbg.pos_left  = raw_l;
                dbg.pos_right = -raw_r;  // negate: right motor is mirrored
                // Average displacement (both in same direction now)
                double disp = (raw_l + (-raw_r)) * 0.5;
                dbg.speed_estimate = disp;
                enc_displacement_.store(disp, std::memory_order_relaxed);
            }

            // Always use IMU pitch directly for telemetry
            double pitch = imu_data.pitch;

            tcp_.send_telemetry(
                pitch, imu_data.roll, imu_data.yaw,
                controller_.get_speed_estimate(), vbus,
                get_state(),
                last_output_.left_velocity, last_output_.right_velocity,
                imu_data, dbg);
        } catch (...) {
            // Device may have disconnected mid-query, ignore and retry
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
