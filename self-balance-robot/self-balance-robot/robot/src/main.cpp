// ═══════════════════════════════════════════════════════════════
//  Self-Balancing Robot — Main Entry Point
//  USB devices auto-detected by VID/PID via libusb
//  No /dev/ttyACM* dependency — works across plug/unplug cycles
// ═══════════════════════════════════════════════════════════════

#include "robot.h"

#include <iostream>
#include <csignal>
#include <cstring>
#include <getopt.h>
#include <sched.h>
#include <termios.h>
#include <thread>

static robot::Robot* g_robot = nullptr;

static void signal_handler(int sig) {
    std::cout << "\n[Main] Signal " << sig << " received, shutting down...\n";
    if (g_robot) g_robot->shutdown();
}

static void print_usage(const char* prog) {
    std::cout <<
        "═══════════════════════════════════════════════════════════\n"
        "  Self-Balancing Robot Controller (libusb)\n"
        "  Raspberry Pi 5 + ODrive v3.5 + SpeedyBee F405 V5\n"
        "═══════════════════════════════════════════════════════════\n"
        "\n"
        "  USB devices are auto-detected by VID/PID:\n"
        "    ODrive v3.5:       1209:0D32\n"
        "    SpeedyBee F405 V5: 0483:5740\n"
        "\n"
        "  No /dev/ttyACM* paths needed — plug in any USB port.\n"
        "\n"
        "Usage: " << prog << " [options]\n"
        "\n"
        "Options:\n"
        "  --tcp-port <port>      TCP control port       [9000]\n"
        "  --rate <hz>            Control loop rate      [200]\n"
        "  --pitch-kp <val>       Pitch PID Kp           [0.30]\n"
        "  --pitch-ki <val>       Pitch PID Ki           [0.005]\n"
        "  --pitch-kd <val>       Pitch PID Kd           [0.025]\n"
        "  --pitch-offset <deg>   Mechanical CoG offset  [0.0]\n"
        "  --max-tilt <deg>       Safety cutoff angle    [45.0]\n"
        "  --max-velocity <v>     Max motor velocity     [2.0]\n"
        "  --list-usb             List USB devices & exit\n"
        "  --help                 Show this help\n"
        "\n"
        "Example:\n"
        "  sudo " << prog << " --pitch-kp 50 --tcp-port 9000\n"
        "\n";
}

static bool set_realtime_priority() {
    struct sched_param param;
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
        std::cerr << "[Main] WARNING: Cannot set RT priority ("
                  << strerror(errno) << "). Run as root.\n";
        return false;
    }
    std::cout << "[Main] Real-time priority set (SCHED_FIFO, prio=80)\n";
    return true;
}

static void list_usb_devices() {
    robot::UsbManager mgr;
    if (!mgr.init()) {
        std::cerr << "Failed to init libusb\n";
        return;
    }

    auto devices = mgr.list_devices();
    std::cout << "\nUSB Devices:\n";
    std::cout << "┌──────────┬─────┬──────────────────────────────────────┐\n";
    std::cout << "│ VID:PID  │ Bus │ Description                          │\n";
    std::cout << "├──────────┼─────┼──────────────────────────────────────┤\n";

    for (auto& d : devices) {
        char vid_pid[16];
        snprintf(vid_pid, sizeof(vid_pid), "%04X:%04X", d.vid, d.pid);

        std::string desc = d.manufacturer;
        if (!desc.empty() && !d.product.empty()) desc += " ";
        desc += d.product;
        if (!d.serial.empty()) desc += " [" + d.serial + "]";
        if (desc.empty()) desc = "(no descriptor)";

        // Mark known devices
        if (d.vid == robot::ODRIVE_USB_ID.vid &&
            d.pid == robot::ODRIVE_USB_ID.pid) {
            desc += "  ★ ODrive";
        } else if (d.vid == robot::F405_USB_ID.vid &&
                   d.pid == robot::F405_USB_ID.pid) {
            desc += "  ★ SpeedyBee F405";
        }

        // Truncate if too long
        if (desc.size() > 36) desc = desc.substr(0, 33) + "...";

        printf("│ %-8s │ %3d │ %-36s │\n", vid_pid, d.bus, desc.c_str());
    }

    std::cout << "└──────────┴─────┴──────────────────────────────────────┘\n";
    std::cout << "\nTotal: " << devices.size() << " devices\n";

    mgr.shutdown();
}

int main(int argc, char* argv[]) {
    robot::Config cfg;
    bool do_list = false;

    static struct option long_options[] = {
        {"tcp-port",     required_argument, nullptr, 't'},
        {"rate",         required_argument, nullptr, 'r'},
        {"pitch-kp",     required_argument, nullptr, 'P'},
        {"pitch-ki",     required_argument, nullptr, 'I'},
        {"pitch-kd",     required_argument, nullptr, 'D'},
        {"pitch-offset", required_argument, nullptr, 'O'},
        {"max-tilt",     required_argument, nullptr, 'T'},
        {"max-velocity", required_argument, nullptr, 'V'},
        {"list-usb",     no_argument,       nullptr, 'l'},
        {"help",         no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "t:r:lh", long_options, nullptr)) != -1) {
        switch (opt) {
            case 't': cfg.tcp_port = static_cast<uint16_t>(std::stoi(optarg)); break;
            case 'r': cfg.control_rate_hz = std::stoi(optarg); break;
            case 'P': cfg.pitch_kp = std::stod(optarg); break;
            case 'I': cfg.pitch_ki = std::stod(optarg); break;
            case 'D': cfg.pitch_kd = std::stod(optarg); break;
            case 'O': cfg.pitch_offset = std::stod(optarg); break;
            case 'T': cfg.max_tilt_angle = std::stod(optarg); break;
            case 'V': cfg.max_velocity = std::stod(optarg); break;
            case 'l': do_list = true; break;
            case 'h': print_usage(argv[0]); return 0;
            default:  print_usage(argv[0]); return 1;
        }
    }

    if (do_list) {
        list_usb_devices();
        return 0;
    }

    std::cout <<
        "\n"
        "  ┌─────────────────────────────────────────────┐\n"
        "  │     Self-Balancing Robot Controller        │\n"
        "  │     RPi5 + ODrive v3.5 + SpeedyBee F405     │\n"
        "  │     USB auto-detect via libusb               │\n"
        "  └─────────────────────────────────────────────┘\n"
        "\n"
        "  ODrive:  VID:PID 1209:0D32 (auto-detect)\n"
        "  IMU:     VID:PID 0483:5740 (auto-detect)\n"
        "  TCP:     port " << cfg.tcp_port << "\n"
        "  Rate:    " << cfg.control_rate_hz << " Hz\n"
        "  PID:     Kp=" << cfg.pitch_kp
                << " Ki=" << cfg.pitch_ki
                << " Kd=" << cfg.pitch_kd << "\n"
        "\n";

    set_realtime_priority();

    robot::Robot robot_instance;
    g_robot = &robot_instance;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    if (!robot_instance.initialize(cfg)) {
        std::cerr << "[Main] Initialization failed!\n";
        return 1;
    }

    // Set terminal to raw mode for keypress detection
    struct termios orig_term, raw_term;
    tcgetattr(STDIN_FILENO, &orig_term);
    raw_term = orig_term;
    raw_term.c_lflag &= ~(ICANON | ECHO);
    raw_term.c_cc[VMIN] = 0;
    raw_term.c_cc[VTIME] = 1;  // 100ms timeout
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_term);

    // Keyboard listener thread — press 'q' or ESC to quit
    std::thread key_thread([&]() {
        while (robot_instance.is_running()) {
            char ch = 0;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == 'q' || ch == 'Q' || ch == 27 /* ESC */) {
                    std::cout << "\n[Main] Quit key pressed, shutting down...\n";
                    robot_instance.shutdown();
                    break;
                }
            }
        }
    });

    std::cout << "  Press 'q' or ESC to quit.\n\n";

    robot_instance.run();

    if (key_thread.joinable()) key_thread.join();

    // Restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);

    return 0;
}
