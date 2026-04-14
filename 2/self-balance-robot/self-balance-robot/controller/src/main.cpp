// ═══════════════════════════════════════════════════════════════
//  Robot Remote Controller — MSI Claw 8 AI+ A2VM
//  Wayland Client + evdev Gamepad Input + TCP to Raspberry Pi 5
// ═══════════════════════════════════════════════════════════════

#include "wayland_app.h"
#include "hud_renderer.h"

#include <linux/input.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <dirent.h>

// Global app pointer for signal handler
static WaylandApp* g_app = nullptr;

static void signal_handler(int sig) {
    (void)sig;
    if (g_app) g_app->running = false;
}

// ═════════════════════════════════════════════════════════════
//  Shared Memory Buffer
// ═════════════════════════════════════════════════════════════

static int create_shm_file(size_t size) {
    char name[] = "/wl_shm-XXXXXX";
    int fd = memfd_create(name, MFD_CLOEXEC);
    if (fd < 0) return -1;
    if (ftruncate(fd, size) < 0) { close(fd); return -1; }
    return fd;
}

static void buffer_release(void* data, wl_buffer* /*buffer*/) {
    auto* b = static_cast<WaylandApp::ShmBuffer*>(data);
    b->busy = false;
}

static const wl_buffer_listener buffer_listener = {
    .release = buffer_release,
};

static bool create_buffer(WaylandApp& app, WaylandApp::ShmBuffer& b) {
    int stride = app.width * 4;
    b.shm_size = stride * app.height;
    b.shm_fd = create_shm_file(b.shm_size);
    if (b.shm_fd < 0) return false;
    b.shm_data = mmap(nullptr, b.shm_size,
                      PROT_READ | PROT_WRITE, MAP_SHARED, b.shm_fd, 0);
    if (b.shm_data == MAP_FAILED) {
        close(b.shm_fd); b.shm_fd = -1; b.shm_data = nullptr; return false;
    }
    wl_shm_pool* pool = wl_shm_create_pool(app.shm, b.shm_fd, b.shm_size);
    b.buffer = wl_shm_pool_create_buffer(
        pool, 0, app.width, app.height, stride, WL_SHM_FORMAT_ARGB8888);
    wl_shm_pool_destroy(pool);
    if (!b.buffer) {
        munmap(b.shm_data, b.shm_size); close(b.shm_fd);
        b.shm_data = nullptr; b.shm_fd = -1; b.shm_size = 0; return false;
    }
    wl_buffer_add_listener(b.buffer, &buffer_listener, &b);
    b.busy = false;
    return true;
}

// ═════════════════════════════════════════════════════════════
//  Wayland Listeners
// ═════════════════════════════════════════════════════════════

static void xdg_surface_configure(void* data, xdg_surface* surf, uint32_t serial) {
    WaylandApp* app = (WaylandApp*)data;
    xdg_surface_ack_configure(surf, serial);
    app->configured = true;
}
static const xdg_surface_listener xdg_surface_listener_impl = {
    .configure = xdg_surface_configure,
};

static void xdg_toplevel_configure(void*, xdg_toplevel*, int32_t, int32_t, wl_array*) {}
static void xdg_toplevel_close(void* data, xdg_toplevel*) {
    ((WaylandApp*)data)->running = false;
}
static void xdg_toplevel_configure_bounds(void*, xdg_toplevel*, int32_t, int32_t) {}
static void xdg_toplevel_wm_capabilities(void*, xdg_toplevel*, wl_array*) {}
static const xdg_toplevel_listener xdg_toplevel_listener_impl = {
    .configure = xdg_toplevel_configure,
    .close = xdg_toplevel_close,
    .configure_bounds = xdg_toplevel_configure_bounds,
    .wm_capabilities = xdg_toplevel_wm_capabilities,
};

static void xdg_wm_base_ping(void*, xdg_wm_base* base, uint32_t serial) {
    xdg_wm_base_pong(base, serial);
}
static const xdg_wm_base_listener xdg_wm_base_listener_impl = {
    .ping = xdg_wm_base_ping,
};

static void keyboard_keymap(void*, wl_keyboard*, uint32_t, int fd, uint32_t) { close(fd); }
static void keyboard_enter(void*, wl_keyboard*, uint32_t, wl_surface*, wl_array*) {}
static void keyboard_leave(void*, wl_keyboard*, uint32_t, wl_surface*) {}
static void keyboard_key(void* data, wl_keyboard*, uint32_t, uint32_t,
                          uint32_t key, uint32_t state) {
    WaylandApp* app = (WaylandApp*)data;
    if (state != WL_KEYBOARD_KEY_STATE_PRESSED) return;
    if (key == 1) app->running = false;
    if (key == 57) {
        app->chart_paused = !app->chart_paused;
        std::cout << "[Chart] " << (app->chart_paused ? "PAUSED" : "RUNNING") << "\n";
    }
}
static void keyboard_modifiers(void*, wl_keyboard*, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t) {}
static void keyboard_repeat(void*, wl_keyboard*, int32_t, int32_t) {}
static const wl_keyboard_listener keyboard_listener_impl = {
    .keymap = keyboard_keymap, .enter = keyboard_enter, .leave = keyboard_leave,
    .key = keyboard_key, .modifiers = keyboard_modifiers, .repeat_info = keyboard_repeat,
};

static void seat_capabilities(void* data, wl_seat* seat, uint32_t caps) {
    WaylandApp* app = (WaylandApp*)data;
    if (caps & WL_SEAT_CAPABILITY_KEYBOARD) {
        app->keyboard = wl_seat_get_keyboard(seat);
        wl_keyboard_add_listener(app->keyboard, &keyboard_listener_impl, app);
    }
}
static void seat_name(void*, wl_seat*, const char*) {}
static const wl_seat_listener seat_listener_impl = {
    .capabilities = seat_capabilities, .name = seat_name,
};

static void registry_global(void* data, wl_registry* registry,
                             uint32_t name, const char* interface, uint32_t) {
    WaylandApp* app = (WaylandApp*)data;
    if (strcmp(interface, wl_compositor_interface.name) == 0)
        app->compositor = (wl_compositor*)wl_registry_bind(registry, name, &wl_compositor_interface, 4);
    else if (strcmp(interface, wl_shm_interface.name) == 0)
        app->shm = (wl_shm*)wl_registry_bind(registry, name, &wl_shm_interface, 1);
    else if (strcmp(interface, xdg_wm_base_interface.name) == 0) {
        app->xdg_base = (xdg_wm_base*)wl_registry_bind(registry, name, &xdg_wm_base_interface, 2);
        xdg_wm_base_add_listener(app->xdg_base, &xdg_wm_base_listener_impl, app);
    }
    else if (strcmp(interface, wl_seat_interface.name) == 0) {
        app->seat = (wl_seat*)wl_registry_bind(registry, name, &wl_seat_interface, 5);
        wl_seat_add_listener(app->seat, &seat_listener_impl, app);
    }
}
static void registry_global_remove(void*, wl_registry*, uint32_t) {}
static const wl_registry_listener registry_listener_impl = {
    .global = registry_global, .global_remove = registry_global_remove,
};

// ═════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════

static void print_usage(const char* prog) {
    std::cout <<
        "Usage: " << prog << " <server_ip> [options]\n"
        "Options:\n"
        "  --joy <device>        evdev device [auto]\n"
        "  --port <port>         TCP port [9000]\n"
        "  --joy-map             Print gamepad events & exit\n"
        "  --btn-arm <code>      ARM key [313]\n"
        "  --btn-estop <code>    E-STOP key [312]\n"
        "  --btn-disarm <code>   DISARM key [306]\n"
        "  --help\n";
}

int main(int argc, char* argv[]) {
    WaylandApp app;
    g_app = &app;
    std::string joy_device = "auto";
    uint16_t port = TCP_PORT;

    // Signal handling — Ctrl+C, Ctrl+Z, terminal close all trigger clean exit
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGTSTP, signal_handler);
    signal(SIGHUP,  signal_handler);

    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        print_usage(argv[0]);
        return (argc < 2) ? 1 : 0;
    }

    app.server_ip = argv[1];

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--joy") == 0 && i + 1 < argc) joy_device = argv[++i];
        else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) port = std::stoi(argv[++i]);
        else if (strcmp(argv[i], "--joy-map") == 0) app.joy_map_mode = true;
        else if (strcmp(argv[i], "--btn-arm") == 0 && i + 1 < argc) app.btn_arm = std::stoi(argv[++i]);
        else if (strcmp(argv[i], "--btn-estop") == 0 && i + 1 < argc) app.btn_estop = std::stoi(argv[++i]);
        else if (strcmp(argv[i], "--btn-disarm") == 0 && i + 1 < argc) app.btn_disarm = std::stoi(argv[++i]);
        else if (strcmp(argv[i], "--no-dinput-switch") == 0) app.auto_switch_dinput = false;
    }

    // ── Load saved parameters ───────────────────────────────
    // Resolve ~/self-balance-robot/controller
    {
        const char* home = getenv("HOME");
        if (home) {
            app.config_dir = std::string(home) + "/self-balance-robot/controller";
        } else {
            app.config_dir = "./";
        }
    }
    // Load config first (sets defaults), then command-line server_ip overrides
    std::string cli_server_ip = app.server_ip;
    app.load_config();
    app.server_ip = cli_server_ip;  // command-line IP always wins

    // ── Gamepad Mapping Mode ─────────────────────────────────
    if (app.joy_map_mode) {
        if (app.gamepad.epollfd_open()) {
            std::cerr << "Cannot open gamepad for mapping.\n";
            return 1;
        }
        std::cout << "Gamepad Mapping Mode. Press Ctrl+C to exit.\n";
        struct epoll_event events[MAX_EPOLL_EVENTS];
        while (true) {
            int nfds = epoll_wait(app.gamepad.epollfd, events, MAX_EPOLL_EVENTS, -1);
            if (nfds < 0) { if (errno == EINTR) continue; break; }
            for (int i = 0; i < nfds; ++i) {
                if (events[i].events & (EPOLLERR | EPOLLHUP))
                    fprintf(stderr, "fd=%d error/hup\n", events[i].data.fd);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // ── Open Gamepad ─────────────────────────────────────────
    joy_device = "EVENT4 EVENT5 EVENT6";
    if (joy_device != "auto") {
        if (app.gamepad.epollfd_open()) {
            std::cerr << "[Main] WARNING: Cannot open gamepad, keyboard only\n";
        } else {
            std::cout << "epollfd_open successed\n";
        }
    }
    if (app.gamepad.is_open) {
        std::cout << "[Main] Gamepad open\n"
                  << "  ARM    = " << app.btn_arm << "\n"
                  << "  DISARM = KEY_B (48)\n"
                  << "  E-STOP = KEY_X (45)\n";
    }

    // ── Connect to Wayland ──────────────────────────────────
    app.display = wl_display_connect(nullptr);
    if (!app.display) { std::cerr << "Cannot connect to Wayland\n"; return 1; }

    app.registry = wl_display_get_registry(app.display);
    wl_registry_add_listener(app.registry, &registry_listener_impl, &app);
    wl_display_roundtrip(app.display);

    if (!app.compositor || !app.shm || !app.xdg_base) {
        std::cerr << "Missing Wayland interfaces\n"; return 1;
    }

    app.surface = wl_compositor_create_surface(app.compositor);
    app.xdg_surf = xdg_wm_base_get_xdg_surface(app.xdg_base, app.surface);
    xdg_surface_add_listener(app.xdg_surf, &xdg_surface_listener_impl, &app);
    app.toplevel = xdg_surface_get_toplevel(app.xdg_surf);
    xdg_toplevel_add_listener(app.toplevel, &xdg_toplevel_listener_impl, &app);
    xdg_toplevel_set_title(app.toplevel, WINDOW_TITLE);
    xdg_toplevel_set_app_id(app.toplevel, "robot-controller");
    xdg_toplevel_set_fullscreen(app.toplevel, nullptr);
    wl_surface_commit(app.surface);
    wl_display_roundtrip(app.display);

    for (int i = 0; i < WaylandApp::BUFFER_COUNT; ++i) {
        if (!create_buffer(app, app.buffers[i])) {
            std::cerr << "Failed to create shm buffer " << i << "\n";
            return 1;
        }
    }

    // ── TCP Connection Thread ───────────────────────────────
    std::atomic<bool> tcp_running{true};
    std::thread tcp_thread([&]() {
        int reconnect_delay_ms = 1000;
        bool was_connected = false;

        while (tcp_running.load() && app.running) {
            if (!app.tcp.is_connected()) {
                if (was_connected) {
                    std::cout << "[TCP] Disconnected, reconnecting...\n";
                    was_connected = false;
                    reconnect_delay_ms = 1000;
                    { std::lock_guard<std::mutex> lock(app.telem_mtx); app.telem.state = "DISCONNECTED"; }
                    app.armed = false;
                }
                if (app.tcp.connect(app.server_ip, port)) {
                    std::cout << "[TCP] Connected to " << app.server_ip << ":" << port << "\n";
                    was_connected = true; reconnect_delay_ms = 1000;

                    // Auto-send saved parameters to robot on connect
                    char pid_buf[1024];
                    snprintf(pid_buf, sizeof(pid_buf),
                        "{\"type\":\"set_pid\","
                        "\"pitch_kp\":%.4f,\"pitch_ki\":%.4f,\"pitch_kd\":%.4f,"
                        "\"pitch_d_alpha\":%.4f,"
                        "\"yaw_kp\":%.4f,"
                        "\"speed_kp\":%.4f,"
                        "\"pos_kp\":%.4f,\"pos_ki\":%.4f,\"pos_kd\":%.4f,"
                        "\"pos_out_max_fwd\":%.2f,\"pos_out_max_bwd\":%.2f,"
                        "\"tgt_pitch_max_fwd\":%.2f,\"tgt_pitch_max_bwd\":%.2f,"
                        "\"pitch_offset\":%.4f,\"max_velocity\":%.4f}",
                        app.pid_pitch[0].value, app.pid_pitch[1].value, app.pid_pitch[2].value,
                        app.pid_pitch[3].value,
                        app.pid_yaw[0].value,
                        app.pid_speed[0].value,
                        app.pid_pos[0].value, app.pid_pos[1].value, app.pid_pos[2].value,
                        app.pid_limits[0].value, app.pid_limits[1].value,
                        app.pid_limits[2].value, app.pid_limits[3].value,
                        app.pid_other[0].value, app.pid_other[1].value);
                    app.tcp.send_line(pid_buf);
                    std::cout << "[TCP] Auto-sent saved parameters to robot\n";
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
                    if (reconnect_delay_ms < 5000) reconnect_delay_ms += 1000;
                    continue;
                }
            }

            auto lines = app.tcp.recv_lines();
            for (auto& line : lines) {
                auto type = json_parse_string(line, "type");
                if (type == "telemetry") {
                    std::lock_guard<std::mutex> lock(app.telem_mtx);
                    app.telem.pitch    = json_parse_double(line, "pitch");
                    app.telem.roll     = json_parse_double(line, "roll");
                    app.telem.yaw      = json_parse_double(line, "yaw");
                    app.telem.speed    = json_parse_double(line, "speed");
                    app.telem.vbus     = json_parse_double(line, "vbus");
                    app.telem.left_vel = json_parse_double(line, "left_vel");
                    app.telem.right_vel = json_parse_double(line, "right_vel");
                    app.telem.gyro_x   = json_parse_double(line, "gyro_x");
                    app.telem.gyro_y   = json_parse_double(line, "gyro_y");
                    app.telem.gyro_z   = json_parse_double(line, "gyro_z");
                    app.telem.acc_x    = json_parse_double(line, "acc_x");
                    app.telem.acc_y    = json_parse_double(line, "acc_y");
                    app.telem.acc_z    = json_parse_double(line, "acc_z");
                    app.telem.tgt_pitch = json_parse_double(line, "tgt_pitch");
                    app.telem.flt_pitch = json_parse_double(line, "flt_pitch");
                    app.telem.pitch_err = json_parse_double(line, "pitch_err");
                    app.telem.pitch_p   = json_parse_double(line, "pitch_p");
                    app.telem.pitch_i   = json_parse_double(line, "pitch_i");
                    app.telem.pitch_d   = json_parse_double(line, "pitch_d");
                    app.telem.pitch_out = json_parse_double(line, "pitch_out");
                    app.telem.tgt_speed = json_parse_double(line, "tgt_speed");
                    app.telem.spd_est   = json_parse_double(line, "spd_est");
                    app.telem.spd_err   = json_parse_double(line, "spd_err");
                    app.telem.ang_vel   = json_parse_double(line, "ang_vel");
                    app.telem.ang_acc   = json_parse_double(line, "ang_acc");
                    app.telem.pos_l     = json_parse_double(line, "pos_l");
                    app.telem.pos_r     = json_parse_double(line, "pos_r");
                    app.telem.pos_disp  = json_parse_double(line, "pos_disp");
                    app.telem.pos_target = json_parse_double(line, "pos_target");
                    app.telem.pos_out   = json_parse_double(line, "pos_out");
                    app.telem.cmd_l     = json_parse_double(line, "cmd_l");
                    app.telem.cmd_r     = json_parse_double(line, "cmd_r");
                    app.telem.odrive_usb = json_parse_bool(line, "odrive_usb");
                    app.telem.imu_usb   = json_parse_bool(line, "imu_usb");
                    app.telem.msg       = json_parse_string(line, "msg");
                    app.telem.state    = json_parse_string(line, "state");
                    app.telem.last_update = std::chrono::steady_clock::now();
                    app.armed = (app.telem.state == "BALANCING" ||
                                 app.telem.state == "ARMED" ||
                                 app.telem.state == "CALIBRATING");
                    if (!app.chart_paused) {
                        app.push_chart_data(app.telem);
                    }

                    // Reply with control data immediately — keeps RC timeout alive
                    // and ensures throttle/steering are always up to date on robot
                    char ctrl_buf[256];
                    snprintf(ctrl_buf, sizeof(ctrl_buf),
                        "{\"type\":\"control\",\"throttle\":%.4f,"
                        "\"steering\":%.4f,\"speed_limit\":1.00}",
                        app.throttle, app.steering);
                    app.tcp.send_line(ctrl_buf);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    // ── Command Sending Thread ──────────────────────────────
    std::thread cmd_thread([&]() {
        struct epoll_event events[MAX_EPOLL_EVENTS];
        while (tcp_running.load() && app.running) {
            int nfds = epoll_wait(app.gamepad.epollfd, events, MAX_EPOLL_EVENTS, 10);
            if (nfds < 0) { if (errno == EINTR) continue; break; }

            for (int i = 0; i < nfds; ++i) {
                if (events[i].events & EPOLLIN) {
                    struct input_event ev[MAX_INPUT_EVENTS];
                    ssize_t rd = read(events[i].data.fd, ev, sizeof(ev));
                    if (rd < 0) continue;
                    int count = rd / sizeof(struct input_event);

                    for (int j = 0; j < count; j++) {
                        int event_type = ev[j].type;
                        int event_code = ev[j].code;
                        int event_value = ev[j].value;

                        if (event_type == EV_ABS) {
                            if (event_code == ABS_Y) {
                                app.throttle = -((double)event_value)/32768.0; 
                           } else if (event_code == ABS_X) {
                                app.steering = ((double)event_value)/32768.0;;
                            }
                        }

                        // MENU → ARM
                        if (event_type == EV_KEY && event_code == MENU_BUTTON && event_value == 1) {
                            app.tcp.send_line("{\"type\":\"arm\"}");
                            app.armed = true;
                            app.cursor_pos = WaylandApp::CHART_LEN / 2;  // reset cursor to middle
                            std::cout << "[Cmd] ARM\n";
                        }
                        // KEY_B → DISARM (safe stop, returns to IDLE)
                        if (event_type == EV_KEY && event_code == KEY_B && event_value == 1) {
                            app.tcp.send_line("{\"type\":\"disarm\"}");
                            app.armed = false;
                            std::cout << "[Cmd] DISARM\n";
                        }
                        // KEY_X → E-STOP (emergency, enters FAULT)
                        if (event_type == EV_KEY && event_code == KEY_X && event_value == 1) {
                            app.tcp.send_line("{\"type\":\"estop\"}");
                            app.armed = false;
                            std::cout << "[Cmd] !! EMERGENCY STOP !!\n";
                        }

                        // A → pause charts
                        if (event_type == EV_KEY && event_code == KEY_A && event_value == 1) {
                            app.chart_paused = !app.chart_paused;
                            if (app.chart_paused) {
                                // Initialize cursor to middle of data
                                app.cursor_pos = WaylandApp::CHART_LEN / 2;
                            }
                            std::cout << "[Chart] " << (app.chart_paused ? "PAUSED" : "RUNNING") << "\n";
                        }

                        // LT/RT → move chart cursor (only when paused)
                        if (app.chart_paused) {
                            // LT digital (BTN_TL=310) → cursor left
                            if (event_type == EV_KEY && event_code == 310 && event_value == 1) {
                                if (app.cursor_pos > 0) app.cursor_pos--;
                            }
                            // RT digital (BTN_TR=311) → cursor right
                            if (event_type == EV_KEY && event_code == 311 && event_value == 1) {
                                if (app.cursor_pos < WaylandApp::CHART_LEN - 1) app.cursor_pos++;
                            }
                        }

                        // ── PID Tuning Controls ─────────────────────
                        if (event_type == EV_KEY && event_code == D_PAD_UP && event_value == 1) {
                            app.pid_sel--;
                            if (app.pid_sel < 0) app.pid_sel = app.current_pid_count() - 1;
                        } else if (event_type == EV_KEY && event_code == D_PAD_DOWN && event_value == 1) {
                            app.pid_sel++;
                            if (app.pid_sel >= app.current_pid_count()) app.pid_sel = 0;
                        }

                        auto* params = app.current_pid_params();
                        auto& p = params[app.pid_sel];
                        if (event_type == EV_KEY && event_code == D_PAD_RIGHT && event_value == 1) {
                            p.value += p.step;
                            if (p.value > p.max) p.value = p.max;
                            app.pid_dirty = true;
                        } else if (event_type == EV_KEY && event_code == D_PAD_LEFT && event_value == 1) {
                            p.value -= p.step;
                            if (p.value < p.min) p.value = p.min;
                            app.pid_dirty = true;
                        }

                        if (event_type == EV_KEY && event_code == LB_BUTTON && event_value == 1) {
                            if (app.pid_group == 0) {
                                app.pid_group = WaylandApp::PID_GROUP_COUNT - 1;
                            } else {
                                app.pid_group--;
                            }
                            app.pid_sel = 0;
                            std::cout << "[PID] Group: " << app.pid_group_names[app.pid_group] << "\n";
                        } else if (event_type == EV_KEY && event_code == RB_BUTTON && event_value == 1) {
                            app.pid_group = (app.pid_group + 1) % WaylandApp::PID_GROUP_COUNT;
                            app.pid_sel = 0;
                            std::cout << "[PID] Group: " << app.pid_group_names[app.pid_group] << "\n";
                        }

                        // Y → send PID values
                        if (event_type == EV_KEY && event_code == KEY_Y && event_value == 1 && app.pid_dirty) {
                            char pid_buf[1024];
                            snprintf(pid_buf, sizeof(pid_buf),
                                "{\"type\":\"set_pid\","
                                "\"pitch_kp\":%.4f,\"pitch_ki\":%.4f,\"pitch_kd\":%.4f,"
                                "\"pitch_d_alpha\":%.4f,"
                                "\"yaw_kp\":%.4f,"
                                "\"speed_kp\":%.4f,"
                                "\"pos_kp\":%.4f,\"pos_ki\":%.4f,\"pos_kd\":%.4f,"
                                "\"pos_out_max_fwd\":%.2f,\"pos_out_max_bwd\":%.2f,"
                                "\"tgt_pitch_max_fwd\":%.2f,\"tgt_pitch_max_bwd\":%.2f,"
                                "\"pitch_offset\":%.4f,\"max_velocity\":%.4f}",
                                app.pid_pitch[0].value, app.pid_pitch[1].value, app.pid_pitch[2].value,
                                app.pid_pitch[3].value,
                                app.pid_yaw[0].value,
                                app.pid_speed[0].value,
                                app.pid_pos[0].value, app.pid_pos[1].value, app.pid_pos[2].value,
                                app.pid_limits[0].value, app.pid_limits[1].value,
                                app.pid_limits[2].value, app.pid_limits[3].value,
                                app.pid_other[0].value, app.pid_other[1].value);
                            app.tcp.send_line(pid_buf);
                            app.pid_dirty = false;
                            // Save to file for persistence across restarts
                            app.save_config();
                            std::cout << "[PID] Sent & saved\n";
                        }
                    }
                }
                if (events[i].events & (EPOLLERR | EPOLLHUP)) {
                    fprintf(stderr, "fd=%d got error/hup\n", events[i].data.fd);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RATE_MS));
        }
    });

    // ── Main Render Loop ────────────────────────────────────
    std::cout << "[Main] Running... Press ESC to quit\n";

    while (app.running) {
        wl_display_dispatch_pending(app.display);
        if (wl_display_flush(app.display) < 0 && errno != EAGAIN) break;

        struct pollfd fds[1];
        fds[0].fd = wl_display_get_fd(app.display);
        fds[0].events = POLLIN;
        int ret = poll(fds, 1, RENDER_RATE_MS);
        if (ret > 0) wl_display_dispatch(app.display);

        if (app.configured) {
            int idx = -1;
            for (int i = 0; i < WaylandApp::BUFFER_COUNT; ++i) {
                int candidate = (app.front_buffer + 1 + i) % WaylandApp::BUFFER_COUNT;
                if (!app.buffers[candidate].busy) { idx = candidate; break; }
            }
            if (idx >= 0) {
                auto& b = app.buffers[idx];
                render_hud(app, b.shm_data);
                b.busy = true;
                app.front_buffer = idx;
                wl_surface_attach(app.surface, b.buffer, 0, 0);
                wl_surface_damage_buffer(app.surface, 0, 0, app.width, app.height);
                wl_surface_commit(app.surface);
            }
        }
    }

    // ── Cleanup ─────────────────────────────────────────────
    tcp_running.store(false);
    if (tcp_thread.joinable()) tcp_thread.join();
    if (cmd_thread.joinable()) cmd_thread.join();

    app.tcp.disconnect();
    app.gamepad.epollfd_close();

    for (int i = 0; i < WaylandApp::BUFFER_COUNT; ++i) {
        auto& b = app.buffers[i];
        if (b.buffer) wl_buffer_destroy(b.buffer);
        if (b.shm_data) munmap(b.shm_data, b.shm_size);
        if (b.shm_fd >= 0) close(b.shm_fd);
    }

    if (app.toplevel) xdg_toplevel_destroy(app.toplevel);
    if (app.xdg_surf) xdg_surface_destroy(app.xdg_surf);
    if (app.surface) wl_surface_destroy(app.surface);
    if (app.keyboard) wl_keyboard_destroy(app.keyboard);
    if (app.xdg_base) xdg_wm_base_destroy(app.xdg_base);
    if (app.shm) wl_shm_destroy(app.shm);
    if (app.seat) wl_seat_destroy(app.seat);
    if (app.compositor) wl_compositor_destroy(app.compositor);
    if (app.registry) wl_registry_destroy(app.registry);
    if (app.display) wl_display_disconnect(app.display);

    std::cout << "[Main] Goodbye!\n";
    return 0;
}
