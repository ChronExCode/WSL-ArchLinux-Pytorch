// ═══════════════════════════════════════════════════════════════
//  Robot Remote Controller — MSI Claw 8 AI+ A2VM
//  Wayland Client + evdev Gamepad Input + TCP to Raspberry Pi 5
//
//  Features:
//    • Reads MSI Claw gamepad via evdev (/dev/input/event*)
//    • Auto-detects gamepad device by scanning capabilities
//    • Renders HUD with Cairo on Wayland surface
//    • Sends control commands over WiFi TCP
//    • Receives and displays telemetry
//    • Full-screen Wayland xdg-shell window
// ═══════════════════════════════════════════════════════════════

#include <wayland-client.h>
#include "xdg-shell-protocol.h"

#include <cairo/cairo.h>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <vector>
#include <map>
#include <dirent.h>

// ═════════════════════════════════════════════════════════════
//  Configuration
// ═════════════════════════════════════════════════════════════

static const int    WINDOW_WIDTH  = 1920;
static const int    WINDOW_HEIGHT = 1080;
static const char*  WINDOW_TITLE  = "Robot Controller";
static const int    TCP_PORT      = 9000;
static const int    SEND_RATE_MS  = 20;   // 50 Hz command rate
static const int    RENDER_RATE_MS = 16;  // ~60 FPS

// ═════════════════════════════════════════════════════════════
//  Telemetry data from robot
// ═════════════════════════════════════════════════════════════

struct Telemetry {
    double pitch      = 0;
    double roll       = 0;
    double yaw        = 0;
    double speed      = 0;
    double vbus       = 0;
    double left_vel   = 0;
    double right_vel  = 0;
    double gyro_x     = 0;
    double gyro_y     = 0;
    double gyro_z     = 0;
    double acc_x      = 0;
    double acc_y      = 0;
    double acc_z      = 0;
    // PID debug
    double tgt_pitch  = 0;
    double flt_pitch  = 0;
    double pitch_err  = 0;
    double pitch_p    = 0;
    double pitch_i    = 0;
    double pitch_d    = 0;
    double pitch_out  = 0;
    double tgt_speed  = 0;
    double spd_est    = 0;
    double spd_err    = 0;
    double ang_vel    = 0;    // gyro_y angular velocity (deg/s)
    double ang_acc    = 0;    // angular acceleration (deg/s²)
    double pos_l      = 0;    // left encoder position (turns)
    double pos_r      = 0;    // right encoder position (turns)
    std::string state = "DISCONNECTED";
    std::chrono::steady_clock::time_point last_update;
};

// ═════════════════════════════════════════════════════════════
//  Gamepad Reader — evdev (/dev/input/event*)
//  Uses raw evdev instead of joydev (js*) because:
//    • joydev module may not be loaded on Arch Linux
//    • Some buttons (START/BACK) only appear in evdev
//    • evdev is always available as the native input layer
//  Codes: BTN_START(315)=Start BTN_SELECT(314)=Back
//         BTN_SOUTH(304)=A BTN_EAST(305)=B
//         ABS_X(0)=LStick_X ABS_Y(1)=LStick_Y
//         ABS_Z(2)=LT ABS_RZ(5)=RT
// ═════════════════════════════════════════════════════════════

static bool test_bit(int bit, const unsigned long* array) {
    return (array[bit/(sizeof(long)*8)] >> (bit%(sizeof(long)*8))) & 1;
}

class GamepadReader {
public:
    bool open(const std::string& device) {
        fd_ = ::open(device.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "[Gamepad] Cannot open " << device << ": " << strerror(errno) << "\n";
            return false;
        }
        char devname[256] = "Unknown";
        ioctl(fd_, EVIOCGNAME(sizeof(devname)), devname);
        name_ = devname;
        std::cout << "[Gamepad] Opened: " << name_ << " (" << device << ")\n";
        // Read axis calibration
        for (int c : {ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_HAT0X, ABS_HAT0Y}) {
            struct input_absinfo info;
            if (ioctl(fd_, EVIOCGABS(c), &info) == 0) {
                abs_info_[c] = {info.minimum, info.maximum, info.flat};
                printf("  ABS_%d: min=%d max=%d flat=%d\n", c, info.minimum, info.maximum, info.flat);
            }
        }
        return true;
    }

    /// Auto-detect gamepad by scanning /dev/input/event*
    bool auto_detect() {
        std::cout << "[Gamepad] Scanning /dev/input/event* ...\n";
        DIR* dir = opendir("/dev/input");
        if (!dir) { std::cerr << "[Gamepad] Cannot open /dev/input\n"; return false; }
        struct dirent* entry;
        std::vector<std::string> candidates;
        while ((entry = readdir(dir)) != nullptr) {
            std::string n = entry->d_name;
            if (n.substr(0, 5) != "event") continue;
            std::string path = "/dev/input/" + n;
            int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0) continue;
            unsigned long evbits[(EV_MAX+8*sizeof(long)-1)/(8*sizeof(long))] = {};
            ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits);
            if (test_bit(EV_ABS, evbits) && test_bit(EV_KEY, evbits)) {
                unsigned long absbits[(ABS_MAX+8*sizeof(long)-1)/(8*sizeof(long))] = {};
                ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(absbits)), absbits);
                unsigned long keybits[(KEY_MAX+8*sizeof(long)-1)/(8*sizeof(long))] = {};
                ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);
                bool has_stick = test_bit(ABS_X, absbits) && test_bit(ABS_Y, absbits);
                bool has_btn = test_bit(BTN_SOUTH, keybits) || test_bit(BTN_A, keybits)
                             || test_bit(BTN_GAMEPAD, keybits) || test_bit(BTN_START, keybits);
                char devname[256] = "";
                ioctl(fd, EVIOCGNAME(sizeof(devname)), devname);
                if (has_stick && has_btn) {
                    std::cout << "  [OK] " << path << ": " << devname << "\n";
                    candidates.push_back(path);
                }
            }
            ::close(fd);
        }
        closedir(dir);
        if (candidates.empty()) {
            std::cerr << "[Gamepad] No gamepad found!\n"
                      << "  Try: ls -la /dev/input/event*\n"
                      << "  Add user to input group: sudo usermod -aG input $USER\n";
            return false;
        }
        return open(candidates[0]);
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    /// Poll events, update internal axis/button state.
    /// Returns true if any event was read.
    bool poll() {
        if (fd_ < 0) return false;
        bool got = false;
        struct input_event ev;
        while (::read(fd_, &ev, sizeof(ev)) == sizeof(ev)) {
            if (ev.type == EV_ABS) {
                std::lock_guard<std::mutex> lock(mtx_);
                axes_[ev.code] = normalize_axis(ev.code, ev.value);
                got = true;
            } else if (ev.type == EV_KEY) {
                std::lock_guard<std::mutex> lock(mtx_);
                int old_val = buttons_[ev.code];
                buttons_[ev.code] = ev.value;
                got = true;
                if (ev.value == 1 && old_val == 0) {
                    std::cout << "[Gamepad] Key " << ev.code
                              << " (" << key_name(ev.code) << ") PRESSED\n";
                }
            }
        }
        return got;
    }

    /// Get axis value (-1.0 to +1.0)
    double axis(int num) const {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = axes_.find(num);
        return (it != axes_.end()) ? it->second : 0.0;
    }

    /// Get button state (true = pressed)
    bool button(int num) const {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = buttons_.find(num);
        return (it != buttons_.end()) ? it->second != 0 : false;
    }

    bool is_open() const { return fd_ >= 0; }
    const std::string& name() const { return name_; }

    /// Print current axis/button state (debug)
    void dump_state() const {
        std::lock_guard<std::mutex> lock(mtx_);
        std::cout << "[Joy] Axes:";
        for (auto& [k, v] : axes_) {
            printf(" %d=%.2f", k, v);
        }
        std::cout << "\n[Joy] Btns:";
        for (auto& [k, v] : buttons_) {
            if (v) printf(" %d=ON", k);
        }
        std::cout << "\n";
    }

private:
    int fd_ = -1;
    std::string name_;
    mutable std::mutex mtx_;
    std::map<int, double> axes_;    // EV_ABS code → normalized
    std::map<int, int> buttons_;    // EV_KEY code → value
    struct AbsInfo { int min=0, max=0, flat=0; };
    std::map<int, AbsInfo> abs_info_;

    double normalize_axis(int code, int value) const {
        auto it = abs_info_.find(code);
        if (it == abs_info_.end()) return value / 32767.0;
        const auto& info = it->second;
        int range = info.max - info.min;
        if (range == 0) return 0.0;

        // Triggers (ABS_Z, ABS_RZ) are always 0..1
        if (code == ABS_Z || code == ABS_RZ) {
            return static_cast<double>(value - info.min) / range;
        }

        // Sticks: map to -1.0..+1.0 centered
        // Works for both XInput (-32768..32767) and DInput (0..255)
        double center = (info.min + info.max) / 2.0;
        double half = range / 2.0;
        double norm = (value - center) / half;
        if (info.flat > 0 && std::abs(norm) < (info.flat / half)) return 0.0;
        return std::max(-1.0, std::min(1.0, norm));
    }

    static const char* key_name(int code) {
        switch (code) {
            case BTN_SOUTH:  return "A";     case BTN_EAST:   return "B";
            case BTN_NORTH:  return "X";     case BTN_WEST:   return "Y";
            case BTN_TL:     return "LB";    case BTN_TR:     return "RB";
            case BTN_SELECT: return "Back";  case BTN_START:  return "Start";
            case BTN_MODE:   return "Home";  case BTN_THUMBL: return "LS";
            case BTN_THUMBR: return "RS";    default: return "?";
        }
    }
};

// ═════════════════════════════════════════════════════════════
//  TCP Client (to Raspberry Pi 5)
// ═════════════════════════════════════════════════════════════

class TcpClient {
public:
    bool connect(const std::string& host, uint16_t port) {
        // Clean up any stale connection
        close_fd();
        partial_.clear();

        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) return false;

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

        // Non-blocking connect with timeout
        fcntl(fd_, F_SETFL, O_NONBLOCK);

        int ret = ::connect(fd_, (struct sockaddr*)&addr, sizeof(addr));
        if (ret < 0 && errno != EINPROGRESS) {
            close_fd();
            return false;
        }

        // Wait for connection
        struct pollfd pfd = { fd_, POLLOUT, 0 };
        ret = ::poll(&pfd, 1, 3000);  // 3s timeout
        if (ret <= 0) {
            close_fd();
            return false;
        }

        // Check for error
        int err;
        socklen_t len = sizeof(err);
        getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err != 0) {
            close_fd();
            return false;
        }

        // Back to blocking, TCP_NODELAY
        fcntl(fd_, F_SETFL, 0);
        int flag = 1;
        setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        connected_ = true;
        return true;
    }

    void disconnect() {
        close_fd();
        partial_.clear();
    }

    bool send_line(const std::string& line) {
        if (!connected_ || fd_ < 0) return false;
        std::string data = line + "\n";
        ssize_t n = ::send(fd_, data.c_str(), data.size(), MSG_NOSIGNAL);
        if (n <= 0) { on_error(); return false; }
        return true;
    }

    /// Read available lines (non-blocking)
    std::vector<std::string> recv_lines() {
        std::vector<std::string> lines;
        if (!connected_ || fd_ < 0) return lines;

        struct pollfd pfd = { fd_, POLLIN, 0 };
        while (::poll(&pfd, 1, 0) > 0) {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                on_error();
                break;
            }
            char buf[4096];
            ssize_t n = ::recv(fd_, buf, sizeof(buf) - 1, 0);
            if (n <= 0) { on_error(); break; }
            buf[n] = '\0';
            partial_ += buf;
        }

        size_t pos;
        while ((pos = partial_.find('\n')) != std::string::npos) {
            lines.push_back(partial_.substr(0, pos));
            partial_ = partial_.substr(pos + 1);
        }

        return lines;
    }

    bool is_connected() const { return connected_; }

private:
    int fd_ = -1;
    bool connected_ = false;
    std::string partial_;

    void close_fd() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        connected_ = false;
    }

    void on_error() {
        close_fd();
        partial_.clear();
    }
};

// ═════════════════════════════════════════════════════════════
//  Wayland App State
// ═════════════════════════════════════════════════════════════

struct WaylandApp {
    // Wayland globals
    wl_display*     display     = nullptr;
    wl_registry*    registry    = nullptr;
    wl_compositor*  compositor  = nullptr;
    wl_shm*         shm         = nullptr;
    wl_seat*        seat        = nullptr;
    wl_keyboard*    keyboard    = nullptr;
    xdg_wm_base*   xdg_base    = nullptr;

    // Surface
    wl_surface*     surface     = nullptr;
    xdg_surface*    xdg_surf    = nullptr;
    xdg_toplevel*   toplevel    = nullptr;

    // Buffer
    wl_buffer*      buffer      = nullptr;
    void*           shm_data    = nullptr;
    int             shm_fd      = -1;
    size_t          shm_size    = 0;

    // State
    bool running    = true;
    bool configured = false;
    int  width      = WINDOW_WIDTH;
    int  height     = WINDOW_HEIGHT;

    // Input
    GamepadReader gamepad;

    // Network
    TcpClient tcp;
    std::string server_ip = "10.0.0.218";
    bool armed = false;
    bool chart_paused = false;  // A toggles chart freeze

    // Telemetry
    std::mutex telem_mtx;
    Telemetry telem;

    // Gamepad mappings — evdev codes (linux/input.h)
    // MSI Claw 8 AI+ in DInput mode (0db0:1902) mapping:
    //
    //   Physical       evdev code   Kernel name
    //   ─────────────────────────────────────────
    //   X (left)       304          BTN_SOUTH
    //   A (bottom)     305          BTN_EAST
    //   B (right)      306          BTN_C
    //   Y (top)        307          BTN_NORTH
    //   LB             308          BTN_WEST
    //   RB             309          BTN_Z
    //   LT digital     310          BTN_TL
    //   RT digital     311          BTN_TR
    //   View           312          BTN_TL2
    //   Menu (≡)       313          BTN_TR2
    //   L-Stick click  314          BTN_SELECT
    //   R-Stick click  315          BTN_START
    //   LT analog      ABS_Z (2)   0.0 ~ 1.0
    //   RT analog      ABS_RZ (5)  0.0 ~ 1.0
    //   L-Stick        ABS_X/ABS_Y
    //   R-Stick        ABS_RX/ABS_RY
    //   D-Pad          ABS_HAT0X/ABS_HAT0Y
    //   M1 (back)      305          same as A
    //   M2 (back)      306          same as B
    //   MSI button     (not on evdev — goes through WMI)
    //
    // Override with: --btn-arm 313 --btn-estop 312 --btn-disarm 306
    int axis_steering  = ABS_X;      // Left stick X
    int axis_throttle  = ABS_Y;      // Left stick Y (inverted)
    int axis_speed_lt  = ABS_Z;      // Left trigger analog
    int axis_speed_rt  = ABS_RZ;     // Right trigger analog
    int btn_arm        = 313;        // Menu (≡) button
    int btn_estop      = 312;        // View button
    int btn_disarm     = 306;        // B (right) button

    bool joy_map_mode  = false;

    // MSI Claw requires HID mode switch from XInput to DInput
    bool auto_switch_dinput = true;  // send HID command on startup

    // ── PID Tuning ──────────────────────────────────────────
    // D-Pad up/down: select parameter
    // D-Pad left/right: adjust value
    // Y: send PID to server
    // RB: cycle PID group
    struct PidParam {
        const char* label;
        double value;
        double step;       // increment per D-Pad press
        double min, max;
    };
    // Group 0: Pitch PID, Group 1: Yaw PID, Group 2: Speed PID, Group 3: Other
    static constexpr int PID_GROUP_COUNT = 4;
    const char* pid_group_names[PID_GROUP_COUNT] = {"PITCH PID", "YAW PID", "SPEED PID", "OTHER"};
    int pid_group = 0;        // current group
    int pid_sel   = 0;        // selected param within group

    // Pitch PID
    PidParam pid_pitch[3] = {
        {"Kp",  0.30, 0.01, 0.0, 5.0},
        {"Ki",  0.005,0.005,0.0, 1.0},
        {"Kd",  0.025,0.005,0.0, 1.0},
    };
    // Yaw PID
    PidParam pid_yaw[3] = {
        {"Kp",  1.0,  0.1,  0.0, 20.0},
        {"Ki",  0.0,  0.01, 0.0,  5.0},
        {"Kd",  0.0,  0.01, 0.0,  5.0},
    };
    // Speed PID (outer loop: anti-drift)
    PidParam pid_speed[3] = {
        {"Kp",   0.6, 0.01, 0.0,  5.0},
        {"Ki",   0.0, 0.005,0.0,  2.0},
        {"Kd",   0.0, 0.005,0.0,  2.0},
    };
    // Other params
    PidParam pid_other[2] = {
        {"pitch_offset", 0.0, 0.1, -10.0, 10.0},
        {"max_velocity", 2.0, 0.5,  0.5, 20.0},
    };

    PidParam* current_pid_params() {
        switch (pid_group) {
            case 0: return pid_pitch;
            case 1: return pid_yaw;
            case 2: return pid_speed;
            case 3: return pid_other;
            default: return pid_pitch;
        }
    }
    int current_pid_count() const {
        switch (pid_group) {
            case 0: case 1: case 2: return 3;
            case 3: return 2;
            default: return 3;
        }
    }
    bool pid_dirty = false;  // true when values changed but not sent

    // ── Chart History ───────────────────────────────────────
    static constexpr int CHART_LEN = 200;  // ~10 seconds at 20Hz
    struct ChartHistory {
        double data[200] = {};
        int head = 0;
        int count = 0;
        void push(double v) {
            data[head] = v;
            head = (head + 1) % 200;
            if (count < 200) count++;
        }
        double get(int i) const {  // i=0 is oldest
            if (i < 0 || i >= count) return 0;
            return data[(head - count + i + 200) % 200];
        }
        void clear() { head = 0; count = 0; }
    };
    // Chart traces
    ChartHistory ch_pitch_out;     // motor velocity command (pitch PID output)
    ChartHistory ch_left_vel;      // left motor velocity
    ChartHistory ch_right_vel;     // right motor velocity
    ChartHistory ch_pitch_err;     // pitch error
    ChartHistory ch_tgt_pitch;     // target pitch
    ChartHistory ch_flt_pitch;     // filtered pitch
    ChartHistory ch_tgt_speed;     // target speed
    ChartHistory ch_spd_est;       // speed estimate
    ChartHistory ch_ang_vel;       // angular velocity (gyro_y)
    ChartHistory ch_ang_acc;       // angular acceleration
    ChartHistory ch_pos_l;         // left encoder position
    ChartHistory ch_pos_r;         // right encoder position

    std::string prev_state;        // track state transitions

    void push_chart_data(const Telemetry& t) {
        // Clear charts on state transition to BALANCING
        if (t.state != prev_state) {
            if (t.state == "BALANCING") {
                clear_all_charts();
            }
            prev_state = t.state;
        }
        // Only record data during BALANCING — FAULT/IDLE values
        // have extreme spikes that corrupt the chart auto-scale
        if (t.state != "BALANCING") return;

        ch_pitch_out.push(t.pitch_out);
        ch_left_vel.push(t.left_vel);
        ch_right_vel.push(t.right_vel);
        ch_pitch_err.push(t.pitch_err);
        ch_tgt_pitch.push(t.tgt_pitch);
        ch_flt_pitch.push(t.flt_pitch);
        ch_tgt_speed.push(t.tgt_speed);
        ch_spd_est.push(t.spd_est);
        ch_ang_vel.push(t.ang_vel);
        ch_ang_acc.push(t.ang_acc);
        ch_pos_l.push(t.pos_l);
        ch_pos_r.push(t.pos_r);
    }

    void clear_all_charts() {
        ch_pitch_out.clear(); ch_left_vel.clear(); ch_right_vel.clear();
        ch_pitch_err.clear(); ch_tgt_pitch.clear(); ch_flt_pitch.clear();
        ch_tgt_speed.clear(); ch_spd_est.clear();
        ch_ang_vel.clear(); ch_ang_acc.clear();
        ch_pos_l.clear(); ch_pos_r.clear();
    }
};

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

static wl_buffer* create_buffer(WaylandApp& app) {
    int stride = app.width * 4;  // ARGB8888
    app.shm_size = stride * app.height;

    app.shm_fd = create_shm_file(app.shm_size);
    if (app.shm_fd < 0) return nullptr;

    app.shm_data = mmap(nullptr, app.shm_size,
                         PROT_READ | PROT_WRITE, MAP_SHARED,
                         app.shm_fd, 0);
    if (app.shm_data == MAP_FAILED) {
        close(app.shm_fd);
        return nullptr;
    }

    wl_shm_pool* pool = wl_shm_create_pool(app.shm, app.shm_fd, app.shm_size);
    wl_buffer* buffer = wl_shm_pool_create_buffer(
        pool, 0, app.width, app.height, stride, WL_SHM_FORMAT_ARGB8888);
    wl_shm_pool_destroy(pool);

    return buffer;
}

// ═════════════════════════════════════════════════════════════
//  HUD Rendering with Cairo
// ═════════════════════════════════════════════════════════════

static double json_parse_double(const std::string& json, const std::string& key, double def = 0.0) {
    std::string search = "\"" + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return def;
    pos += search.size();
    while (pos < json.size() && json[pos] == ' ') pos++;
    try { return std::stod(json.substr(pos)); } catch (...) { return def; }
}

static std::string json_parse_string(const std::string& json, const std::string& key) {
    std::string search = "\"" + key + "\":\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return {};
    pos += search.size();
    size_t end = json.find('"', pos);
    if (end == std::string::npos) return {};
    return json.substr(pos, end - pos);
}

static void render_hud(WaylandApp& app) {
    int w = app.width;
    int h = app.height;
    int stride = w * 4;

    cairo_surface_t* cs = cairo_image_surface_create_for_data(
        (unsigned char*)app.shm_data,
        CAIRO_FORMAT_ARGB32, w, h, stride);
    cairo_t* cr = cairo_create(cs);

    // ── Background ──────────────────────────────────────────
    // Dark gradient background
    cairo_pattern_t* bg = cairo_pattern_create_linear(0, 0, 0, h);
    cairo_pattern_add_color_stop_rgb(bg, 0.0, 0.05, 0.05, 0.10);
    cairo_pattern_add_color_stop_rgb(bg, 1.0, 0.02, 0.02, 0.06);
    cairo_set_source(cr, bg);
    cairo_paint(cr);
    cairo_pattern_destroy(bg);

    // Grid overlay
    cairo_set_source_rgba(cr, 0.15, 0.25, 0.35, 0.15);
    cairo_set_line_width(cr, 0.5);
    for (int x = 0; x < w; x += 40) {
        cairo_move_to(cr, x, 0);
        cairo_line_to(cr, x, h);
    }
    for (int y = 0; y < h; y += 40) {
        cairo_move_to(cr, 0, y);
        cairo_line_to(cr, w, y);
    }
    cairo_stroke(cr);

    // Get data
    Telemetry telem;
    {
        std::lock_guard<std::mutex> lock(app.telem_mtx);
        telem = app.telem;
    }
    double throttle = -app.gamepad.axis(app.axis_throttle);  // Y inverted
    double steering = app.gamepad.axis(app.axis_steering);
    double lt = app.gamepad.axis(app.axis_speed_lt);
    double rt = app.gamepad.axis(app.axis_speed_rt);
    double speed_limit = 1.0 - (lt + rt) * 0.5;
    if (speed_limit < 0.1) speed_limit = 0.1;

    // ── Title Bar ───────────────────────────────────────────
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 28);

    // Connection status indicator
    bool connected = app.tcp.is_connected();
    bool data_fresh = false;
    {
        auto age = std::chrono::steady_clock::now() - telem.last_update;
        data_fresh = std::chrono::duration_cast<std::chrono::seconds>(age).count() < 2;
    }

    // Status dot
    if (connected && data_fresh) {
        cairo_set_source_rgb(cr, 0.2, 0.9, 0.4);  // green
    } else if (connected) {
        cairo_set_source_rgb(cr, 0.9, 0.7, 0.1);  // yellow
    } else {
        cairo_set_source_rgb(cr, 0.9, 0.2, 0.2);  // red
    }
    cairo_arc(cr, 30, 35, 10, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 0.85, 0.9, 0.95);
    cairo_move_to(cr, 52, 44);
    cairo_show_text(cr, "ROBOT CONTROL");

    // State
    if (telem.state == "BALANCING") {
        cairo_set_source_rgb(cr, 0.2, 0.9, 0.4);
    } else if (telem.state == "ARMED" || telem.state == "CALIBRATING") {
        cairo_set_source_rgb(cr, 0.9, 0.7, 0.1);
    } else if (telem.state == "FAULT") {
        cairo_set_source_rgb(cr, 0.9, 0.2, 0.2);
    } else if (telem.state == "DISCONNECTED") {
        // Blink red/dark every ~500ms
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if ((ms / 500) % 2 == 0) {
            cairo_set_source_rgb(cr, 0.9, 0.15, 0.1);
        } else {
            cairo_set_source_rgb(cr, 0.4, 0.08, 0.05);
        }
    } else {
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.6);
    }
    cairo_set_font_size(cr, 24);
    cairo_move_to(cr, 350, 44);
    cairo_show_text(cr, telem.state.c_str());

    // Server IP
    cairo_set_source_rgb(cr, 0.5, 0.6, 0.7);
    cairo_set_font_size(cr, 16);
    char ip_buf[128];
    snprintf(ip_buf, sizeof(ip_buf), "Server: %s:%d",
             app.server_ip.c_str(), TCP_PORT);
    cairo_move_to(cr, w - 350, 44);
    cairo_show_text(cr, ip_buf);

    // ── Artificial Horizon (center, above charts) ─────────
    double cx = w / 2.0;
    double cy = h / 2.0 - 80;
    double horizon_r = 150;

    // Outer ring
    cairo_set_source_rgba(cr, 0.2, 0.4, 0.6, 0.3);
    cairo_set_line_width(cr, 2);
    cairo_arc(cr, cx, cy, horizon_r + 5, 0, 2 * M_PI);
    cairo_stroke(cr);

    // Clipping circle
    cairo_save(cr);
    cairo_arc(cr, cx, cy, horizon_r, 0, 2 * M_PI);
    cairo_clip(cr);

    // Sky / ground split based on pitch
    double pitch_px = telem.pitch * 3.0;  // pixels per degree
    double roll_rad = telem.roll * M_PI / 180.0;

    cairo_save(cr);
    cairo_translate(cr, cx, cy);
    cairo_rotate(cr, roll_rad);

    // Sky
    cairo_set_source_rgb(cr, 0.1, 0.15, 0.35);
    cairo_rectangle(cr, -horizon_r - 50, -horizon_r - 200 - pitch_px,
                    2 * (horizon_r + 50), horizon_r + 200);
    cairo_fill(cr);

    // Ground
    cairo_set_source_rgb(cr, 0.15, 0.10, 0.05);
    cairo_rectangle(cr, -horizon_r - 50, -pitch_px,
                    2 * (horizon_r + 50), horizon_r + 200);
    cairo_fill(cr);

    // Horizon line
    cairo_set_source_rgb(cr, 0.9, 0.9, 0.9);
    cairo_set_line_width(cr, 2);
    cairo_move_to(cr, -horizon_r, -pitch_px);
    cairo_line_to(cr, horizon_r, -pitch_px);
    cairo_stroke(cr);

    // Pitch markers
    cairo_set_font_size(cr, 12);
    for (int deg = -30; deg <= 30; deg += 10) {
        if (deg == 0) continue;
        double y = -deg * 3.0 - pitch_px;
        double half_w = (deg % 20 == 0) ? 50 : 30;
        cairo_set_source_rgba(cr, 0.8, 0.8, 0.8, 0.7);
        cairo_set_line_width(cr, 1);
        cairo_move_to(cr, -half_w, y);
        cairo_line_to(cr, half_w, y);
        cairo_stroke(cr);

        char lbl[8];
        snprintf(lbl, sizeof(lbl), "%d", deg);
        cairo_move_to(cr, half_w + 5, y + 4);
        cairo_show_text(cr, lbl);
    }

    cairo_restore(cr);

    // Center reticle (fixed)
    cairo_set_source_rgb(cr, 0.0, 1.0, 0.4);
    cairo_set_line_width(cr, 2.5);
    cairo_move_to(cr, -60, 0);
    cairo_line_to(cr, -15, 0);
    cairo_line_to(cr, -8, 10);
    cairo_move_to(cr, 60, 0);
    cairo_line_to(cr, 15, 0);
    cairo_line_to(cr, 8, 10);
    cairo_move_to(cr, 0, -3);
    cairo_line_to(cr, 0, 3);
    // These are relative to center, so we need to translate
    // Actually let's redo with absolute coords
    cairo_restore(cr);  // restore clip

    // Draw center reticle on top
    cairo_set_source_rgb(cr, 0.0, 1.0, 0.4);
    cairo_set_line_width(cr, 2.5);
    cairo_move_to(cr, cx - 60, cy);
    cairo_line_to(cr, cx - 15, cy);
    cairo_line_to(cr, cx - 8, cy + 10);
    cairo_stroke(cr);
    cairo_move_to(cr, cx + 60, cy);
    cairo_line_to(cr, cx + 15, cy);
    cairo_line_to(cr, cx + 8, cy + 10);
    cairo_stroke(cr);

    // ── Left Panel: Joystick Visualizer ─────────────────────
    double lp_x = 150;
    double lp_y = cy;
    double stick_r = 100;

    // Background circle
    cairo_set_source_rgba(cr, 0.1, 0.15, 0.2, 0.8);
    cairo_arc(cr, lp_x, lp_y, stick_r + 10, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_set_source_rgba(cr, 0.3, 0.5, 0.7, 0.3);
    cairo_set_line_width(cr, 1.5);
    cairo_arc(cr, lp_x, lp_y, stick_r, 0, 2 * M_PI);
    cairo_stroke(cr);

    // Crosshair
    cairo_set_source_rgba(cr, 0.3, 0.4, 0.5, 0.5);
    cairo_move_to(cr, lp_x - stick_r, lp_y);
    cairo_line_to(cr, lp_x + stick_r, lp_y);
    cairo_move_to(cr, lp_x, lp_y - stick_r);
    cairo_line_to(cr, lp_x, lp_y + stick_r);
    cairo_stroke(cr);

    // Stick position
    double sx = lp_x + steering * stick_r;
    double sy = lp_y - throttle * stick_r;  // already inverted above

    cairo_set_source_rgb(cr, 0.0, 0.9, 0.5);
    cairo_arc(cr, sx, sy, 12, 0, 2 * M_PI);
    cairo_fill(cr);

    // Label
    cairo_set_source_rgb(cr, 0.6, 0.7, 0.8);
    cairo_set_font_size(cr, 14);
    cairo_move_to(cr, lp_x - 30, lp_y + stick_r + 30);
    cairo_show_text(cr, "LEFT STICK");

    char stick_val[64];
    snprintf(stick_val, sizeof(stick_val), "T:%.2f S:%.2f", throttle, steering);
    cairo_move_to(cr, lp_x - 45, lp_y + stick_r + 50);
    cairo_show_text(cr, stick_val);

    // ── Left Panel: PID Tuning ──────────────────────────────
    {
        double pp_x = 30;
        double pp_y = lp_y + stick_r + 80;
        double pp_w = 240;
        int n_params = app.current_pid_count();
        double pp_h = 40 + n_params * 38 + 30;

        // Panel background
        cairo_set_source_rgba(cr, 0.05, 0.08, 0.12, 0.85);
        cairo_rectangle(cr, pp_x, pp_y, pp_w, pp_h);
        cairo_fill(cr);

        // Border — highlight if dirty (unsent changes)
        if (app.pid_dirty) {
            cairo_set_source_rgba(cr, 0.9, 0.6, 0.1, 0.8);
        } else {
            cairo_set_source_rgba(cr, 0.2, 0.4, 0.6, 0.5);
        }
        cairo_set_line_width(cr, 1);
        cairo_rectangle(cr, pp_x, pp_y, pp_w, pp_h);
        cairo_stroke(cr);

        // Group title
        cairo_set_source_rgb(cr, 0.7, 0.85, 1.0);
        cairo_set_font_size(cr, 15);
        cairo_move_to(cr, pp_x + 10, pp_y + 22);
        char group_title[64];
        snprintf(group_title, sizeof(group_title), "◀ %s ▶",
                 app.pid_group_names[app.pid_group]);
        cairo_show_text(cr, group_title);

        // Dirty indicator
        if (app.pid_dirty) {
            cairo_set_source_rgb(cr, 0.9, 0.6, 0.1);
            cairo_set_font_size(cr, 12);
            cairo_move_to(cr, pp_x + pp_w - 65, pp_y + 22);
            cairo_show_text(cr, "[Y] Send");
        }

        // Parameter rows
        auto* params = app.current_pid_params();
        for (int i = 0; i < n_params; i++) {
            double row_y = pp_y + 40 + i * 38;
            bool selected = (i == app.pid_sel);

            // Selection highlight
            if (selected) {
                cairo_set_source_rgba(cr, 0.15, 0.25, 0.4, 0.8);
                cairo_rectangle(cr, pp_x + 5, row_y - 2, pp_w - 10, 34);
                cairo_fill(cr);

                // Selection arrow
                cairo_set_source_rgb(cr, 0.0, 0.9, 0.5);
                cairo_set_font_size(cr, 16);
                cairo_move_to(cr, pp_x + 10, row_y + 20);
                cairo_show_text(cr, "▶");
            }

            // Label
            cairo_set_source_rgba(cr, 0.5, 0.6, 0.7, selected ? 1.0 : 0.7);
            cairo_set_font_size(cr, 13);
            cairo_move_to(cr, pp_x + 28, row_y + 12);
            cairo_show_text(cr, params[i].label);

            // Value
            if (selected) {
                cairo_set_source_rgb(cr, 0.0, 0.95, 0.5);
            } else {
                cairo_set_source_rgb(cr, 0.8, 0.85, 0.9);
            }
            cairo_set_font_size(cr, 20);
            char val_buf[32];
            if (params[i].step >= 1.0) {
                snprintf(val_buf, sizeof(val_buf), "%.1f", params[i].value);
            } else if (params[i].step >= 0.01) {
                snprintf(val_buf, sizeof(val_buf), "%.2f", params[i].value);
            } else {
                snprintf(val_buf, sizeof(val_buf), "%.3f", params[i].value);
            }
            cairo_move_to(cr, pp_x + 100, row_y + 22);
            cairo_show_text(cr, val_buf);

            // Step indicator for selected
            if (selected) {
                cairo_set_source_rgba(cr, 0.4, 0.5, 0.6, 0.8);
                cairo_set_font_size(cr, 11);
                char step_buf[32];
                snprintf(step_buf, sizeof(step_buf), "±%.3g", params[i].step);
                cairo_move_to(cr, pp_x + 190, row_y + 22);
                cairo_show_text(cr, step_buf);
            }
        }

        // Bottom hint
        cairo_set_source_rgba(cr, 0.4, 0.5, 0.6, 0.6);
        cairo_set_font_size(cr, 11);
        cairo_move_to(cr, pp_x + 10, pp_y + pp_h - 8);
        cairo_show_text(cr, "D-Pad:Select/Adjust  RB:Group  Y:Send");
    }

    // ── Right Panel: Telemetry ──────────────────────────────
    double rp_x = w - 300;
    double rp_y = 60;

    // Panel background
    cairo_set_source_rgba(cr, 0.05, 0.08, 0.12, 0.85);
    cairo_rectangle(cr, rp_x - 10, rp_y - 10, 280, 780);
    cairo_fill(cr);

    cairo_set_source_rgba(cr, 0.2, 0.4, 0.6, 0.5);
    cairo_set_line_width(cr, 1);
    cairo_rectangle(cr, rp_x - 10, rp_y - 10, 280, 780);
    cairo_stroke(cr);

    // Telemetry items
    struct TelItem {
        const char* label;
        char value[32];
        double r, g, b;
    };

    TelItem items[] = {
        {"PITCH",     {}, 0.9, 0.9, 0.3},
        {"ROLL",      {}, 0.3, 0.9, 0.9},
        {"YAW",       {}, 0.9, 0.5, 0.9},
        {"GYRO X",    {}, 0.9, 0.7, 0.5},
        {"GYRO Y",    {}, 0.9, 0.7, 0.5},
        {"GYRO Z",    {}, 0.9, 0.7, 0.5},
        {"ACC X",     {}, 0.6, 0.9, 0.7},
        {"ACC Y",     {}, 0.6, 0.9, 0.7},
        {"ACC Z",     {}, 0.6, 0.9, 0.7},
        {"SPEED",     {}, 0.3, 0.9, 0.5},
        {"VBUS",      {}, 0.9, 0.6, 0.3},
        {"L MOTOR",   {}, 0.5, 0.7, 0.9},
        {"R MOTOR",   {}, 0.5, 0.7, 0.9},
        {"SPD LIMIT", {}, 0.7, 0.3, 0.9},
        {"ARMED",     {}, 0.9, 0.3, 0.3},
    };
    const int n_items = sizeof(items) / sizeof(items[0]);

    snprintf(items[0].value,  32, "%.1f°",  telem.pitch);
    snprintf(items[1].value,  32, "%.1f°",  telem.roll);
    snprintf(items[2].value,  32, "%.1f°",  telem.yaw);
    snprintf(items[3].value,  32, "%.1f°/s", telem.gyro_x);
    snprintf(items[4].value,  32, "%.1f°/s", telem.gyro_y);
    snprintf(items[5].value,  32, "%.1f°/s", telem.gyro_z);
    snprintf(items[6].value,  32, "%.2f g",  telem.acc_x);
    snprintf(items[7].value,  32, "%.2f g",  telem.acc_y);
    snprintf(items[8].value,  32, "%.2f g",  telem.acc_z);
    snprintf(items[9].value,  32, "%.1f",    telem.speed);
    snprintf(items[10].value, 32, "%.1fV",   telem.vbus);
    snprintf(items[11].value, 32, "%.1f",    telem.left_vel);
    snprintf(items[12].value, 32, "%.1f",    telem.right_vel);
    snprintf(items[13].value, 32, "%.0f%%",  speed_limit * 100);
    snprintf(items[14].value, 32, "%s",      app.armed ? "YES" : "NO");

    cairo_set_font_size(cr, 13);
    for (int i = 0; i < n_items; ++i) {
        double y = rp_y + 12 + i * 50;

        // Label
        cairo_set_source_rgba(cr, 0.5, 0.6, 0.7, 0.8);
        cairo_move_to(cr, rp_x, y);
        cairo_show_text(cr, items[i].label);

        // Value
        cairo_set_source_rgb(cr, items[i].r, items[i].g, items[i].b);
        cairo_set_font_size(cr, 24);
        cairo_move_to(cr, rp_x, y + 26);
        cairo_show_text(cr, items[i].value);
        cairo_set_font_size(cr, 13);
    }

    // ── Charts: Real-time PID Debug ─────────────────────────
    // Helper lambda to draw a chart
    // Helper: compute auto Y-range from chart data
    auto auto_range = [](std::initializer_list<const WaylandApp::ChartHistory*> hists) -> double {
        double max_val = 0.1;  // minimum range
        for (auto* h : hists) {
            for (int i = 0; i < h->count; i++) {
                double v = std::abs(h->get(i));
                if (v > max_val) max_val = v;
            }
        }
        // Round up to nice number
        if (max_val < 1.0) return std::ceil(max_val * 10) / 10.0;
        if (max_val < 10.0) return std::ceil(max_val);
        return std::ceil(max_val / 5) * 5;
    };

    auto draw_chart = [&](double cx, double cy, double cw, double ch_h,
                          const char* title,
                          std::initializer_list<std::pair<const WaylandApp::ChartHistory*, const char*>> traces,
                          std::initializer_list<std::tuple<double, double, double>> colors,
                          double y_range) {
        // Auto-scale: compute from data if y_range <= 0
        if (y_range <= 0) {
            double max_val = 0.1;
            for (auto& [hist, name] : traces) {
                for (int i = 0; i < hist->count; i++) {
                    double v = std::abs(hist->get(i));
                    if (v > max_val) max_val = v;
                }
            }
            if (max_val < 1.0) y_range = std::ceil(max_val * 10) / 10.0;
            else if (max_val < 10.0) y_range = std::ceil(max_val);
            else y_range = std::ceil(max_val / 5) * 5;
        }

        // Background
        cairo_set_source_rgba(cr, 0.04, 0.06, 0.10, 0.85);
        cairo_rectangle(cr, cx, cy, cw, ch_h);
        cairo_fill(cr);

        // Border
        cairo_set_source_rgba(cr, 0.2, 0.35, 0.5, 0.5);
        cairo_set_line_width(cr, 1);
        cairo_rectangle(cr, cx, cy, cw, ch_h);
        cairo_stroke(cr);

        // Zero line
        double zero_y = cy + ch_h / 2.0;
        cairo_set_source_rgba(cr, 0.3, 0.4, 0.5, 0.4);
        cairo_set_line_width(cr, 0.5);
        cairo_move_to(cr, cx, zero_y);
        cairo_line_to(cr, cx + cw, zero_y);
        cairo_stroke(cr);

        // Title (larger font)
        cairo_set_source_rgba(cr, 0.6, 0.7, 0.8, 0.8);
        cairo_set_font_size(cr, 14);
        cairo_move_to(cr, cx + 8, cy + 18);
        cairo_show_text(cr, title);

        // PAUSED indicator
        if (app.chart_paused) {
            cairo_set_source_rgba(cr, 1.0, 0.4, 0.3, 0.9);
            cairo_set_font_size(cr, 14);
            cairo_text_extents_t pext;
            cairo_text_extents(cr, "PAUSED", &pext);
            cairo_move_to(cr, cx + cw / 2 - pext.width / 2, cy + 18);
            cairo_show_text(cr, "PAUSED");
        }

        // Y-range labels (larger font)
        cairo_set_source_rgba(cr, 0.4, 0.5, 0.6, 0.6);
        cairo_set_font_size(cr, 12);
        char lbl[16];
        snprintf(lbl, sizeof(lbl), "+%.1f", y_range);
        cairo_move_to(cr, cx + cw - 50, cy + 16);
        cairo_show_text(cr, lbl);
        snprintf(lbl, sizeof(lbl), "-%.1f", y_range);
        cairo_move_to(cr, cx + cw - 50, cy + ch_h - 5);
        cairo_show_text(cr, lbl);

        // Draw traces (thicker lines) — clip to chart area
        cairo_save(cr);
        cairo_rectangle(cr, cx, cy, cw, ch_h);
        cairo_clip(cr);
        cairo_new_path(cr);  // clear any residual path from text/rect drawing

        auto color_it = colors.begin();
        int legend_idx = 0;
        for (auto& [hist, name] : traces) {
            auto [cr_r, cr_g, cr_b] = *color_it;
            if (color_it + 1 != colors.end()) ++color_it;

            int n = hist->count;
            if (n < 2) { legend_idx++; continue; }

            cairo_set_source_rgba(cr, cr_r, cr_g, cr_b, 0.9);
            cairo_set_line_width(cr, 2.0);

            bool started = false;
            for (int i = 0; i < n; i++) {
                double v = hist->get(i);
                double px = cx + (double)i / (WaylandApp::CHART_LEN - 1) * cw;
                double py = zero_y - (v / y_range) * (ch_h / 2.0 - 4);
                py = std::max(cy + 2.0, std::min(cy + ch_h - 2.0, py));
                if (!started) { cairo_move_to(cr, px, py); started = true; }
                else { cairo_line_to(cr, px, py); }
            }
            cairo_stroke(cr);
            cairo_new_path(cr);  // clear current point, prevent stray lines
            legend_idx++;
        }
        cairo_restore(cr);  // remove clip

        // Legend: drawn outside clip, separate loop
        auto color_it2 = colors.begin();
        int legend_idx2 = 0;
        for (auto& [hist2, name2] : traces) {
            auto [lr, lg, lb] = *color_it2;
            if (color_it2 + 1 != colors.end()) ++color_it2;

            cairo_set_font_size(cr, 14);
            double ly = cy + 38 + legend_idx2 * 20;
            // Color swatch line
            cairo_set_source_rgba(cr, lr, lg, lb, 0.9);
            cairo_set_line_width(cr, 3);
            cairo_move_to(cr, cx + 10, ly - 4);
            cairo_line_to(cr, cx + 24, ly - 4);
            cairo_stroke(cr);
            // Label + value on same line
            char legend_buf[48];
            int n2 = hist2->count;
            if (n2 > 0) {
                snprintf(legend_buf, sizeof(legend_buf), "%s: %.2f", name2, hist2->get(n2 - 1));
            } else {
                snprintf(legend_buf, sizeof(legend_buf), "%s", name2);
            }
            cairo_move_to(cr, cx + 28, ly);
            cairo_show_text(cr, legend_buf);
            legend_idx2++;
        }
    };

    // Chart layout: 3 charts stacked below attitude indicator
    double chart_x = 300, chart_w = w - 620;
    double chart_h = 130, chart_gap = 6;
    double chart_y0 = 640;

    // Chart 1: Pitch angle — target vs actual, error
    draw_chart(chart_x, chart_y0, chart_w, chart_h,
        "PITCH (°)",
        {{&app.ch_tgt_pitch, "tgt"},
         {&app.ch_flt_pitch, "act"},
         {&app.ch_pitch_err, "err"}},
        {{0.9, 0.5, 0.2},
         {0.2, 0.9, 0.4},
         {0.9, 0.3, 0.3}},
        0);  // auto-scale

    // Chart 2: Motor velocity — command, left/right
    draw_chart(chart_x, chart_y0 + chart_h + chart_gap, chart_w, chart_h,
        "MOTOR VEL (t/s)",
        {{&app.ch_pitch_out, "cmd"},
         {&app.ch_left_vel, "L"},
         {&app.ch_right_vel, "R"}},
        {{0.9, 0.9, 0.3},
         {0.3, 0.6, 0.9},
         {0.9, 0.3, 0.9}},
        0);  // auto-scale

    // Chart 3: Encoder position — L/R motor position (turns)
    draw_chart(chart_x, chart_y0 + 2 * (chart_h + chart_gap), chart_w, chart_h,
        "ENCODER POS (turns)",
        {{&app.ch_pos_l, "L pos"},
         {&app.ch_pos_r, "R pos"},
         {&app.ch_spd_est, "drift"}},
        {{0.3, 0.6, 0.9},
         {0.9, 0.3, 0.9},
         {0.9, 0.9, 0.3}},
        0);  // auto-scale

    // ── Bottom: Button hints ────────────────────────────────
    cairo_set_source_rgba(cr, 0.4, 0.5, 0.6, 0.7);
    cairo_set_font_size(cr, 15);
    cairo_move_to(cr, 50, h - 20);
    cairo_show_text(cr, "[MENU] Arm  [B] Disarm  [VIEW] E-STOP  [X] Pause Charts  [LT/RT] Speed  [D-Pad] PID  [RB] Group  [Y] Send PID");

    // ── Cleanup ─────────────────────────────────────────────
    cairo_destroy(cr);
    cairo_surface_destroy(cs);
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

static void xdg_toplevel_configure(void*, xdg_toplevel*, int32_t w, int32_t h,
                                    wl_array*) {
    // Could handle resize here
    (void)w; (void)h;
}

static void xdg_toplevel_close(void* data, xdg_toplevel*) {
    WaylandApp* app = (WaylandApp*)data;
    app->running = false;
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

static void keyboard_keymap(void*, wl_keyboard*, uint32_t, int fd, uint32_t) {
    close(fd);
}
static void keyboard_enter(void*, wl_keyboard*, uint32_t, wl_surface*, wl_array*) {}
static void keyboard_leave(void*, wl_keyboard*, uint32_t, wl_surface*) {}

static void keyboard_key(void* data, wl_keyboard*, uint32_t, uint32_t,
                          uint32_t key, uint32_t state) {
    WaylandApp* app = (WaylandApp*)data;
    if (state != WL_KEYBOARD_KEY_STATE_PRESSED) return;

    // ESC to quit
    if (key == 1) app->running = false;

    // 'q' to quit
    if (key == 16) app->running = false;

    // Space bar (scancode 57) to toggle chart pause
    if (key == 57) {
        app->chart_paused = !app->chart_paused;
        std::cout << "[Chart] " << (app->chart_paused ? "PAUSED" : "RUNNING") << "\n";
    }
}

static void keyboard_modifiers(void*, wl_keyboard*, uint32_t, uint32_t,
                                uint32_t, uint32_t, uint32_t) {}
static void keyboard_repeat(void*, wl_keyboard*, int32_t, int32_t) {}

static const wl_keyboard_listener keyboard_listener_impl = {
    .keymap = keyboard_keymap,
    .enter = keyboard_enter,
    .leave = keyboard_leave,
    .key = keyboard_key,
    .modifiers = keyboard_modifiers,
    .repeat_info = keyboard_repeat,
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
    .capabilities = seat_capabilities,
    .name = seat_name,
};

static void registry_global(void* data, wl_registry* registry,
                             uint32_t name, const char* interface,
                             uint32_t /*version*/) {
    WaylandApp* app = (WaylandApp*)data;

    if (strcmp(interface, wl_compositor_interface.name) == 0) {
        app->compositor = (wl_compositor*)wl_registry_bind(
            registry, name, &wl_compositor_interface, 4);
    }
    else if (strcmp(interface, wl_shm_interface.name) == 0) {
        app->shm = (wl_shm*)wl_registry_bind(
            registry, name, &wl_shm_interface, 1);
    }
    else if (strcmp(interface, xdg_wm_base_interface.name) == 0) {
        app->xdg_base = (xdg_wm_base*)wl_registry_bind(
            registry, name, &xdg_wm_base_interface, 2);
        xdg_wm_base_add_listener(app->xdg_base, &xdg_wm_base_listener_impl, app);
    }
    else if (strcmp(interface, wl_seat_interface.name) == 0) {
        app->seat = (wl_seat*)wl_registry_bind(
            registry, name, &wl_seat_interface, 5);
        wl_seat_add_listener(app->seat, &seat_listener_impl, app);
    }
}

static void registry_global_remove(void*, wl_registry*, uint32_t) {}

static const wl_registry_listener registry_listener_impl = {
    .global = registry_global,
    .global_remove = registry_global_remove,
};

// ═════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════

static void print_usage(const char* prog) {
    std::cout <<
        "═══════════════════════════════════════════════════════════\n"
        "  Robot Remote Controller — MSI Claw 8 AI+\n"
        "  Wayland Client + Joystick + TCP\n"
        "═══════════════════════════════════════════════════════════\n"
        "\n"
        "Usage: " << prog << " <server_ip> [options]\n"
        "\n"
        "Options:\n"
        "  --joy <device>        evdev device          [auto-detect]\n"
        "                        e.g. /dev/input/event5\n"
        "  --port <port>         TCP port              [9000]\n"
        "  --joy-map             Print gamepad events (for mapping) & exit\n"
        "  --btn-arm <code>      evdev key code for ARM     [313 = Menu/≡]\n"
        "  --btn-estop <code>    evdev key code for E-STOP  [312 = View]\n"
        "  --btn-disarm <code>   evdev key code for DISARM  [306 = B]\n"
        "  --axis-throttle <code> ABS code for throttle     [1 = ABS_Y]\n"
        "  --axis-steering <code> ABS code for steering     [0 = ABS_X]\n"
        "  --axis-lt <code>      ABS code for LT            [2 = ABS_Z]\n"
        "  --axis-rt <code>      ABS code for RT            [5 = ABS_RZ]\n"
        "  --help                Show this help\n"
        "\n"
        "Gamepad Mapping Mode:\n"
        "  " << prog << " 0 --joy-map\n"
        "  Press buttons and move sticks to see evdev codes.\n"
        "  Then use --btn-arm / --axis-throttle etc. to configure.\n"
        "\n";
}

int main(int argc, char* argv[]) {
    WaylandApp app;
    std::string joy_device = "auto";  // auto-detect evdev gamepad
    uint16_t port = TCP_PORT;

    // Parse args
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        print_usage(argv[0]);
        return (argc < 2) ? 1 : 0;
    }
    app.server_ip = argv[1];

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--joy") == 0 && i + 1 < argc) {
            joy_device = argv[++i];
        } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--joy-map") == 0) {
            app.joy_map_mode = true;
        } else if (strcmp(argv[i], "--btn-arm") == 0 && i + 1 < argc) {
            app.btn_arm = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--btn-estop") == 0 && i + 1 < argc) {
            app.btn_estop = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--btn-disarm") == 0 && i + 1 < argc) {
            app.btn_disarm = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--axis-throttle") == 0 && i + 1 < argc) {
            app.axis_throttle = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--axis-steering") == 0 && i + 1 < argc) {
            app.axis_steering = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--axis-lt") == 0 && i + 1 < argc) {
            app.axis_speed_lt = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--axis-rt") == 0 && i + 1 < argc) {
            app.axis_speed_rt = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--no-dinput-switch") == 0) {
            app.auto_switch_dinput = false;
        }
    }

    // ── MSI Claw: Switch to DInput mode ─────────────────────
    //  MSI Claw 8 AI+ boots in XInput mode (VID:PID 0db0:1901).
    //  Buttons like Menu/View don't produce evdev events in XInput.
    //  Sending a HID output report switches it to DInput (0db0:1902),
    //  which causes USB re-enumeration with correct button mapping.
    //
    //  IMPORTANT: Even if the device persists DInput mode (1902) across
    //  reboots, the mode switch command must be re-sent after each boot
    //  to enable full button mapping (Menu/View keys).
    //
    //  This block runs BEFORE --joy-map so that mapping mode also
    //  sees the correct button events.

    // Helper: scan /sys/class/hidraw for MSI Claw devices matching a string in uevent
    auto find_msi_claw_hidraw = [](const std::string& match_str) -> std::vector<std::string> {
        std::vector<std::string> results;
        DIR* hiddir = opendir("/sys/class/hidraw");
        if (!hiddir) return results;
        struct dirent* entry;
        while ((entry = readdir(hiddir)) != nullptr) {
            std::string name = entry->d_name;
            if (name.substr(0, 6) != "hidraw") continue;
            std::string uevent_path = "/sys/class/hidraw/" + name + "/device/uevent";
            FILE* f = fopen(uevent_path.c_str(), "r");
            if (!f) continue;
            char buf[512];
            bool has_vid = false, has_match = false;
            while (fgets(buf, sizeof(buf), f)) {
                if (strstr(buf, "0DB0")) has_vid = true;
                if (strstr(buf, match_str.c_str())) has_match = true;
            }
            fclose(f);
            if (has_vid && has_match) results.push_back("/dev/" + name);
        }
        closedir(hiddir);
        return results;
    };

    // Helper: send HID mode switch command to MSI Claw hidraw devices
    auto send_mode_switch = [](const std::vector<std::string>& devpaths, uint8_t mode_byte) -> bool {
        uint8_t cmd[] = {0x0F, 0x00, 0x00, 0x3C, 0x24, mode_byte, 0x00, 0x00};
        bool sent = false;
        for (const auto& devpath : devpaths) {
            int fd = ::open(devpath.c_str(), O_RDWR);
            if (fd < 0) {
                std::cerr << "[Main] Cannot open " << devpath << ": " << strerror(errno) << "\n";
                continue;
            }
            ssize_t n = ::write(fd, cmd, sizeof(cmd));
            ::close(fd);
            if (n == (ssize_t)sizeof(cmd)) {
                std::cout << "[Main] Mode switch (0x" << std::hex << (int)mode_byte
                          << std::dec << ") sent to " << devpath << "\n";
                sent = true;
            }
        }
        return sent;
    };

    if (app.auto_switch_dinput) {
        auto all_devs = find_msi_claw_hidraw("0DB0");
        bool is_xinput = !find_msi_claw_hidraw("1901").empty();
        bool is_dinput = !find_msi_claw_hidraw("1902").empty();

        if (all_devs.empty()) {
            std::cout << "[Main] No MSI Claw detected (VID 0db0)\n";
        } else {
            if (is_xinput) {
                std::cout << "[Main] MSI Claw in XInput mode (0db0:1901)\n";
            } else if (is_dinput) {
                std::cout << "[Main] MSI Claw in DInput mode (0db0:1902), re-sending switch command...\n";
            }
            std::cout << "[Main] Sending DInput mode switch to " << all_devs.size() << " hidraw node(s)...\n";
            if (send_mode_switch(all_devs, 0x02)) {
                std::cout << "[Main] Waiting 3s for USB re-enumeration...\n";
                std::this_thread::sleep_for(std::chrono::seconds(3));
            } else {
                std::cerr << "[Main] WARNING: Failed to send mode switch to any device\n"
                          << "       Try: sudo usermod -aG input $USER\n";
            }
        }
    }

    // ── Gamepad Mapping Mode ─────────────────────────────────
    if (app.joy_map_mode) {
        bool opened = false;
        if (joy_device == "auto") {
            opened = app.gamepad.auto_detect();
        } else {
            opened = app.gamepad.open(joy_device);
        }
        if (!opened) {
            std::cerr << "Cannot open gamepad for mapping.\n"
                      << "Try: sudo usermod -aG input $USER\n"
                      << "Or specify: --joy /dev/input/eventN\n";
            return 1;
        }
        std::cout << "\n"
            "╔═══════════════════════════════════════════════╗\n"
            "║  Gamepad Mapping Mode (evdev)                  ║\n"
            "║                                                ║\n"
            "║  Press buttons → shows EV_KEY code + name      ║\n"
            "║  Move sticks/triggers → shows ABS code + value ║\n"
            "║                                                ║\n"
            "║  Use codes with --btn-arm, --axis-throttle etc.║\n"
            "║  Press Ctrl+C to exit.                         ║\n"
            "╚═══════════════════════════════════════════════╝\n\n"
            "  Default mapping:\n"
            "    --btn-arm " << BTN_START << " (BTN_START)\n"
            "    --btn-estop " << BTN_SELECT << " (BTN_SELECT)\n"
            "    --btn-disarm " << BTN_EAST << " (BTN_EAST/B)\n"
            "    --axis-throttle " << ABS_Y << " (ABS_Y)\n"
            "    --axis-steering " << ABS_X << " (ABS_X)\n"
            "    --axis-lt " << ABS_Z << " (ABS_Z)\n"
            "    --axis-rt " << ABS_RZ << " (ABS_RZ)\n\n"
            "  Waiting for input events...\n\n";

        // evdev ABS codes to scan for display
        static const int abs_codes[] = {
            ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ,
            ABS_HAT0X, ABS_HAT0Y
        };
        static const char* abs_names[] = {
            "ABS_X/LStick_X", "ABS_Y/LStick_Y", "ABS_Z/LT",
            "ABS_RX/RStick_X", "ABS_RY/RStick_Y", "ABS_RZ/RT",
            "ABS_HAT0X/DPad_X", "ABS_HAT0Y/DPad_Y"
        };

        while (true) {
            app.gamepad.poll();
            // Button presses are already logged by GamepadReader::poll()
            // Show axes with non-zero values
            for (int i = 0; i < 8; i++) {
                double v = app.gamepad.axis(abs_codes[i]);
                if (std::abs(v) > 0.1) {
                    printf("[Map] Axis %3d %-20s = %+.3f\n",
                           abs_codes[i], abs_names[i], v);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }


    // ── Open Gamepad ─────────────────────────────────────────
    if (joy_device == "auto") {
        if (!app.gamepad.auto_detect()) {
            std::cerr << "[Main] WARNING: No gamepad found, keyboard only\n";
        }
    } else {
        if (!app.gamepad.open(joy_device)) {
            std::cerr << "[Main] WARNING: Cannot open " << joy_device << ", keyboard only\n";
        }
    }
    if (app.gamepad.is_open()) {
        std::cout << "[Main] Gamepad: " << app.gamepad.name() << "\n"
                  << "[Main] Button mapping (evdev codes):\n"
                  << "  ARM    = " << app.btn_arm << " (Menu/≡)\n"
                  << "  DISARM = " << app.btn_disarm << " (B)\n"
                  << "  E-STOP = " << app.btn_estop << " (View)\n"
                  << "  Throttle = ABS_" << app.axis_throttle << " (LStick Y)\n"
                  << "  Steering = ABS_" << app.axis_steering << " (LStick X)\n"
                  << "  LT = ABS_" << app.axis_speed_lt
                  << "  RT = ABS_" << app.axis_speed_rt << " (speed limit)\n"
                  << "  Use --joy-map to detect correct codes.\n";
    }

    // ── Connect to Wayland ──────────────────────────────────
    app.display = wl_display_connect(nullptr);
    if (!app.display) {
        std::cerr << "[Main] Cannot connect to Wayland display\n";
        return 1;
    }

    app.registry = wl_display_get_registry(app.display);
    wl_registry_add_listener(app.registry, &registry_listener_impl, &app);
    wl_display_roundtrip(app.display);

    if (!app.compositor || !app.shm || !app.xdg_base) {
        std::cerr << "[Main] Missing required Wayland interfaces\n";
        return 1;
    }

    // Create surface
    app.surface = wl_compositor_create_surface(app.compositor);
    app.xdg_surf = xdg_wm_base_get_xdg_surface(app.xdg_base, app.surface);
    xdg_surface_add_listener(app.xdg_surf, &xdg_surface_listener_impl, &app);

    app.toplevel = xdg_surface_get_toplevel(app.xdg_surf);
    xdg_toplevel_add_listener(app.toplevel, &xdg_toplevel_listener_impl, &app);
    xdg_toplevel_set_title(app.toplevel, WINDOW_TITLE);
    xdg_toplevel_set_app_id(app.toplevel, "robot-controller");
    // Request fullscreen for MSI Claw
    xdg_toplevel_set_fullscreen(app.toplevel, nullptr);

    wl_surface_commit(app.surface);
    wl_display_roundtrip(app.display);

    // Create buffer
    app.buffer = create_buffer(app);
    if (!app.buffer) {
        std::cerr << "[Main] Failed to create Wayland buffer\n";
        return 1;
    }

    // ── TCP Connection Thread ───────────────────────────────
    std::atomic<bool> tcp_running{true};
    std::thread tcp_thread([&]() {
        int reconnect_delay_ms = 1000;  // start at 1s, max 5s
        bool was_connected = false;

        while (tcp_running.load()) {
            if (!app.tcp.is_connected()) {
                if (was_connected) {
                    std::cout << "[TCP] Disconnected, reconnecting...\n";
                    was_connected = false;
                    reconnect_delay_ms = 1000;  // reset backoff
                    // Reset telemetry state
                    {
                        std::lock_guard<std::mutex> lock(app.telem_mtx);
                        app.telem.state = "DISCONNECTED";
                    }
                    app.armed = false;
                }
                if (app.tcp.connect(app.server_ip, port)) {
                    std::cout << "[TCP] Connected to " << app.server_ip << ":" << port << "\n";
                    was_connected = true;
                    reconnect_delay_ms = 1000;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
                    if (reconnect_delay_ms < 5000) reconnect_delay_ms += 1000;
                    continue;
                }
            }

            // Receive telemetry
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
                    app.telem.state    = json_parse_string(line, "state");
                    app.telem.last_update = std::chrono::steady_clock::now();
                    // Sync armed flag from robot state (robot is the source of truth)
                    app.armed = (app.telem.state == "BALANCING" || app.telem.state == "ARMED" || app.telem.state == "CALIBRATING");
                    if (!app.chart_paused) {
                        app.push_chart_data(app.telem);
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    // ── Command Sending Thread ──────────────────────────────
    std::thread cmd_thread([&]() {
        bool prev_arm = false, prev_estop = false, prev_disarm = false;

        while (tcp_running.load()) {
            app.gamepad.poll();

            if (app.tcp.is_connected()) {
                // Read joystick
                double throttle = -app.gamepad.axis(app.axis_throttle);
                double steering = app.gamepad.axis(app.axis_steering);
                double lt = app.gamepad.axis(app.axis_speed_lt);
                double rt = app.gamepad.axis(app.axis_speed_rt);
                double speed_limit = 1.0 - (lt + rt) * 0.5;
                if (speed_limit < 0.1) speed_limit = 0.1;

                // Apply deadband
                if (std::abs(throttle) < 0.05) throttle = 0;
                if (std::abs(steering) < 0.05) steering = 0;

                // Send control command
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "{\"type\":\"control\",\"throttle\":%.4f,"
                    "\"steering\":%.4f,\"speed_limit\":%.2f}",
                    throttle, steering, speed_limit);
                app.tcp.send_line(buf);

                // Button state changes (edge-triggered)
                bool arm_btn = app.gamepad.button(app.btn_arm);
                bool estop_btn = app.gamepad.button(app.btn_estop);
                bool disarm_btn = app.gamepad.button(app.btn_disarm);

                if (arm_btn && !prev_arm) {
                    app.tcp.send_line("{\"type\":\"arm\"}");
                    app.armed = true;
                    std::cout << "[Cmd] ARM\n";
                }
                if (disarm_btn && !prev_disarm) {
                    app.tcp.send_line("{\"type\":\"disarm\"}");
                    app.armed = false;
                    std::cout << "[Cmd] DISARM\n";
                }
                if (estop_btn && !prev_estop) {
                    app.tcp.send_line("{\"type\":\"estop\"}");
                    app.armed = false;
                    std::cout << "[Cmd] !! EMERGENCY STOP !!\n";
                }

                prev_arm = arm_btn;
                prev_estop = estop_btn;
                prev_disarm = disarm_btn;

                // ── X (304): Toggle chart pause ──────────────
                static bool prev_x_pause = false;
                bool x_btn = app.gamepad.button(304);  // X (BTN_SOUTH)
                if (x_btn && !prev_x_pause) {
                    app.chart_paused = !app.chart_paused;
                    std::cout << "[Chart] " << (app.chart_paused ? "PAUSED" : "RUNNING") << "\n";
                }
                prev_x_pause = x_btn;

                // ── PID Tuning Controls ─────────────────────
                // D-Pad up/down: select parameter
                // D-Pad left/right: adjust value
                // RB (309): cycle PID group
                // Y (307): send PID to server
                static int prev_dpad_y = 0, prev_dpad_x = 0;
                static bool prev_rb = false, prev_y = false;

                int dpad_y = (int)std::round(app.gamepad.axis(ABS_HAT0Y));
                int dpad_x = (int)std::round(app.gamepad.axis(ABS_HAT0X));
                bool rb_btn = app.gamepad.button(309);   // RB
                bool y_btn  = app.gamepad.button(307);   // Y

                // D-Pad up/down — select param (edge-triggered)
                if (dpad_y != prev_dpad_y) {
                    if (dpad_y < 0) {  // D-Pad up
                        app.pid_sel--;
                        if (app.pid_sel < 0) app.pid_sel = app.current_pid_count() - 1;
                    } else if (dpad_y > 0) {  // D-Pad down
                        app.pid_sel++;
                        if (app.pid_sel >= app.current_pid_count()) app.pid_sel = 0;
                    }
                    prev_dpad_y = dpad_y;
                }

                // D-Pad left/right — adjust value (edge-triggered)
                if (dpad_x != prev_dpad_x) {
                    auto* params = app.current_pid_params();
                    auto& p = params[app.pid_sel];
                    if (dpad_x > 0) {  // right = increase
                        p.value += p.step;
                        if (p.value > p.max) p.value = p.max;
                        app.pid_dirty = true;
                    } else if (dpad_x < 0) {  // left = decrease
                        p.value -= p.step;
                        if (p.value < p.min) p.value = p.min;
                        app.pid_dirty = true;
                    }
                    prev_dpad_x = dpad_x;
                }

                // RB — cycle PID group (edge-triggered)
                if (rb_btn && !prev_rb) {
                    app.pid_group = (app.pid_group + 1) % WaylandApp::PID_GROUP_COUNT;
                    app.pid_sel = 0;
                    std::cout << "[PID] Group: " << app.pid_group_names[app.pid_group] << "\n";
                }
                prev_rb = rb_btn;

                // Y — send PID values to server (edge-triggered)
                if (y_btn && !prev_y && app.pid_dirty) {
                    char pid_buf[512];
                    snprintf(pid_buf, sizeof(pid_buf),
                        "{\"type\":\"set_pid\","
                        "\"pitch_kp\":%.4f,\"pitch_ki\":%.4f,\"pitch_kd\":%.4f,"
                        "\"yaw_kp\":%.4f,"
                        "\"speed_kp\":%.4f,"
                        "\"pitch_offset\":%.4f,\"max_velocity\":%.4f}",
                        app.pid_pitch[0].value, app.pid_pitch[1].value, app.pid_pitch[2].value,
                        app.pid_yaw[0].value,
                        app.pid_speed[0].value,
                        app.pid_other[0].value, app.pid_other[1].value);
                    app.tcp.send_line(pid_buf);
                    app.pid_dirty = false;
                    std::cout << "[PID] Sent: pitch(" << app.pid_pitch[0].value
                              << "/" << app.pid_pitch[1].value
                              << "/" << app.pid_pitch[2].value
                              << ") yaw_kp=" << app.pid_yaw[0].value
                              << " speed_kp=" << app.pid_speed[0].value << "\n";
                }
                prev_y = y_btn;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RATE_MS));
        }
    });

    // ── Main Render Loop ────────────────────────────────────
    std::cout << "[Main] Running... Press ESC or 'q' to quit\n";

    while (app.running) {
        // Dispatch Wayland events (non-blocking)
        wl_display_dispatch_pending(app.display);

        if (wl_display_flush(app.display) < 0 &&
            errno != EAGAIN) break;

        // Poll Wayland fd
        struct pollfd fds[1];
        fds[0].fd = wl_display_get_fd(app.display);
        fds[0].events = POLLIN;
        int ret = poll(fds, 1, RENDER_RATE_MS);

        if (ret > 0) {
            wl_display_dispatch(app.display);
        }

        // Render
        if (app.configured) {
            render_hud(app);
            wl_surface_attach(app.surface, app.buffer, 0, 0);
            wl_surface_damage_buffer(app.surface, 0, 0, app.width, app.height);
            wl_surface_commit(app.surface);
        }
    }

    // ── Cleanup ─────────────────────────────────────────────
    tcp_running.store(false);
    if (tcp_thread.joinable()) tcp_thread.join();
    if (cmd_thread.joinable()) cmd_thread.join();

    app.tcp.disconnect();
    app.gamepad.close();

    if (app.buffer) wl_buffer_destroy(app.buffer);
    if (app.shm_data) munmap(app.shm_data, app.shm_size);
    if (app.shm_fd >= 0) close(app.shm_fd);
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
