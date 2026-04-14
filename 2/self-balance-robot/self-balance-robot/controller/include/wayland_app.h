#pragma once
// ═══════════════════════════════════════════════════════════════
//  WaylandApp — Application State
// ═══════════════════════════════════════════════════════════════

#include "controller_types.h"
#include "gamepad_reader.h"
#include "tcp_client.h"

#include <wayland-client.h>
#include "xdg-shell-protocol.h"

#include <mutex>
#include <atomic>
#include <string>

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
    struct ShmBuffer {
        wl_buffer* buffer = nullptr;
        void* shm_data = nullptr;
        int shm_fd = -1;
        size_t shm_size = 0;
        bool busy = false;
    };
    static constexpr int BUFFER_COUNT = 2;
    ShmBuffer buffers[BUFFER_COUNT];
    int front_buffer = 0;

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
    bool chart_paused = false;

    // Telemetry
    std::mutex telem_mtx;
    Telemetry telem;

    // Gamepad mappings
    int axis_steering  = ABS_X;
    int axis_throttle  = ABS_Y;
    int axis_speed_lt  = ABS_Z;
    int axis_speed_rt  = ABS_RZ;
    int btn_arm        = 313;
    int btn_estop      = 312;
    int btn_disarm     = 306;

    bool joy_map_mode  = false;
    bool auto_switch_dinput = true;

    double throttle = 0.0;
    double steering = 0.0;

    // ── PID Tuning ──────────────────────────────────────────
    static constexpr int PID_GROUP_COUNT = 6;
    const char* pid_group_names[PID_GROUP_COUNT] = {
        "PITCH PID", "YAW PID", "SPEED PID", "POSITION PID", "LIMITS", "OTHER"
    };
    int pid_group = 0;
    int pid_sel   = 0;

    // Pitch PID
    PidParam pid_pitch[4] = {
        {"Kp",      0.30, 0.01, 0.0, 5.0},
        {"Ki",      0.005,0.005,0.0, 1.0},
        {"Kd",      0.025,0.005,0.0, 1.0},
        {"D_alpha", 0.2,  0.05, 0.01, 1.0},
    };
    // Yaw PID
    PidParam pid_yaw[3] = {
        {"Kp",  1.0,  0.1,  0.0, 20.0},
        {"Ki",  0.0,  0.01, 0.0,  5.0},
        {"Kd",  0.0,  0.01, 0.0,  5.0},
    };
    // Speed PID (preserved for future use)
    PidParam pid_speed[3] = {
        {"Kp",   0.6, 0.01, 0.0,  5.0},
        {"Ki",   0.0, 0.005,0.0,  2.0},
        {"Kd",   0.0, 0.005,0.0,  2.0},
    };
    // Position PID
    PidParam pid_pos[3] = {
        {"Kp",   0.5, 0.05, 0.0,  5.0},
        {"Ki",   0.1, 0.01, 0.0,  2.0},
        {"Kd",   0.2, 0.05, 0.0,  5.0},
    };
    // Limits (asymmetric clamps)
    PidParam pid_limits[4] = {
        {"pos_out_fwd",     5.0,  0.5,    0.5,  20.0},
        {"pos_out_bwd",    -5.0,  0.5,  -20.0,  -0.5},
        {"tgt_pitch_fwd",  15.0,  1.0,    1.0,  45.0},
        {"tgt_pitch_bwd", -15.0,  1.0,  -45.0,  -1.0},
    };
    // Other params
    PidParam pid_other[2] = {
        {"pitch_offset",    0.0,  0.1,  -10.0,  10.0},
        {"max_velocity",    2.0,  0.5,    0.5,  20.0},
    };

    PidParam* current_pid_params() {
        switch (pid_group) {
            case 0: return pid_pitch;
            case 1: return pid_yaw;
            case 2: return pid_speed;
            case 3: return pid_pos;
            case 4: return pid_limits;
            case 5: return pid_other;
            default: return pid_pitch;
        }
    }
    int current_pid_count() const {
        switch (pid_group) {
            case 0: return 4;  // PITCH: Kp, Ki, Kd, D_alpha
            case 1: case 2: case 3: return 3;
            case 4: return 4;  // LIMITS
            case 5: return 2;  // OTHER
            default: return 3;
        }
    }
    bool pid_dirty = false;

    // ── Chart Cursor ────────────────────────────────────────
    int cursor_pos = -1;  // -1 = no cursor, 0..CHART_LEN-1 = position

    // ── Config Persistence ──────────────────────────────────
    static constexpr const char* CONFIG_FILE = "params.json";
    std::string config_dir;  // set from main(), e.g. ~/self-balance-robot/controller

    /// Save all tuning params to JSON file
    bool save_config() const;

    /// Load tuning params from JSON file (returns false if file not found)
    bool load_config();

    // ── Chart History ───────────────────────────────────────
    static constexpr int CHART_LEN = 200;
    struct ChartHistory {
        double data[200] = {};
        int head = 0;
        int count = 0;
        void push(double v) {
            data[head] = v;
            head = (head + 1) % 200;
            if (count < 200) count++;
        }
        double get(int i) const {
            if (i < 0 || i >= count) return 0;
            return data[(head - count + i + 200) % 200];
        }
        void clear() { head = 0; count = 0; }
    };

    ChartHistory ch_pitch_out;
    ChartHistory ch_left_vel;
    ChartHistory ch_right_vel;
    ChartHistory ch_pitch_err;
    ChartHistory ch_tgt_pitch;
    ChartHistory ch_flt_pitch;
    ChartHistory ch_tgt_speed;
    ChartHistory ch_spd_est;
    ChartHistory ch_ang_vel;
    ChartHistory ch_ang_acc;
    ChartHistory ch_pos_l;
    ChartHistory ch_pos_r;
    ChartHistory ch_cmd_l;
    ChartHistory ch_cmd_r;

    std::string prev_state;

    void push_chart_data(const Telemetry& t) {
        if (t.state != prev_state) {
            if (t.state == "BALANCING") clear_all_charts();
            prev_state = t.state;
        }
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
        ch_cmd_l.push(t.cmd_l);
        ch_cmd_r.push(t.cmd_r);
    }

    void clear_all_charts() {
        ch_pitch_out.clear(); ch_left_vel.clear(); ch_right_vel.clear();
        ch_pitch_err.clear(); ch_tgt_pitch.clear(); ch_flt_pitch.clear();
        ch_tgt_speed.clear(); ch_spd_est.clear();
        ch_ang_vel.clear(); ch_ang_acc.clear();
        ch_pos_l.clear(); ch_pos_r.clear();
        ch_cmd_l.clear(); ch_cmd_r.clear();
    }
};
