#pragma once
// ═══════════════════════════════════════════════════════════════
//  Controller Types & Constants
// ═══════════════════════════════════════════════════════════════

#include <string>
#include <chrono>
#include "mini_json.h"

// ── Configuration ────────────────────────────────────────────
static const int    WINDOW_WIDTH  = 1920;
static const int    WINDOW_HEIGHT = 1200;
[[maybe_unused]]  static const char*  WINDOW_TITLE  = "Robot Controller";
static const int    TCP_PORT      = 9000;
static const int    SEND_RATE_MS  = 20;   // 50 Hz command rate
static const int    RENDER_RATE_MS = 16;  // ~60 FPS


// ── Keyboard scan codes (evdev) ──────────────────────────────
#define D_PAD_LEFT  47
#define D_PAD_RIGHT 38
#define D_PAD_UP    49
#define D_PAD_DOWN  46

#define RB_BUTTON   20
#define LB_BUTTON   17

#define MENU_BUTTON 315

#define KEY_A       30
#define KEY_X       45
#define KEY_Y       21
#define KEY_B       48

// ── Telemetry ────────────────────────────────────────────────
struct Telemetry {
    double pitch      = 0;
    double roll       = 0;
    double yaw        = 0;
    double speed      = 0;
    double vbus       = 0;
    double left_vel   = 0;   // measured left wheel velocity (feedback)
    double right_vel  = 0;   // measured right wheel velocity (feedback)
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
    double ang_vel    = 0;
    double ang_acc    = 0;
    double pos_l      = 0;
    double pos_r      = 0;
    // Position loop
    double pos_disp   = 0;
    double pos_target = 0;
    double pos_out    = 0;
    // Motor command velocities (PID output, forward=positive)
    double cmd_l      = 0;
    double cmd_r      = 0;
    // USB connection status
    bool   odrive_usb = false;
    bool   imu_usb    = false;

    // Status message from robot
    std::string msg;

    std::string state = "DISCONNECTED";
    std::chrono::steady_clock::time_point last_update;
};

// ── PID Tuning Parameter ─────────────────────────────────────
struct PidParam {
    const char* key;
    const char* label;
    double value;
    double step;
    double min;
    double max;
    int group = 0;
};

// ── JSON Helpers ─────────────────────────────────────────────
static inline double json_parse_double(const std::string& json,
                                        const std::string& key,
                                        double def = 0.0) {
    return minijson::get_number_or(json, key, def);
}

static inline std::string json_parse_string(const std::string& json,
                                             const std::string& key) {
    return minijson::get_string_or(json, key);
}

static inline bool json_parse_bool(const std::string& json,
                                    const std::string& key,
                                    bool def = false) {
    bool value = def;
    (void)minijson::get_bool(json, key, value);
    return value;
}
