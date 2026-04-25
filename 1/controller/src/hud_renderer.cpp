// ═══════════════════════════════════════════════════════════════
//  HUD Renderer — Cairo-based HUD drawing
//  Renders: title bar, USB status, artificial horizon, joystick,
//           PID tuning panel, telemetry panel, position panel,
//           real-time charts, button hints
// ═══════════════════════════════════════════════════════════════

#include "hud_renderer.h"
#include "wayland_app.h"

#include <cairo/cairo.h>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <utility>
#include <tuple>



void render_hud(WaylandApp& app, void* shm_data) {

    int w = app.width;
    int h = app.height;
    int stride = w * 4;

    cairo_surface_t* cs = cairo_image_surface_create_for_data(
        (unsigned char*)shm_data,
        CAIRO_FORMAT_ARGB32, w, h, stride);

    cairo_t* cr = cairo_create(cs);

    // ── Background ──────────────────────────────────────────
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
    double lt = 0.2;
    double rt = 0.1;
    double speed_limit = 1.0 - (lt + rt) * 0.5;
    if (speed_limit < 0.1) speed_limit = 0.1;

    // ── Title Bar ───────────────────────────────────────────
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 28);

    bool connected = app.tcp.is_connected();
    bool data_fresh = false;
    {
        auto age = std::chrono::steady_clock::now() - telem.last_update;
        data_fresh = std::chrono::duration_cast<std::chrono::seconds>(age).count() < 2;
    }

    // Status dot
    if (connected && data_fresh) {
        cairo_set_source_rgb(cr, 0.2, 0.9, 0.4);
    } else if (connected) {
        cairo_set_source_rgb(cr, 0.9, 0.7, 0.1);
    } else {
        cairo_set_source_rgb(cr, 0.9, 0.2, 0.2);
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

    // ── USB Connection Status Indicators ─────────────────────
    {
        double usb_x = 530;
        double usb_y = 44;
        cairo_set_font_size(cr, 18);

        // ODrive indicator
        if (telem.odrive_usb) {
            cairo_set_source_rgb(cr, 0.2, 0.9, 0.4);
        } else {
            cairo_set_source_rgb(cr, 0.9, 0.2, 0.2);
        }
        cairo_arc(cr, usb_x, usb_y - 6, 5, 0, 2 * M_PI);
        cairo_fill(cr);
        cairo_set_source_rgb(cr, 0.6, 0.7, 0.8);
        cairo_move_to(cr, usb_x + 10, usb_y);
        cairo_show_text(cr, "ODrive");

        // IMU indicator
        usb_x += 100;
        if (telem.imu_usb) {
            cairo_set_source_rgb(cr, 0.2, 0.9, 0.4);
        } else {
            cairo_set_source_rgb(cr, 0.9, 0.2, 0.2);
        }
        cairo_arc(cr, usb_x, usb_y - 6, 5, 0, 2 * M_PI);
        cairo_fill(cr);
        cairo_set_source_rgb(cr, 0.6, 0.7, 0.8);
        cairo_move_to(cr, usb_x + 10, usb_y);
        cairo_show_text(cr, "IMU");

        // Status message bar
        if (!telem.msg.empty()) {
            cairo_set_source_rgb(cr, 1.0, 0.8, 0.2);
            cairo_set_font_size(cr, 18);
            cairo_move_to(cr, usb_x + 80, usb_y);
            cairo_show_text(cr, telem.msg.c_str());
        }
    }

    // Server IP
    cairo_set_source_rgb(cr, 0.5, 0.6, 0.7);
    cairo_set_font_size(cr, 24);
    char ip_buf[128];
    snprintf(ip_buf, sizeof(ip_buf), "Server: %s:%d",
             app.server_ip.c_str(), TCP_PORT);
    cairo_move_to(cr, w - 350, 38);
    cairo_show_text(cr, ip_buf);

    // ── Artificial Horizon ──────────────────────────────────
    double horizon_r = 110;
    double cx = w - 300 - horizon_r - 40;  // left of right panel
    double cy = 190;

    cairo_set_source_rgba(cr, 0.2, 0.4, 0.6, 0.3);
    cairo_set_line_width(cr, 2);
    cairo_move_to(cr, cx + horizon_r + 8, cy);
    cairo_arc(cr, cx, cy, horizon_r + 5, 0, 2 * M_PI);
    cairo_stroke(cr);

    cairo_save(cr);
    cairo_arc(cr, cx, cy, horizon_r, 0, 2 * M_PI);
    cairo_clip(cr);

    double pitch_px = telem.pitch * 3.0;
    double roll_rad = telem.roll * M_PI / 180.0;

    cairo_save(cr);
    cairo_translate(cr, cx, cy);
    cairo_rotate(cr, roll_rad);

    cairo_set_source_rgb(cr, 0.1, 0.15, 0.35);
    cairo_rectangle(cr, -horizon_r - 50, -horizon_r - 200 - pitch_px,
                    2 * (horizon_r + 50), horizon_r + 200);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 0.15, 0.10, 0.05);
    cairo_rectangle(cr, -horizon_r - 50, -pitch_px,
                    2 * (horizon_r + 50), horizon_r + 200);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 0.9, 0.9, 0.9);
    cairo_set_line_width(cr, 2);
    cairo_move_to(cr, -horizon_r, -pitch_px);
    cairo_line_to(cr, horizon_r, -pitch_px);
    cairo_stroke(cr);

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
    cairo_restore(cr);

    // Center reticle
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
    cairo_move_to(cr, cx, cy - 6);
    cairo_line_to(cr, cx, cy + 6);
    cairo_stroke(cr);

    // ── Joystick Visualizer ─────────────────────────────────
    double lp_x = 130;
    double lp_y = 170;
    double stick_r = 90;

    cairo_set_source_rgba(cr, 0.1, 0.15, 0.2, 0.8);
    cairo_arc(cr, lp_x, lp_y, stick_r + 10, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_set_source_rgba(cr, 0.3, 0.5, 0.7, 0.3);
    cairo_set_line_width(cr, 1.5);
    cairo_arc(cr, lp_x, lp_y, stick_r, 0, 2 * M_PI);
    cairo_stroke(cr);

    cairo_set_source_rgba(cr, 0.3, 0.4, 0.5, 0.5);
    cairo_move_to(cr, lp_x - stick_r, lp_y);
    cairo_line_to(cr, lp_x + stick_r, lp_y);
    cairo_move_to(cr, lp_x, lp_y - stick_r);
    cairo_line_to(cr, lp_x, lp_y + stick_r);
    cairo_stroke(cr);

    double sx = lp_x + app.steering * stick_r;
    double sy = lp_y - app.throttle * stick_r;
    cairo_set_source_rgb(cr, 0.0, 0.9, 0.5);
    cairo_arc(cr, sx, sy, 12, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 0.6, 0.7, 0.8);
    cairo_set_font_size(cr, 20);
    cairo_move_to(cr, lp_x - 50, lp_y + stick_r + 35);
    cairo_show_text(cr, "LEFT STICK");

    char stick_val[64];
    snprintf(stick_val, sizeof(stick_val), "T:%.2f S:%.2f", app.throttle, app.steering);
    cairo_move_to(cr, lp_x - 70, lp_y + stick_r + 60);
    cairo_set_source_rgb(cr, 0.7, 0.7, 0.0);
    cairo_show_text(cr, stick_val);

    // ── PID Tuning Panel (right of joystick) ──────────────────
    {
        double pp_x = 250;
        double pp_y = 70;
        double pp_w = 560;
        int n_params = app.current_pid_count();
        double pp_h = 42 + n_params * 40 + 30;

        cairo_set_source_rgba(cr, 0.05, 0.08, 0.12, 0.85);
        cairo_rectangle(cr, pp_x, pp_y, pp_w, pp_h);
        cairo_fill(cr);

        if (app.pid_dirty) {
            cairo_set_source_rgba(cr, 0.9, 0.6, 0.1, 0.8);
        } else {
            cairo_set_source_rgba(cr, 0.2, 0.4, 0.6, 0.5);
        }
        cairo_set_line_width(cr, 1);
        cairo_rectangle(cr, pp_x, pp_y, pp_w, pp_h);
        cairo_stroke(cr);

        cairo_set_source_rgb(cr, 0.7, 0.85, 1.0);
        cairo_set_font_size(cr, 22);
        cairo_move_to(cr, pp_x + 10, pp_y + 22);
        std::string group_title = std::string("◀ ") + app.current_group_name() + " ▶";
        cairo_show_text(cr, group_title.c_str());

        if (app.pid_dirty) {
            cairo_set_source_rgb(cr, 0.9, 0.6, 0.1);
            cairo_set_font_size(cr, 18);
            cairo_move_to(cr, pp_x + pp_w - 110, pp_y + 22);
            cairo_show_text(cr, "[Y] Send");
        }

        auto params = app.current_pid_params();
        for (int i = 0; i < n_params; i++) {
            double row_y = pp_y + 42 + i * 40;
            bool selected = (i == app.pid_sel);

            if (selected) {
                cairo_set_source_rgba(cr, 0.15, 0.25, 0.4, 0.8);
                cairo_rectangle(cr, pp_x + 5, row_y - 4, pp_w - 10, 36);
                cairo_fill(cr);
                cairo_set_source_rgb(cr, 0.0, 0.9, 0.5);
                cairo_set_font_size(cr, 20);
                cairo_move_to(cr, pp_x + 10, row_y + 21);
                cairo_show_text(cr, "▶");
            }

            cairo_set_source_rgba(cr, 0.5, 0.6, 0.7, selected ? 1.0 : 0.7);
            cairo_set_font_size(cr, 19);
            cairo_move_to(cr, pp_x + 28, row_y + 20);
            cairo_show_text(cr, params[i]->label);

            if (selected) {
                cairo_set_source_rgb(cr, 0.0, 0.95, 0.5);
            } else {
                cairo_set_source_rgb(cr, 0.8, 0.85, 0.9);
            }
            cairo_set_font_size(cr, 25);
            char val_buf[32];
            if (params[i]->step >= 1.0) {
                snprintf(val_buf, sizeof(val_buf), "%.1f", params[i]->value);
            } else if (params[i]->step >= 0.01) {
                snprintf(val_buf, sizeof(val_buf), "%.2f", params[i]->value);
            } else {
                snprintf(val_buf, sizeof(val_buf), "%.3f", params[i]->value);
            }
            cairo_move_to(cr, pp_x + 360, row_y + 23);
            cairo_show_text(cr, val_buf);

            if (selected) {
                cairo_set_source_rgba(cr, 0.4, 0.5, 0.6, 0.8);
                cairo_set_font_size(cr, 19);
                char step_buf[32];
                snprintf(step_buf, sizeof(step_buf), "±%.3g", params[i]->step);
                cairo_move_to(cr, pp_x + 460, row_y + 21);
                cairo_show_text(cr, step_buf);
            }
        }
    }

    // ── Right Panel: Telemetry ──────────────────────────────
    double rp_x = w - 300;
    double rp_y = 60;

    cairo_set_source_rgba(cr, 0.05, 0.08, 0.12, 0.85);
    cairo_rectangle(cr, rp_x - 10, rp_y - 10, 280, 1080);
    cairo_fill(cr);

    cairo_set_source_rgba(cr, 0.2, 0.4, 0.6, 0.5);
    cairo_set_line_width(cr, 1);
    cairo_rectangle(cr, rp_x - 10, rp_y - 10, 280, 1080);
    cairo_stroke(cr);

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
        {"L VEL",     {}, 0.5, 0.7, 0.9},
        {"R VEL",     {}, 0.5, 0.7, 0.9},
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

    for (int i = 0; i < n_items; ++i) {
        double y = rp_y + 12 + i * 70;
        cairo_set_source_rgba(cr, 0.5, 0.6, 0.7, 0.8);
        cairo_set_font_size(cr, 18);
        cairo_move_to(cr, rp_x, y);
        cairo_show_text(cr, items[i].label);
        cairo_set_source_rgb(cr, items[i].r, items[i].g, items[i].b);
        cairo_set_font_size(cr, 32);
        cairo_move_to(cr, rp_x, y + 32);
        cairo_show_text(cr, items[i].value);
    }

    // ── Position Panel (right of PID panel) ───────────────────
    {
        double pp_x = 250 + 560 + 30;  // to the right of the parameter panel with a fixed gap
        double pp_y = 70;
        double pp_w = 220;
        double pp_h = 160;

        cairo_set_source_rgba(cr, 0.05, 0.08, 0.12, 0.85);
        cairo_rectangle(cr, pp_x, pp_y, pp_w, pp_h);
        cairo_fill(cr);
        cairo_set_source_rgba(cr, 0.2, 0.5, 0.4, 0.5);
        cairo_set_line_width(cr, 1);
        cairo_rectangle(cr, pp_x, pp_y, pp_w, pp_h);
        cairo_stroke(cr);

        cairo_set_source_rgb(cr, 0.5, 0.9, 0.7);
        cairo_set_font_size(cr, 20);
        cairo_move_to(cr, pp_x + 10, pp_y + 22);
        cairo_show_text(cr, "POSITION");

        struct PosItem {
            const char* label;
            double value;
        };
        PosItem pos_items[] = {
            {"DISP",   telem.pos_disp},
            {"TARGET", telem.pos_target},
            {"OUTPUT", telem.pos_out},
        };

        for (int i = 0; i < 3; i++) {
            double row_y = pp_y + 45 + i * 38;

            cairo_set_source_rgba(cr, 0.5, 0.6, 0.7, 0.8);
            cairo_set_font_size(cr, 16);
            cairo_move_to(cr, pp_x + 10, row_y);
            cairo_show_text(cr, pos_items[i].label);

            cairo_set_source_rgb(cr, 0.5, 0.9, 0.7);
            cairo_set_font_size(cr, 24);
            char val[32];
            snprintf(val, sizeof(val), "%.3f", pos_items[i].value);
            cairo_move_to(cr, pp_x + 90, row_y + 2);
            cairo_show_text(cr, val);
        }
    }

    // ── Charts: Real-time PID Debug ─────────────────────────
    auto draw_chart = [&](double chart_cx, double chart_cy, double cw, double ch_h,
                          const char* title,
                          std::initializer_list<std::pair<const WaylandApp::ChartHistory*, const char*>> traces,
                          std::initializer_list<std::tuple<double, double, double>> colors,
                          double y_range) {
        // Left/right margins for legend text and y-range labels
        double margin_l = 180;  // left margin for legend
        double margin_r = 80;   // right margin for y-range labels
        double plot_x = chart_cx + margin_l;
        double plot_w = cw - margin_l - margin_r;

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

        // Background (full width)
        cairo_set_source_rgba(cr, 0.04, 0.06, 0.10, 0.85);
        cairo_rectangle(cr, chart_cx, chart_cy, cw, ch_h);
        cairo_fill(cr);

        // Border (full width)
        cairo_set_source_rgba(cr, 0.2, 0.35, 0.5, 0.5);
        cairo_set_line_width(cr, 1);
        cairo_rectangle(cr, chart_cx, chart_cy, cw, ch_h);
        cairo_stroke(cr);

        // Zero line (plot area only)
        double zero_y = chart_cy + ch_h / 2.0;
        cairo_set_source_rgba(cr, 0.3, 0.4, 0.5, 0.4);
        cairo_set_line_width(cr, 0.5);
        cairo_move_to(cr, plot_x, zero_y);
        cairo_line_to(cr, plot_x + plot_w, zero_y);
        cairo_stroke(cr);

        // Title
        cairo_set_source_rgba(cr, 0.6, 0.7, 0.8, 0.8);
        cairo_set_font_size(cr, 24);
        cairo_move_to(cr, chart_cx + 8, chart_cy + 18);
        cairo_show_text(cr, title);

        // Y-range labels (right margin)
        cairo_set_source_rgba(cr, 0.4, 0.5, 0.6, 0.6);
        cairo_set_font_size(cr, 20);
        char lbl[16];
        snprintf(lbl, sizeof(lbl), "+%.1f", y_range);
        cairo_move_to(cr, plot_x + plot_w + 5, chart_cy + 18);
        cairo_show_text(cr, lbl);
        snprintf(lbl, sizeof(lbl), "-%.1f", y_range);
        cairo_move_to(cr, plot_x + plot_w + 5, chart_cy + ch_h - 5);
        cairo_show_text(cr, lbl);

        // Draw traces — clip to plot area
        cairo_save(cr);
        cairo_rectangle(cr, plot_x, chart_cy, plot_w, ch_h);
        cairo_clip(cr);
        cairo_new_path(cr);

        auto color_it = colors.begin();
        for (auto& [hist, name] : traces) {
            auto [cr_r, cr_g, cr_b] = *color_it;
            if (color_it + 1 != colors.end()) ++color_it;

            int n = hist->count;
            if (n < 2) continue;

            cairo_set_source_rgba(cr, cr_r, cr_g, cr_b, 0.9);
            cairo_set_line_width(cr, 2.0);

            bool started = false;
            for (int i = 0; i < n; i++) {
                double v = hist->get(i);
                double px = plot_x + (double)i / (WaylandApp::CHART_LEN - 1) * plot_w;
                double py = zero_y - (v / y_range) * (ch_h / 2.0 - 4);
                py = std::max(chart_cy + 2.0, std::min(chart_cy + ch_h - 2.0, py));
                if (!started) { cairo_move_to(cr, px, py); started = true; }
                else { cairo_line_to(cr, px, py); }
            }
            cairo_stroke(cr);
            cairo_new_path(cr);
        }

        // Vertical cursor
        const int effective_cursor = app.clamp_cursor_pos_to_data();
        const int chart_samples = app.chart_sample_count();
        if ((app.chart_paused || app.cursor_visible) && effective_cursor >= 0 && chart_samples > 0) {
            double cursor_px = plot_x + (double)effective_cursor / (WaylandApp::CHART_LEN - 1) * plot_w;
            cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.6);
            cairo_set_line_width(cr, 1.0);
            cairo_move_to(cr, cursor_px, chart_cy);
            cairo_line_to(cr, cursor_px, chart_cy + ch_h);
            cairo_stroke(cr);
        }

        cairo_restore(cr);  // remove clip

        // Legend (left margin, outside clip)
        auto color_it2 = colors.begin();
        int legend_idx2 = 0;
        for (auto& [hist2, name2] : traces) {
            auto [lr, lg, lb] = *color_it2;
            if (color_it2 + 1 != colors.end()) ++color_it2;

            cairo_set_font_size(cr, 18);
            double ly = chart_cy + 40 + legend_idx2 * 22;
            // Color swatch
            cairo_set_source_rgba(cr, lr, lg, lb, 0.9);
            cairo_set_line_width(cr, 3);
            cairo_move_to(cr, chart_cx + 8, ly - 4);
            cairo_line_to(cr, chart_cx + 20, ly - 4);
            cairo_stroke(cr);
            // Label + value (at cursor if paused, else latest)
            char legend_buf[48];
            int n2 = hist2->count;
            if (n2 > 0) {
                int idx = ((app.chart_paused || app.cursor_visible) && effective_cursor >= 0 && effective_cursor < n2)
                          ? effective_cursor : n2 - 1;
                snprintf(legend_buf, sizeof(legend_buf), "%s: %.2f", name2, hist2->get(idx));
            } else {
                snprintf(legend_buf, sizeof(legend_buf), "%s", name2);
            }
            cairo_move_to(cr, chart_cx + 24, ly);
            cairo_show_text(cr, legend_buf);
            legend_idx2++;
        }
    };

    double chart_x = 30, chart_w = w - 360;
    double chart_h = 160, chart_gap = 8;
    double chart_y0 = 634;

    // IMU chart above the three main charts
    draw_chart(chart_x, chart_y0 - chart_h - chart_gap, chart_w, chart_h,
        "IMU",
        {{&app.ch_gyro_x, "gX"},
         {&app.ch_gyro_y, "gY"},
         {&app.ch_gyro_z, "gZ"},
         {&app.ch_acc_x, "aX"},
         {&app.ch_acc_y, "aY"},
         {&app.ch_acc_z, "aZ"}},
        {{0.9, 0.3, 0.3},
         {0.3, 0.9, 0.3},
         {0.3, 0.3, 0.9},
         {0.9, 0.6, 0.2},
         {0.2, 0.9, 0.9},
         {0.9, 0.2, 0.9}},
        0);

    draw_chart(chart_x, chart_y0, chart_w, chart_h,
        "PITCH (\xC2\xB0)",
        {{&app.ch_tgt_pitch, "tgt"},
         {&app.ch_flt_pitch, "act"},
         {&app.ch_pitch_err, "err"}},
        {{0.9, 0.5, 0.2},
         {0.2, 0.9, 0.4},
         {0.9, 0.3, 0.3}},
        0);

    draw_chart(chart_x, chart_y0 + chart_h + chart_gap, chart_w, chart_h,
        "MOTOR VEL (t/s)",
        {{&app.ch_pitch_out, "base"},
         {&app.ch_cmd_l, "cmd_L"},
         {&app.ch_cmd_r, "cmd_R"},
         {&app.ch_left_vel, "vel_L"},
         {&app.ch_right_vel, "vel_R"}},
        {{0.9, 0.9, 0.3},
         {0.2, 0.8, 0.4},
         {0.8, 0.4, 0.2},
         {0.3, 0.6, 0.9},
         {0.9, 0.3, 0.9}},
        0);

    draw_chart(chart_x, chart_y0 + 2 * (chart_h + chart_gap), chart_w, chart_h,
        "ENCODER POS (turns)",
        {{&app.ch_pos_l, "L pos"},
         {&app.ch_pos_r, "R pos"},
         {&app.ch_spd_est, "drift"}},
        {{0.3, 0.6, 0.9},
         {0.9, 0.3, 0.9},
         {0.9, 0.9, 0.3}},
        0);

    // ── Bottom: Button hints ────────────────────────────────
    cairo_set_source_rgba(cr, 0.4, 0.5, 0.6, 0.7);
    cairo_set_font_size(cr, 26);
    cairo_move_to(cr, 50, h - 20);
    cairo_show_text(cr, "[MENU] Arm  [B] Disarm  [X] E-STOP  [A] Pause  [LT/RT] Cursor  [D-Pad] PID  [LB/RB] Group  [Y] Send");

    if (app.chart_paused) {
        cairo_set_source_rgb(cr, 1.0, 0.4, 0.3);
        cairo_set_font_size(cr, 32);
        cairo_move_to(cr, w - 180, h - 15);
        cairo_show_text(cr, "PAUSED");
    }

    // ── Cleanup ─────────────────────────────────────────────
    cairo_destroy(cr);
    cairo_surface_destroy(cs);
}
