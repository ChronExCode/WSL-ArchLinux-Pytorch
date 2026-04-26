#pragma once
#include "controller_types.h"
#include "gamepad_reader.h"
#include "tcp_client.h"
#include <wayland-client.h>
#include "xdg-shell-protocol.h"
#include <algorithm>
#include <chrono>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>

struct WaylandApp {
    wl_display* display=nullptr; wl_registry* registry=nullptr; wl_compositor* compositor=nullptr; wl_shm* shm=nullptr; wl_seat* seat=nullptr; wl_keyboard* keyboard=nullptr; xdg_wm_base* xdg_base=nullptr;
    wl_surface* surface=nullptr; xdg_surface* xdg_surf=nullptr; xdg_toplevel* toplevel=nullptr;
    struct ShmBuffer { wl_buffer* buffer=nullptr; void* shm_data=nullptr; int shm_fd=-1; size_t shm_size=0; bool busy=false; };
    static constexpr int BUFFER_COUNT=2; ShmBuffer buffers[BUFFER_COUNT]; int front_buffer=0;
    bool running=true; bool configured=false; int width=WINDOW_WIDTH; int height=WINDOW_HEIGHT;
    GamepadReader gamepad;
    TcpClient tcp; std::string server_ip="10.0.0.218"; bool armed=false; bool chart_paused=false;
    std::mutex telem_mtx; Telemetry telem;
    int axis_steering=ABS_X, axis_throttle=ABS_Y, axis_speed_lt=ABS_Z, axis_speed_rt=ABS_RZ, btn_arm=313, btn_estop=312, btn_disarm=306;
    bool joy_map_mode=false, auto_switch_dinput=true; double throttle=0.0, steering=0.0;

    std::vector<std::string> param_groups;
    std::vector<PidParam> params;
    static constexpr int MAX_PARAMS_PER_GROUP = 5;
    int pid_group=0, pid_sel=0; bool pid_dirty=false;

    WaylandApp(){ initialize_params(); }
    void initialize_params();
    int current_pid_count() const { int c=0; for (const auto& p:params) if (p.group==pid_group) ++c; return c; }
    PidParam* current_pid_param(){ int idx=0; for (auto& p:params){ if(p.group!=pid_group) continue; if(idx==pid_sel) return &p; ++idx;} return nullptr; }
    std::vector<PidParam*> current_pid_params(){ std::vector<PidParam*> out; for (auto& p:params) if(p.group==pid_group) out.push_back(&p); return out; }
    void clamp_selection(){ int c=current_pid_count(); pid_sel = c<=0?0:std::max(0,std::min(pid_sel,c-1)); }
    const std::string& current_group_name() const { return param_groups[pid_group]; }
    std::string serialize_param_json(const char* type="set_pid") const;

    int cursor_pos=-1; bool cursor_visible=false; std::chrono::steady_clock::time_point cursor_last_active;
    static constexpr const char* CONFIG_FILE="params.json"; std::string config_dir; bool save_config() const; bool load_config();

    static constexpr int CHART_LEN=200; struct ChartHistory { double data[200] = {}; int head=0,count=0; void push(double v){data[head]=v; head=(head+1)%200; if(count<200)count++;} double get(int i) const { if(i<0||i>=count) return 0; return data[(head-count+i+200)%200]; } void clear(){head=0;count=0;} };
    ChartHistory ch_pitch_out,ch_left_vel,ch_right_vel,ch_pitch_err,ch_tgt_pitch,ch_actual_pitch,ch_flt_pitch,ch_flt_pitch_rate,ch_tgt_speed,ch_spd_est,ch_flt_vel,ch_ang_vel,ch_ang_acc,ch_pos_l,ch_pos_r,ch_pos_disp,ch_obs_disp,ch_cmd_l,ch_cmd_r,ch_gyro_x,ch_gyro_y,ch_gyro_z,ch_acc_x,ch_acc_y,ch_acc_z;
    std::string prev_state;
    void push_chart_data(const Telemetry& t){ if(t.state!=prev_state){ if(t.state=="BALANCING") clear_all_charts(); prev_state=t.state;} if(t.state!="BALANCING") return; ch_pitch_out.push(t.pitch_out); ch_left_vel.push(t.left_vel); ch_right_vel.push(t.right_vel); ch_pitch_err.push(t.pitch_err); ch_tgt_pitch.push(t.tgt_pitch); ch_actual_pitch.push(t.actual_pitch); ch_flt_pitch.push(t.flt_pitch); ch_flt_pitch_rate.push(t.flt_pitch_rate); ch_tgt_speed.push(t.tgt_speed); ch_spd_est.push(t.spd_est); ch_flt_vel.push(t.flt_vel); ch_ang_vel.push(t.ang_vel); ch_ang_acc.push(t.ang_acc); ch_pos_l.push(t.pos_l); ch_pos_r.push(t.pos_r); ch_pos_disp.push(t.pos_disp); ch_obs_disp.push(t.obs_disp); ch_cmd_l.push(t.cmd_l); ch_cmd_r.push(t.cmd_r); ch_gyro_x.push(t.gyro_x); ch_gyro_y.push(t.gyro_y); ch_gyro_z.push(t.gyro_z); ch_acc_x.push(t.acc_x); ch_acc_y.push(t.acc_y); ch_acc_z.push(t.acc_z);}
    void clear_all_charts(){ ch_pitch_out.clear(); ch_left_vel.clear(); ch_right_vel.clear(); ch_pitch_err.clear(); ch_tgt_pitch.clear(); ch_actual_pitch.clear(); ch_flt_pitch.clear(); ch_flt_pitch_rate.clear(); ch_tgt_speed.clear(); ch_spd_est.clear(); ch_flt_vel.clear(); ch_ang_vel.clear(); ch_ang_acc.clear(); ch_pos_l.clear(); ch_pos_r.clear(); ch_pos_disp.clear(); ch_obs_disp.clear(); ch_cmd_l.clear(); ch_cmd_r.clear(); ch_gyro_x.clear(); ch_gyro_y.clear(); ch_gyro_z.clear(); ch_acc_x.clear(); ch_acc_y.clear(); ch_acc_z.clear(); }
    int chart_sample_count() const { return ch_pitch_out.count; }
    int clamp_cursor_pos_to_data() const { const int n = chart_sample_count(); if (n <= 0) return -1; if (cursor_pos < 0) return n / 2; return std::max(0, std::min(cursor_pos, n - 1)); }
};

inline void WaylandApp::initialize_params(){
    params.clear();
    std::vector<std::string> base_groups={"PITCH PID","YAW/SPEED PID","POSITION/LIMITS","FILTERS","CORE","ADAPTIVE","REFERENCE","OBSERVER","EKF","MODEL","AUTO LQR","MANUAL LQR","NMPC MODEL","NMPC COST","NMPC CONSISTENCY","NMPC VALID"};
    auto add=[&](const char* key,const char* label,double value,double step,double min,double max,int group){ params.push_back({key,label,value,step,min,max,group}); };
    add("pitch_kp","pitch_kp",0.30,0.01,0.0,5.0,0); add("pitch_ki","pitch_ki",0.005,0.005,0.0,1.0,0); add("pitch_kd","pitch_kd",0.025,0.005,0.0,1.0,0); add("pitch_d_alpha","pitch_d_alpha",0.2,0.05,0.01,1.0,0);
    add("yaw_kp","yaw_kp",1.0,0.1,0.0,20.0,1); add("yaw_ki","yaw_ki",0.0,0.01,0.0,5.0,1); add("yaw_kd","yaw_kd",0.0,0.01,0.0,5.0,1); add("speed_kp","speed_kp",0.6,0.01,0.0,5.0,1); add("speed_ki","speed_ki",0.0,0.005,0.0,2.0,1); add("speed_kd","speed_kd",0.0,0.005,0.0,2.0,1);
    add("pos_kp","pos_kp",0.5,0.05,0.0,5.0,2); add("pos_ki","pos_ki",0.1,0.01,0.0,2.0,2); add("pos_kd","pos_kd",0.2,0.05,0.0,5.0,2); add("pos_out_max_fwd","pos_out_max_fwd",5.0,0.5,0.5,20.0,2); add("pos_out_max_bwd","pos_out_max_bwd",-5.0,0.5,-20.0,-0.5,2); add("target_pitch_max_fwd","target_pitch_max_fwd",15.0,1.0,1.0,45.0,2); add("target_pitch_max_bwd","target_pitch_max_bwd",-15.0,1.0,-45.0,-1.0,2);
    add("comp_filter_alpha","comp_filter_alpha",0.98,0.01,0.0,1.0,3); add("pitch_lpf_alpha","pitch_lpf_alpha",0.50,0.01,0.0,1.0,3); add("wheel_velocity_lpf_alpha","wheel_velocity_lpf_alpha",0.25,0.01,0.0,1.0,3); add("command_lpf_alpha","command_lpf_alpha",0.18,0.01,0.0,1.0,3); add("steering_lpf_alpha","steering_lpf_alpha",0.20,0.01,0.0,1.0,3);
    add("pitch_offset","pitch_offset",0.0,0.1,-10.0,10.0,4); add("max_velocity","max_velocity",2.0,0.1,0.2,20.0,4); add("max_torque","max_torque",6.0,0.2,0.0,20.0,4); add("joystick_deadband","joystick_deadband",0.05,0.01,0.0,0.4,4); add("speed_ramp_rate","speed_ramp_rate",10.0,0.5,0.5,50.0,4); add("max_tilt_angle","max_tilt_angle",45.0,1.0,10.0,80.0,4); add("wheel_base_m","wheel_base_m",0.45,0.01,0.1,1.0,4);
    add("adaptive_balance_enable","adaptive_enable",1.0,1.0,0.0,1.0,5); add("adaptive_balance_rate","adaptive_rate",0.08,0.01,0.0,1.0,5); add("adaptive_balance_max_trim_deg","adaptive_max_trim",6.0,0.2,0.0,20.0,5); add("adaptive_balance_pitch_window_deg","adaptive_pitch_win",8.0,0.5,0.5,20.0,5); add("adaptive_balance_gyro_window_dps","adaptive_gyro_win",25.0,1.0,0.0,200.0,5); add("adaptive_balance_speed_window_tps","adaptive_speed_win",0.35,0.05,0.0,5.0,5); add("adaptive_balance_cmd_window","adaptive_cmd_win",0.08,0.01,0.0,1.0,5);
    add("target_speed_rate_limit","speed_rate_limit",6.0,0.0,0.1,30.0,6); add("target_pitch_rate_limit_dps","pitch_rate_limit",120.0,0.0,5.0,500.0,6); add("target_displacement_catchup_rate","disp_catchup",0.8,0.05,0.0,10.0,6); add("stop_speed_threshold_tps","stop_speed_thresh",0.12,0.01,0.0,2.0,6); add("stop_hold_capture_window_tps","stop_hold_window",0.20,0.01,0.0,2.0,6); add("reference_position_gain","ref_pos_gain",0.18,0.01,0.0,5.0,6); add("reference_velocity_damping_gain","ref_vel_damp",0.10,0.01,0.0,5.0,6); add("speed_to_pitch_ff","speed_to_pitch_ff",2.0,0.05,0.0,20.0,6); add("accel_to_pitch_ff","accel_to_pitch_ff",0.25,0.01,0.0,10.0,6);
    add("observer_pos_alpha","observer_pos_alpha",0.55,0.01,0.0,1.0,7); add("observer_vel_beta","observer_vel_beta",0.18,0.01,0.0,1.0,7);
    add("ekf_enable","ekf_enable",1.0,1.0,0.0,1.0,8); add("ekf_q_position","ekf_q_pos",1e-4,1e-5,1e-9,1.0,8); add("ekf_q_velocity","ekf_q_vel",2e-2,1e-3,1e-9,10.0,8); add("ekf_q_pitch","ekf_q_pitch",4e-2,1e-3,1e-9,10.0,8); add("ekf_q_pitch_rate","ekf_q_rate",1.5,0.05,1e-9,100.0,8);
    add("ekf_r_position","ekf_r_pos",2e-4,1e-5,1e-9,1.0,8); add("ekf_r_velocity","ekf_r_vel",4e-2,1e-3,1e-9,10.0,8); add("ekf_r_pitch","ekf_r_pitch",0.8,0.05,1e-9,100.0,8); add("ekf_r_pitch_rate","ekf_r_rate",2.0,0.05,1e-9,100.0,8); add("ekf_init_pos_var","ekf_init_pos",5e-3,1e-4,1e-9,10.0,8); add("ekf_init_vel_var","ekf_init_vel",5e-2,1e-3,1e-9,10.0,8); add("ekf_init_pitch_var","ekf_init_pitch",4.0,0.1,1e-9,100.0,8); add("ekf_init_pitch_rate_var","ekf_init_rate",16.0,0.5,1e-9,200.0,8);
    add("body_mass_kg","body_mass_kg",9.0,0.1,0.1,50.0,9); add("wheel_mass_kg","wheel_mass_kg",0.6,0.05,0.01,10.0,9); add("com_height_m","com_height_m",0.22,0.005,0.05,1.0,9); add("wheel_radius_m","wheel_radius_m",0.085,0.002,0.02,0.5,9); add("drivetrain_time_constant_s","drive_tau",0.08,0.005,0.01,1.0,9); add("gravity_mps2","gravity",9.81,0.01,5.0,15.0,9);
    add("auto_lqr_enable","auto_lqr_enable",1.0,1.0,0.0,1.0,10); add("lqr_q_x","lqr_q_x",20.0,1.0,0.0,1000.0,10); add("lqr_q_v","lqr_q_v",6.0,0.5,0.0,500.0,10); add("lqr_q_theta","lqr_q_theta",220.0,5.0,0.0,5000.0,10); add("lqr_q_theta_dot","lqr_q_theta_dot",18.0,1.0,0.0,1000.0,10); add("lqr_r_u","lqr_r_u",1.4,0.1,0.01,100.0,10);
    add("lqr_k_theta","lqr_k_theta",1.8,0.05,0.0,20.0,11); add("lqr_k_theta_d","lqr_k_theta_d",0.08,0.005,0.0,10.0,11); add("lqr_k_x","lqr_k_x",0.24,0.01,0.0,10.0,11); add("lqr_k_v","lqr_k_v",0.45,0.01,0.0,10.0,11); add("lqr_speed_gain_scale","lqr_gain_scale",0.18,0.01,0.0,5.0,11); add("lqr_gain_scale_max","lqr_gain_scale_max",1.6,0.05,0.5,10.0,11); add("lqr_integral_k","lqr_integral_k",0.0,0.005,0.0,5.0,11); add("lqr_integral_limit","lqr_integral_limit",3.0,0.1,0.0,50.0,11);
    add("nmpc_enabled","nmpc_enabled",1.0,1.0,0.0,1.0,12); add("nmpc_horizon_steps","nmpc_horizon",12.0,1.0,2.0,32.0,12); add("nmpc_candidate_count","nmpc_candidates",16.0,1.0,2.0,64.0,12); add("nmpc_pitch_min_deg","nmpc_pitch_min",-12.0,0.5,-45.0,0.0,12); add("nmpc_pitch_max_deg","nmpc_pitch_max",12.0,0.5,0.0,45.0,12); add("nmpc_pitch_slew_dps","nmpc_pitch_slew",180.0,5.0,1.0,720.0,12); add("nmpc_model_accel_gain","nmpc_accel_gain",7.5,0.1,0.0,100.0,12); add("nmpc_model_accel_damping","nmpc_accel_damp",2.0,0.1,0.0,50.0,12); add("nmpc_model_theta_stiffness","nmpc_theta_stiff",18.0,0.5,0.0,200.0,12); add("nmpc_model_theta_damping","nmpc_theta_damp",3.5,0.1,0.0,50.0,12); add("nmpc_model_couple_gain","nmpc_couple_gain",0.11,0.01,0.0,10.0,12);
    add("nmpc_w_theta","nmpc_w_theta",8.0,0.2,0.0,2000.0,13); add("nmpc_w_theta_rate","nmpc_w_theta_rate",0.25,0.05,0.0,200.0,13); add("nmpc_w_x","nmpc_w_x",2.5,0.1,0.0,500.0,13); add("nmpc_w_v","nmpc_w_v",0.8,0.05,0.0,500.0,13); add("nmpc_w_u","nmpc_w_u",0.05,0.01,0.0,100.0,13); add("nmpc_w_du","nmpc_w_du",0.15,0.01,0.0,100.0,13); add("nmpc_w_terminal_theta","nmpc_w_term_theta",6.0,0.2,0.0,2000.0,13); add("nmpc_w_terminal_x","nmpc_w_term_x",4.0,0.2,0.0,1000.0,13); add("nmpc_w_terminal_v","nmpc_w_term_v",2.0,0.2,0.0,1000.0,13); add("nmpc_w_terminal_u","nmpc_w_term_u",0.10,0.01,0.0,1000.0,13); add("nmpc_reference_velocity_blend","nmpc_v_blend",0.65,0.05,0.0,1.0,13); add("nmpc_reference_position_preview_gain","nmpc_x_preview",1.0,0.05,0.0,3.0,13); add("nmpc_terminal_lqr_scale","nmpc_term_lqr",1.0,0.05,0.0,10.0,13);
    add("nmpc_use_physical_model","nmpc_use_phys",1.0,1.0,0.0,1.0,13); add("nmpc_use_auto_lqr_terminal","nmpc_use_lqr_term",1.0,1.0,0.0,1.0,13); add("nmpc_stage_lqr_tail_mix","nmpc_tail_mix",0.35,0.05,0.0,1.0,13); add("nmpc_ltv_velocity_scale","nmpc_ltv_v",0.15,0.01,0.0,2.0,13); add("nmpc_ltv_pitch_scale","nmpc_ltv_pitch",0.10,0.01,0.0,2.0,13);
    add("stale_result_max_age_s","stale_age_s",0.05,0.005,0.0,1.0,15); add("nmpc_state_mismatch_pitch_deg","mismatch_pitch",6.0,0.2,0.0,30.0,15); add("nmpc_state_mismatch_vel_tps","mismatch_vel",0.8,0.05,0.0,10.0,15); add("nmpc_state_mismatch_disp_turns","mismatch_disp",0.35,0.01,0.0,10.0,15);

    // Rebuild UI groups so the panel never shows more than MAX_PARAMS_PER_GROUP rows.
    // This avoids text overlap on the handheld screen while keeping the original
    // parameter order stable for navigation, save/load, and TCP transmission.
    std::vector<std::string> split_groups;
    std::vector<int> seen(base_groups.size(), 0);
    std::vector<int> first_split(base_groups.size(), -1);
    for (auto& param : params) {
        const int base = param.group;
        const int chunk = seen[base] / MAX_PARAMS_PER_GROUP;
        if (chunk == 0) {
            if (first_split[base] < 0) {
                first_split[base] = static_cast<int>(split_groups.size());
                split_groups.push_back(base_groups[base]);
            }
        } else if (first_split[base] + chunk >= static_cast<int>(split_groups.size())) {
            split_groups.push_back(base_groups[base] + " " + std::to_string(chunk + 1));
        }
        param.group = first_split[base] + chunk;
        seen[base]++;
    }
    param_groups = std::move(split_groups);
}

inline std::string WaylandApp::serialize_param_json(const char* type) const {
    minijson::ObjectBuilder obj;
    obj.add_string("type", type);
    for (const auto& p : params) obj.add_number(p.key, p.value);
    return obj.str();
}
