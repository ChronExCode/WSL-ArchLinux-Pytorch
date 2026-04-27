#include "logger.h"
#include "mini_json.h"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace robot {
namespace {

template <typename AddFn>
void add_config_fields(minijson::ObjectBuilder& obj, const Config& cfg, AddFn add_bool) {
    obj.add_number("pitch_kp", cfg.pitch_kp); obj.add_number("pitch_ki", cfg.pitch_ki); obj.add_number("pitch_kd", cfg.pitch_kd); obj.add_number("pitch_d_alpha", cfg.pitch_d_alpha);
    obj.add_number("yaw_kp", cfg.yaw_kp); obj.add_number("yaw_ki", cfg.yaw_ki); obj.add_number("yaw_kd", cfg.yaw_kd);
    obj.add_number("speed_kp", cfg.speed_kp); obj.add_number("speed_ki", cfg.speed_ki); obj.add_number("speed_kd", cfg.speed_kd);
    obj.add_number("pos_kp", cfg.pos_kp); obj.add_number("pos_ki", cfg.pos_ki); obj.add_number("pos_kd", cfg.pos_kd);
    obj.add_number("pos_out_max_fwd", cfg.pos_out_max_fwd); obj.add_number("pos_out_max_bwd", cfg.pos_out_max_bwd);
    obj.add_number("target_pitch_max_fwd", cfg.target_pitch_max_fwd); obj.add_number("target_pitch_max_bwd", cfg.target_pitch_max_bwd);
    obj.add_number("comp_filter_alpha", cfg.comp_filter_alpha); obj.add_number("pitch_lpf_alpha", cfg.pitch_lpf_alpha); obj.add_number("wheel_velocity_lpf_alpha", cfg.wheel_velocity_lpf_alpha); obj.add_number("command_lpf_alpha", cfg.command_lpf_alpha); obj.add_number("steering_lpf_alpha", cfg.steering_lpf_alpha);
    obj.add_number("pitch_offset", cfg.pitch_offset); obj.add_number("max_velocity", cfg.max_velocity); obj.add_number("max_torque", cfg.max_torque); obj.add_number("joystick_deadband", cfg.joystick_deadband); obj.add_number("control_rate_hz", cfg.control_rate_hz); obj.add_number("speed_ramp_rate", cfg.speed_ramp_rate); obj.add_number("tcp_port", cfg.tcp_port); obj.add_number("msp_request_interval_us", cfg.msp_request_interval_us); obj.add_number("max_tilt_angle", cfg.max_tilt_angle); obj.add_number("rc_timeout_ms", cfg.rc_timeout_ms); obj.add_number("wheel_base_m", cfg.wheel_base_m); obj.add_number("usb_bulk_timeout_ms", cfg.usb_bulk_timeout_ms); obj.add_number("usb_rx_queue_depth", cfg.usb_rx_queue_depth); add_bool("logging_enable", cfg.logging_enable); obj.add_number("logging_decimation", cfg.logging_decimation);
    add_bool("adaptive_balance_enable", cfg.adaptive_balance_enable); obj.add_number("adaptive_balance_rate", cfg.adaptive_balance_rate); obj.add_number("adaptive_balance_max_trim_deg", cfg.adaptive_balance_max_trim_deg); obj.add_number("adaptive_balance_pitch_window_deg", cfg.adaptive_balance_pitch_window_deg); obj.add_number("adaptive_balance_gyro_window_dps", cfg.adaptive_balance_gyro_window_dps); obj.add_number("adaptive_balance_speed_window_tps", cfg.adaptive_balance_speed_window_tps); obj.add_number("adaptive_balance_cmd_window", cfg.adaptive_balance_cmd_window);
    obj.add_number("target_speed_rate_limit", cfg.target_speed_rate_limit); obj.add_number("target_pitch_rate_limit_dps", cfg.target_pitch_rate_limit_dps); obj.add_number("target_displacement_catchup_rate", cfg.target_displacement_catchup_rate); obj.add_number("stop_speed_threshold_tps", cfg.stop_speed_threshold_tps); obj.add_number("stop_hold_capture_window_tps", cfg.stop_hold_capture_window_tps); obj.add_number("reference_position_gain", cfg.reference_position_gain); obj.add_number("reference_velocity_damping_gain", cfg.reference_velocity_damping_gain); obj.add_number("speed_to_pitch_ff", cfg.speed_to_pitch_ff); obj.add_number("accel_to_pitch_ff", cfg.accel_to_pitch_ff); obj.add_number("velocity_feedforward_gain", cfg.velocity_feedforward_gain); add_bool("auto_tune_enable", cfg.auto_tune_enable); obj.add_number("auto_tune_rate", cfg.auto_tune_rate); obj.add_number("auto_tune_min_scale", cfg.auto_tune_min_scale); obj.add_number("auto_tune_max_scale", cfg.auto_tune_max_scale); obj.add_number("auto_tune_pitch_error_target_deg", cfg.auto_tune_pitch_error_target_deg); obj.add_number("auto_tune_speed_error_target_tps", cfg.auto_tune_speed_error_target_tps); obj.add_number("friction_comp_tps", cfg.friction_comp_tps); obj.add_number("friction_comp_deadband_tps", cfg.friction_comp_deadband_tps); obj.add_number("friction_comp_fade_tps", cfg.friction_comp_fade_tps);
    obj.add_number("observer_pos_alpha", cfg.observer_pos_alpha); obj.add_number("observer_vel_beta", cfg.observer_vel_beta);
    add_bool("ekf_enable", cfg.ekf_enable); obj.add_number("ekf_q_position", cfg.ekf_q_position); obj.add_number("ekf_q_velocity", cfg.ekf_q_velocity); obj.add_number("ekf_q_pitch", cfg.ekf_q_pitch); obj.add_number("ekf_q_pitch_rate", cfg.ekf_q_pitch_rate); obj.add_number("ekf_r_position", cfg.ekf_r_position); obj.add_number("ekf_r_velocity", cfg.ekf_r_velocity); obj.add_number("ekf_r_pitch", cfg.ekf_r_pitch); obj.add_number("ekf_r_pitch_rate", cfg.ekf_r_pitch_rate); obj.add_number("ekf_init_pos_var", cfg.ekf_init_pos_var); obj.add_number("ekf_init_vel_var", cfg.ekf_init_vel_var); obj.add_number("ekf_init_pitch_var", cfg.ekf_init_pitch_var); obj.add_number("ekf_init_pitch_rate_var", cfg.ekf_init_pitch_rate_var);
    obj.add_number("body_mass_kg", cfg.body_mass_kg); obj.add_number("wheel_mass_kg", cfg.wheel_mass_kg); obj.add_number("com_height_m", cfg.com_height_m); obj.add_number("wheel_radius_m", cfg.wheel_radius_m); obj.add_number("drivetrain_time_constant_s", cfg.drivetrain_time_constant_s); obj.add_number("gravity_mps2", cfg.gravity_mps2); obj.add_number("auto_lqr_enable", cfg.auto_lqr_enable); obj.add_number("lqr_q_x", cfg.lqr_q_x); obj.add_number("lqr_q_v", cfg.lqr_q_v); obj.add_number("lqr_q_theta", cfg.lqr_q_theta); obj.add_number("lqr_q_theta_dot", cfg.lqr_q_theta_dot); obj.add_number("lqr_r_u", cfg.lqr_r_u);
    obj.add_number("lqr_k_theta", cfg.lqr_k_theta); obj.add_number("lqr_k_theta_d", cfg.lqr_k_theta_d); obj.add_number("lqr_k_x", cfg.lqr_k_x); obj.add_number("lqr_k_v", cfg.lqr_k_v); obj.add_number("lqr_speed_gain_scale", cfg.lqr_speed_gain_scale); obj.add_number("lqr_gain_scale_max", cfg.lqr_gain_scale_max); obj.add_number("lqr_integral_k", cfg.lqr_integral_k); obj.add_number("lqr_integral_limit", cfg.lqr_integral_limit);
    add_bool("nmpc_enabled", cfg.nmpc_enabled); obj.add_number("nmpc_horizon_steps", cfg.nmpc_horizon_steps); obj.add_number("nmpc_candidate_count", cfg.nmpc_candidate_count); obj.add_number("nmpc_pitch_min_deg", cfg.nmpc_pitch_min_deg); obj.add_number("nmpc_pitch_max_deg", cfg.nmpc_pitch_max_deg); obj.add_number("nmpc_pitch_slew_dps", cfg.nmpc_pitch_slew_dps); obj.add_number("nmpc_model_accel_gain", cfg.nmpc_model_accel_gain); obj.add_number("nmpc_model_accel_damping", cfg.nmpc_model_accel_damping); obj.add_number("nmpc_model_theta_stiffness", cfg.nmpc_model_theta_stiffness); obj.add_number("nmpc_model_theta_damping", cfg.nmpc_model_theta_damping); obj.add_number("nmpc_model_couple_gain", cfg.nmpc_model_couple_gain);
    obj.add_number("nmpc_w_theta", cfg.nmpc_w_theta); obj.add_number("nmpc_w_theta_rate", cfg.nmpc_w_theta_rate); obj.add_number("nmpc_w_x", cfg.nmpc_w_x); obj.add_number("nmpc_w_v", cfg.nmpc_w_v); obj.add_number("nmpc_w_u", cfg.nmpc_w_u); obj.add_number("nmpc_w_du", cfg.nmpc_w_du); obj.add_number("nmpc_w_terminal_theta", cfg.nmpc_w_terminal_theta); obj.add_number("nmpc_w_terminal_x", cfg.nmpc_w_terminal_x); obj.add_number("nmpc_w_terminal_v", cfg.nmpc_w_terminal_v); obj.add_number("nmpc_w_terminal_u", cfg.nmpc_w_terminal_u); obj.add_number("nmpc_reference_velocity_blend", cfg.nmpc_reference_velocity_blend); obj.add_number("nmpc_reference_position_preview_gain", cfg.nmpc_reference_position_preview_gain); obj.add_number("nmpc_terminal_lqr_scale", cfg.nmpc_terminal_lqr_scale); obj.add_number("nmpc_ltv_velocity_scale", cfg.nmpc_ltv_velocity_scale); obj.add_number("nmpc_ltv_pitch_scale", cfg.nmpc_ltv_pitch_scale); obj.add_number("nmpc_use_physical_model", cfg.nmpc_use_physical_model); obj.add_number("nmpc_use_auto_lqr_terminal", cfg.nmpc_use_auto_lqr_terminal); obj.add_number("nmpc_stage_lqr_tail_mix", cfg.nmpc_stage_lqr_tail_mix);
    obj.add_number("stale_result_max_age_s", cfg.stale_result_max_age_s); obj.add_number("nmpc_state_mismatch_pitch_deg", cfg.nmpc_state_mismatch_pitch_deg); obj.add_number("nmpc_state_mismatch_vel_tps", cfg.nmpc_state_mismatch_vel_tps); obj.add_number("nmpc_state_mismatch_disp_turns", cfg.nmpc_state_mismatch_disp_turns);
}

std::string config_to_json(const Config& cfg) {
    minijson::ObjectBuilder obj;
    add_config_fields(obj, cfg, [&](const std::string& k, bool v){ obj.add_bool(k, v); });
    return obj.str();
}

} // namespace

bool CsvLogger::start(const std::string& directory, bool enabled) {
    enabled_ = enabled;
    if (!enabled_) return true;
    namespace fs = std::filesystem;
    fs::create_directories(directory);
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream name;
    name << directory << "/run_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    file_.open(name.str());
    if (!file_.is_open()) { enabled_ = false; return false; }
    file_ << "t_s,state,pitch,roll,yaw,gyro_y,acc_x,acc_y,acc_z,throttle,steering,speed_limit,tgt_pitch,flt_pitch,tgt_speed,spd_est,pos_disp,pos_target,base_cmd,cmd_l,cmd_r,enc_disp,enc_vel,nmpc_used,nmpc_cost\n";
    return true;
}

void CsvLogger::write_comment_line_unlocked(const std::string& line) {
    if (enabled_ && file_.is_open()) file_ << "#" << line << '\n';
}

void CsvLogger::log_config_snapshot(const Config& cfg) {
    if (!enabled_ || !file_.is_open()) return;
    std::lock_guard<std::mutex> lock(mtx_);
    write_comment_line_unlocked("CONFIG_SNAPSHOT " + config_to_json(cfg));
}

void CsvLogger::log_config_update(double t_s, const Config& cfg, const std::string& source) {
    if (!enabled_ || !file_.is_open()) return;
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostringstream oss;
    oss << "CONFIG_UPDATE t_s=" << std::setprecision(10) << t_s << " source=\"" << minijson::escape(source) << "\" payload=" << config_to_json(cfg);
    write_comment_line_unlocked(oss.str());
}

void CsvLogger::log_event(double t_s, const std::string& name, const std::string& message) {
    if (!enabled_ || !file_.is_open()) return;
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostringstream oss;
    oss << "EVENT t_s=" << std::setprecision(10) << t_s << " name=\"" << minijson::escape(name) << "\" msg=\"" << minijson::escape(message) << "\"";
    write_comment_line_unlocked(oss.str());
}

void CsvLogger::stop() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (file_.is_open()) file_.close();
}

void CsvLogger::log(double t_s, SystemState state, const ImuData& imu, const RemoteCommand& cmd,
                    const PidDebugData& dbg, const MotorOutput& out, double enc_disp, double enc_vel) {
    if (!enabled_ || !file_.is_open()) return;
    std::lock_guard<std::mutex> lock(mtx_);
    file_ << t_s << ',' << static_cast<int>(state) << ','
          << imu.pitch << ',' << imu.roll << ',' << imu.yaw << ',' << imu.gyro_y << ','
          << imu.acc_x << ',' << imu.acc_y << ',' << imu.acc_z << ','
          << cmd.throttle << ',' << cmd.steering << ',' << cmd.speed_limit << ','
          << dbg.target_pitch << ',' << dbg.filtered_pitch << ',' << dbg.target_speed << ',' << dbg.speed_estimate << ','
          << dbg.pos_disp << ',' << dbg.pos_target << ',' << dbg.base_cmd << ','
          << out.left_velocity << ',' << out.right_velocity << ','
          << enc_disp << ',' << enc_vel << ','
          << (dbg.nmpc_used ? 1 : 0) << ',' << dbg.nmpc_cost << '\n';
}

}  // namespace robot
