#include "balance_controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "control_model.h"

namespace robot {

PID::PID(double kp, double ki, double kd, double out_min, double out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

double PID::compute(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return 0.0;
    last_setpoint_ = setpoint;
    const double error = setpoint - measurement;
    p_term_ = kp_ * error;
    const double pre_sat = p_term_ + i_term_ + d_term_;
    const bool sat = (pre_sat >= out_max_ && error > 0.0) || (pre_sat <= out_min_ && error < 0.0);
    if (!sat) integral_ += error * dt;
    i_term_ = clamp(ki_ * integral_, out_min_, out_max_);

    if (first_run_) {
        d_term_ = 0.0;
        prev_measurement_ = measurement;
        d_filtered_ = 0.0;
        first_run_ = false;
    } else {
        const double d_raw = -kd_ * (measurement - prev_measurement_) / dt;
        d_filtered_ = d_alpha_ * d_raw + (1.0 - d_alpha_) * d_filtered_;
        d_term_ = d_filtered_;
    }
    prev_measurement_ = measurement;
    prev_error_ = error;
    return clamp(p_term_ + i_term_ + d_term_, out_min_, out_max_);
}

double PID::compute_with_gyro(double setpoint, double measurement, double gyro_rate, double dt) {
    if (dt <= 0.0) return 0.0;
    last_setpoint_ = setpoint;
    const double error = setpoint - measurement;
    p_term_ = kp_ * error;
    const double pre_sat = p_term_ + i_term_ + d_term_;
    const bool sat = (pre_sat >= out_max_ && error > 0.0) || (pre_sat <= out_min_ && error < 0.0);
    if (!sat) integral_ += error * dt;
    i_term_ = clamp(ki_ * integral_, out_min_, out_max_);
    const double d_raw = -kd_ * gyro_rate;
    d_filtered_ = d_alpha_ * d_raw + (1.0 - d_alpha_) * d_filtered_;
    d_term_ = d_filtered_;
    prev_measurement_ = measurement;
    prev_error_ = error;
    first_run_ = false;
    return clamp(p_term_ + i_term_ + d_term_, out_min_, out_max_);
}

void PID::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    prev_measurement_ = 0.0;
    d_filtered_ = 0.0;
    first_run_ = true;
    p_term_ = i_term_ = d_term_ = last_setpoint_ = 0.0;
}

void PID::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::set_limits(double min, double max) {
    out_min_ = min;
    out_max_ = max;
}

ComplementaryFilter::ComplementaryFilter(double alpha) : alpha_(alpha) {}

double ComplementaryFilter::update(double acc_pitch_deg, double gyro_rate_dps, double dt) {
    if (!initialized_) {
        pitch_ = acc_pitch_deg;
        initialized_ = true;
        return pitch_;
    }
    pitch_ = alpha_ * (pitch_ + gyro_rate_dps * dt) + (1.0 - alpha_) * acc_pitch_deg;
    return pitch_;
}

void ComplementaryFilter::reset() {
    pitch_ = 0.0;
    initialized_ = false;
}

BalanceController::BalanceController() : BalanceController(Config{}) {}

BalanceController::BalanceController(const Config& cfg) : cfg_(cfg), comp_filter_(cfg.comp_filter_alpha) {
    legacy_pitch_pid_ = PID(cfg_.pitch_kp, cfg_.pitch_ki, cfg_.pitch_kd, -50.0, 50.0);
    legacy_pitch_pid_.set_d_alpha(cfg_.pitch_d_alpha);
    legacy_speed_pid_ = PID(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd, -15.0, 15.0);
    legacy_yaw_pid_ = PID(cfg_.yaw_kp, cfg_.yaw_ki, cfg_.yaw_kd, -50.0, 50.0);
    legacy_pos_pid_ = PID(cfg_.pos_kp, cfg_.pos_ki, cfg_.pos_kd, cfg_.pos_out_max_bwd, cfg_.pos_out_max_fwd);
    nmpc_.start(cfg_, 1.0 / std::max(1, cfg_.control_rate_hz));
    synthesize_auto_lqr();
}

BalanceController::~BalanceController() { nmpc_.stop(); }

void BalanceController::set_config(const Config& cfg) {
    cfg_ = cfg;
    legacy_pitch_pid_.set_gains(cfg_.pitch_kp, cfg_.pitch_ki, cfg_.pitch_kd);
    legacy_pitch_pid_.set_d_alpha(cfg_.pitch_d_alpha);
    legacy_speed_pid_.set_gains(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd);
    legacy_yaw_pid_.set_gains(cfg_.yaw_kp, cfg_.yaw_ki, cfg_.yaw_kd);
    legacy_pos_pid_.set_gains(cfg_.pos_kp, cfg_.pos_ki, cfg_.pos_kd);
    legacy_pos_pid_.set_limits(cfg_.pos_out_max_bwd, cfg_.pos_out_max_fwd);
    comp_filter_ = ComplementaryFilter(cfg_.comp_filter_alpha);
    nmpc_.set_config(cfg_, 1.0 / std::max(1, cfg_.control_rate_hz));
    synthesize_auto_lqr();
}

void BalanceController::synthesize_auto_lqr() {
    Vec4 K{};
    Mat4 P{};
    auto_lqr_valid_ = solve_velocity_lqr_from_config(cfg_, 1.0 / std::max(1, cfg_.control_rate_hz), 0.0, 0.0, cfg_.lqr_speed_gain_scale, 0.0, K, P, true);
    for (int i = 0; i < 4; ++i) auto_lqr_k_[i] = K[i];
}

void BalanceController::reset() {
    legacy_pitch_pid_.reset();
    legacy_speed_pid_.reset();
    legacy_yaw_pid_.reset();
    legacy_pos_pid_.reset();
    comp_filter_.reset();
    nmpc_.reset();
    ekf_.reset();

    adaptive_trim_deg_ = 0.0;
    filtered_pitch_deg_ = 0.0;
    filtered_pitch_rate_dps_ = 0.0;
    observed_displacement_turns_ = displacement_turns_;
    filtered_velocity_tps_ = 0.0;
    state_initialized_ = false;
    throttle_filtered_ = 0.0;
    steering_filtered_ = 0.0;
    target_speed_tps_ = 0.0;
    target_displacement_turns_ = displacement_turns_;
    position_hold_active_ = false;
    nominal_pitch_deg_ = balance_point_deg_ + cfg_.pitch_offset;
    final_target_pitch_deg_ = nominal_pitch_deg_;
    prev_target_speed_tps_ = 0.0;
    prev_target_pitch_deg_ = final_target_pitch_deg_;
    theta_integral_ = 0.0;
    prev_base_velocity_cmd_tps_ = 0.0;
    auto_tune_scale_ = 1.0;
    auto_tune_pitch_rms_ = 0.0;
    auto_tune_speed_rms_ = 0.0;
    control_tick_ = 0;
    debug_ = {};
}

void BalanceController::set_balance_point(double pitch_deg) {
    balance_point_deg_ = pitch_deg;
    adaptive_trim_deg_ = 0.0;
    target_displacement_turns_ = displacement_turns_;
    position_hold_active_ = false;
    nominal_pitch_deg_ = pitch_deg + cfg_.pitch_offset;
    final_target_pitch_deg_ = nominal_pitch_deg_;
    prev_target_pitch_deg_ = final_target_pitch_deg_;
    theta_integral_ = 0.0;
    prev_base_velocity_cmd_tps_ = 0.0;
    legacy_pos_pid_.reset();
    nmpc_.reset();
    std::cout << "[Balance] Balance point set to " << pitch_deg << "°\n";
}

void BalanceController::set_displacement(double turns) { displacement_turns_ = turns; }
void BalanceController::set_wheel_velocity(double turns_per_sec) { wheel_velocity_raw_tps_ = turns_per_sec; }
bool BalanceController::is_tilt_safe(double pitch_deg) const { return std::abs(pitch_deg) < cfg_.max_tilt_angle; }
double BalanceController::process_throttle(double raw) const { return apply_deadband(raw, cfg_.joystick_deadband); }
double BalanceController::process_steering(double raw) const { return apply_deadband(raw, cfg_.joystick_deadband); }
double BalanceController::gain_schedule(double base_gain, double speed_abs) const {
    const double scaled = base_gain * (1.0 + cfg_.lqr_speed_gain_scale * speed_abs);
    return std::min(scaled, base_gain * cfg_.lqr_gain_scale_max);
}

EstimatedState BalanceController::estimate_state(const ImuData& imu, double dt) {
    EstimatedState state{};
    if (dt <= 0.0) return state;

    const double acc_pitch = rad_to_deg(std::atan2(-imu.acc_x, std::sqrt(imu.acc_y * imu.acc_y + imu.acc_z * imu.acc_z + 1e-9)));
    double actual_pitch = comp_filter_.update(acc_pitch, imu.gyro_y, dt);
    if (imu.valid && std::abs(imu.pitch) < 90.0) {
        actual_pitch = 0.7 * imu.pitch + 0.3 * actual_pitch;
    }

    debug_.actual_pitch = actual_pitch;

    if (!state_initialized_) {
        filtered_pitch_deg_ = actual_pitch;
        observed_displacement_turns_ = displacement_turns_;
        filtered_velocity_tps_ = wheel_velocity_raw_tps_;
        filtered_pitch_rate_dps_ = imu.gyro_y;
        if (cfg_.ekf_enable) {
            ekf_.set_config(cfg_);
            ekf_.initialize(displacement_turns_, wheel_velocity_raw_tps_, actual_pitch, imu.gyro_y);
        }
        state_initialized_ = true;
    } else if (cfg_.ekf_enable) {
        ekf_.predict(prev_base_velocity_cmd_tps_, dt);
        ekf_.update_position(displacement_turns_);
        ekf_.update_velocity(wheel_velocity_raw_tps_);
        ekf_.update_pitch(actual_pitch);
        ekf_.update_pitch_rate(imu.gyro_y);
        const EstimatedState ekf_state = ekf_.get_state();
        observed_displacement_turns_ = ekf_state.displacement_turns;
        filtered_velocity_tps_ = ekf_state.velocity_tps;
        filtered_pitch_deg_ = ekf_state.pitch_deg;
        filtered_pitch_rate_dps_ = ekf_state.pitch_rate_dps;
    } else {
        filtered_pitch_deg_ = lpf(filtered_pitch_deg_, actual_pitch, cfg_.pitch_lpf_alpha);
        const double predicted_disp = observed_displacement_turns_ + filtered_velocity_tps_ * dt;
        const double disp_residual = displacement_turns_ - predicted_disp;
        observed_displacement_turns_ = predicted_disp + cfg_.observer_pos_alpha * disp_residual;
        filtered_velocity_tps_ += (cfg_.observer_vel_beta * disp_residual) / std::max(dt, 1e-6);
        filtered_velocity_tps_ = lpf(filtered_velocity_tps_, wheel_velocity_raw_tps_, cfg_.wheel_velocity_lpf_alpha);
        filtered_pitch_rate_dps_ = imu.gyro_y;
    }

    state.pitch_deg = filtered_pitch_deg_;
    state.pitch_rate_dps = filtered_pitch_rate_dps_;
    state.displacement_turns = observed_displacement_turns_;
    state.velocity_tps = filtered_velocity_tps_;
    state.balance_trim_deg = adaptive_trim_deg_;
    state.valid = imu.valid;

    const bool allow_trim = cfg_.adaptive_balance_enable &&
        std::abs(throttle_filtered_) < cfg_.adaptive_balance_cmd_window &&
        std::abs(steering_filtered_) < cfg_.adaptive_balance_cmd_window &&
        std::abs(state.velocity_tps) < cfg_.adaptive_balance_speed_window_tps &&
        std::abs(state.pitch_rate_dps) < cfg_.adaptive_balance_gyro_window_dps &&
        std::abs(state.pitch_deg - balance_point_deg_) < cfg_.adaptive_balance_pitch_window_deg;

    if (allow_trim) {
        const double err = state.pitch_deg - (balance_point_deg_ + adaptive_trim_deg_);
        adaptive_trim_deg_ += cfg_.adaptive_balance_rate * err * dt;
        adaptive_trim_deg_ = clamp(adaptive_trim_deg_, -cfg_.adaptive_balance_max_trim_deg, cfg_.adaptive_balance_max_trim_deg);
        state.balance_trim_deg = adaptive_trim_deg_;
    }

    return state;
}

ReferenceState BalanceController::generate_reference(const RemoteCommand& cmd, const EstimatedState& state, double dt) {
    ReferenceState ref{};

    const double throttle = process_throttle(cmd.throttle);
    const double steering = process_steering(cmd.steering);
    const double speed_mul = clamp(cmd.speed_limit, 0.0, 1.0);
    throttle_filtered_ = lpf(throttle_filtered_, throttle, cfg_.command_lpf_alpha);
    steering_filtered_ = lpf(steering_filtered_, steering, cfg_.steering_lpf_alpha);

    const double raw_speed_ref = throttle_filtered_ * cfg_.max_velocity * speed_mul;
    target_speed_tps_ = ramp_limit(target_speed_tps_, raw_speed_ref, cfg_.target_speed_rate_limit, dt);

    const double theta_bias = balance_point_deg_ + cfg_.pitch_offset + adaptive_trim_deg_;

    // Throttle-driven velocity mode with gated position hold.
    // While throttle is active or the robot is still rolling, the displacement
    // target follows measured displacement. After throttle returns to zero and
    // the robot is nearly stopped/upright, capture the current displacement and
    // use a weak position loop to hold the robot in place.
    const bool throttle_zero = std::abs(throttle_filtered_) < 1e-4;
    const bool target_speed_zero = std::abs(target_speed_tps_) <= cfg_.stop_speed_threshold_tps;
    const bool nearly_stopped = std::abs(state.velocity_tps) <= cfg_.stop_hold_capture_window_tps;
    const bool upright_enough = std::abs(state.pitch_deg - theta_bias) <= 8.0;
    const bool allow_position_hold = throttle_zero && target_speed_zero && nearly_stopped && upright_enough;

    if (!allow_position_hold) {
        position_hold_active_ = false;
        target_displacement_turns_ = state.displacement_turns;
    } else if (!position_hold_active_) {
        position_hold_active_ = true;
        target_displacement_turns_ = state.displacement_turns;
    }

    const double speed_accel = (target_speed_tps_ - prev_target_speed_tps_) / std::max(dt, 1e-6);

    // Cascaded balance control:
    //   throttle -> target_speed_tps_
    //   outer speed loop -> target lean angle
    //   zero-throttle hold -> weak position correction -> target lean angle
    //   inner balance loop -> wheel velocity command
    //
    // Positive theta_error commands positive wheel velocity. If the robot rolls
    // forward past the captured hold point, position_error > 0 makes target
    // pitch positive; theta_error becomes negative and the wheels pull back.
    const double speed_error_tps = target_speed_tps_ - state.velocity_tps;
    const double position_error_turns = position_hold_active_
        ? (state.displacement_turns - target_displacement_turns_)
        : 0.0;
    const double theta_from_speed = -cfg_.speed_to_pitch_ff * speed_error_tps;
    const double theta_from_accel = -cfg_.accel_to_pitch_ff * speed_accel;
    const double theta_from_position = cfg_.reference_position_gain * position_error_turns;
    const double theta_from_velocity_damping = cfg_.reference_velocity_damping_gain * state.velocity_tps;

    nominal_pitch_deg_ = theta_bias + theta_from_speed + theta_from_accel +
                         theta_from_position + theta_from_velocity_damping;
    nominal_pitch_deg_ = clamp(nominal_pitch_deg_, cfg_.target_pitch_max_bwd, cfg_.target_pitch_max_fwd);
    final_target_pitch_deg_ = ramp_limit(prev_target_pitch_deg_, nominal_pitch_deg_, cfg_.target_pitch_rate_limit_dps, dt);
    final_target_pitch_deg_ = clamp(final_target_pitch_deg_, cfg_.target_pitch_max_bwd, cfg_.target_pitch_max_fwd);

    ref.target_speed_tps = target_speed_tps_;
    ref.target_displacement_turns = target_displacement_turns_;
    ref.nominal_pitch_deg = nominal_pitch_deg_;
    ref.constrained_pitch_deg = final_target_pitch_deg_;

    prev_target_speed_tps_ = target_speed_tps_;
    return ref;
}

ReferenceState BalanceController::apply_nmpc_if_valid(const EstimatedState& state, const ReferenceState& ref, double dt) {
    ReferenceState out = ref;
    debug_.nmpc_used = false;
    debug_.nmpc_cost = 0.0;
    debug_.nmpc_age_ms = 0.0;

    if (!cfg_.nmpc_enabled) return out;

    NmpcState nstate{state.pitch_deg, state.pitch_rate_dps, state.displacement_turns, state.velocity_tps};
    NmpcCommand ncmd;
    ncmd.target_speed_tps = ref.target_speed_tps;
    ncmd.target_displacement_turns = ref.target_displacement_turns;
    ncmd.nominal_pitch_deg = ref.nominal_pitch_deg;
    ncmd.balance_bias_deg = balance_point_deg_ + cfg_.pitch_offset + adaptive_trim_deg_;
    ncmd.previous_pitch_ref_deg = prev_target_pitch_deg_;
    nmpc_.publish(nstate, ncmd);

    NmpcResult r;
    if (!nmpc_.try_get_latest(r)) return out;

    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - r.publish_time).count();
    const bool fresh = age <= cfg_.stale_result_max_age_s;
    const bool same_context = (r.source_tick + 8 >= control_tick_) &&
        std::abs(state.pitch_deg - r.source_pitch_deg) <= cfg_.nmpc_state_mismatch_pitch_deg &&
        std::abs(state.velocity_tps - r.source_velocity_tps) <= cfg_.nmpc_state_mismatch_vel_tps &&
        std::abs(state.displacement_turns - r.source_displacement_turns) <= cfg_.nmpc_state_mismatch_disp_turns;

    debug_.nmpc_cost = r.cost;
    debug_.nmpc_age_ms = age * 1000.0;

    if (!(fresh && same_context)) return out;

    out.constrained_pitch_deg = ramp_limit(prev_target_pitch_deg_, r.target_pitch_deg, cfg_.target_pitch_rate_limit_dps, dt);
    out.constrained_pitch_deg = clamp(out.constrained_pitch_deg, cfg_.target_pitch_max_bwd, cfg_.target_pitch_max_fwd);
    out.target_speed_tps = ramp_limit(out.target_speed_tps, r.target_velocity_tps, cfg_.target_speed_rate_limit, dt);
    out.target_displacement_turns = r.target_displacement_turns;
    debug_.nmpc_used = true;
    return out;
}

void BalanceController::update_auto_tune(const EstimatedState& state, const ReferenceState& ref, double dt) {
    if (!cfg_.auto_tune_enable || dt <= 0.0) {
        auto_tune_scale_ = 1.0;
        auto_tune_pitch_rms_ *= 0.98;
        auto_tune_speed_rms_ *= 0.98;
        return;
    }

    const double theta_error = state.pitch_deg - ref.constrained_pitch_deg;
    const double speed_error = ref.target_speed_tps - state.velocity_tps;
    const bool safe_window = std::abs(theta_error) < 10.0 &&
                             std::abs(state.pitch_rate_dps) < 120.0 &&
                             std::abs(ref.constrained_pitch_deg - (balance_point_deg_ + cfg_.pitch_offset + adaptive_trim_deg_)) < 10.0;
    if (!safe_window) {
        auto_tune_scale_ = std::max(cfg_.auto_tune_min_scale, auto_tune_scale_ * (1.0 - 3.0 * cfg_.auto_tune_rate * dt));
        return;
    }

    const double alpha = std::clamp(2.0 * dt, 0.0, 1.0);
    auto_tune_pitch_rms_ = std::sqrt((1.0 - alpha) * auto_tune_pitch_rms_ * auto_tune_pitch_rms_ + alpha * theta_error * theta_error);
    auto_tune_speed_rms_ = std::sqrt((1.0 - alpha) * auto_tune_speed_rms_ * auto_tune_speed_rms_ + alpha * speed_error * speed_error);

    double request = 0.0;
    if (auto_tune_pitch_rms_ > cfg_.auto_tune_pitch_error_target_deg) request += 1.0;
    if (auto_tune_speed_rms_ > cfg_.auto_tune_speed_error_target_tps) request += 0.5;
    if (auto_tune_pitch_rms_ < 0.45 * cfg_.auto_tune_pitch_error_target_deg &&
        auto_tune_speed_rms_ < 0.45 * cfg_.auto_tune_speed_error_target_tps) request -= 0.5;

    auto_tune_scale_ += cfg_.auto_tune_rate * request * dt;
    auto_tune_scale_ = clamp(auto_tune_scale_, cfg_.auto_tune_min_scale, cfg_.auto_tune_max_scale);
}

double BalanceController::friction_compensation(double base_velocity, const ReferenceState& ref, const EstimatedState& state) const {
    if (cfg_.friction_comp_tps <= 0.0) return 0.0;

    const double drive_intent = std::abs(base_velocity) > cfg_.friction_comp_deadband_tps
        ? base_velocity
        : (std::abs(ref.target_speed_tps) > cfg_.friction_comp_deadband_tps ? ref.target_speed_tps : -state.velocity_tps);
    if (std::abs(drive_intent) <= cfg_.friction_comp_deadband_tps) return 0.0;

    const double fade = 1.0 - clamp(std::abs(state.velocity_tps) / cfg_.friction_comp_fade_tps, 0.0, 1.0);
    return std::copysign(cfg_.friction_comp_tps * fade, drive_intent);
}

MotorOutput BalanceController::compute_stabilizing_control(const EstimatedState& state, const ReferenceState& ref, const RemoteCommand& cmd, double dt) {
    MotorOutput output{};

    const double theta_error = state.pitch_deg - ref.constrained_pitch_deg;
    const double theta_d_error = state.pitch_rate_dps;
    const double x_error = state.displacement_turns - ref.target_displacement_turns;
    const double v_error = state.velocity_tps - ref.target_speed_tps;
    const double speed_abs = std::abs(state.velocity_tps);

    update_auto_tune(state, ref, dt);
    const double tuned_scale = cfg_.auto_tune_enable ? auto_tune_scale_ : 1.0;
    const double k_theta = tuned_scale * gain_schedule(cfg_.lqr_k_theta, speed_abs);
    const double k_theta_d = tuned_scale * gain_schedule(cfg_.lqr_k_theta_d, speed_abs);
    const double k_x = gain_schedule(cfg_.lqr_k_x, speed_abs);
    const double k_v = gain_schedule(cfg_.lqr_k_v, speed_abs);

    const bool output_was_saturated = std::abs(prev_base_velocity_cmd_tps_) >= 0.98 * cfg_.max_velocity;
    const bool integrator_would_worsen_sat = output_was_saturated &&
        ((prev_base_velocity_cmd_tps_ > 0.0 && theta_error > 0.0) ||
         (prev_base_velocity_cmd_tps_ < 0.0 && theta_error < 0.0));
    const bool allow_integrator = std::abs(theta_error) < 8.0 &&
                                  std::abs(state.pitch_rate_dps) < 80.0 &&
                                  std::abs(v_error) < cfg_.max_velocity &&
                                  !integrator_would_worsen_sat;
    if (allow_integrator) {
        theta_integral_ += theta_error * dt;
        theta_integral_ = clamp(theta_integral_, -cfg_.lqr_integral_limit, cfg_.lqr_integral_limit);
    } else {
        theta_integral_ *= 0.97;
    }

    double fb_theta = k_theta * theta_error;
    double fb_theta_d = k_theta_d * theta_d_error;

    // Hybrid cascaded control:
    // generate_reference() makes the target pitch, while this inner loop still
    // keeps a small direct speed/hold feedback path so throttle and position
    // hold have real authority.
    const double pos_hold_gate = position_hold_active_ ? 1.0 : 0.0;
    double fb_x = -pos_hold_gate * k_x * x_error;
    double fb_v = -k_v * v_error;
    const double i_term = cfg_.lqr_integral_k * theta_integral_;
    const double ff_term = cfg_.velocity_feedforward_gain * ref.target_speed_tps;
    if (cfg_.auto_lqr_enable >= 0.5 && auto_lqr_valid_) {
        // Auto-LQR gains are stored in physical-state order [x, v, theta, theta_dot].
        fb_x = -pos_hold_gate * auto_lqr_k_[0] * x_error;
        fb_v = -auto_lqr_k_[1] * v_error;
        fb_theta = auto_lqr_k_[2] * deg_to_rad(theta_error);
        fb_theta_d = auto_lqr_k_[3] * deg_to_rad(theta_d_error);
    }

    double base_velocity_unsat = ff_term + fb_theta + fb_theta_d + fb_x + fb_v + i_term;
    const double friction_ff = friction_compensation(base_velocity_unsat, ref, state);
    double base_velocity = base_velocity_unsat + friction_ff;
    base_velocity = clamp(base_velocity, -cfg_.max_velocity, cfg_.max_velocity);

    const double speed_mul = clamp(cmd.speed_limit, 0.0, 1.0);
    const double steering_soft_limit = std::max(0.25, 1.0 - 0.03 * std::abs(theta_error));
    const double yaw_diff = steering_filtered_ * cfg_.max_velocity * speed_mul * steering_soft_limit;
    output.left_velocity = clamp(base_velocity + yaw_diff, -cfg_.max_velocity, cfg_.max_velocity);
    output.right_velocity = clamp(base_velocity - yaw_diff, -cfg_.max_velocity, cfg_.max_velocity);

    debug_.target_pitch = ref.constrained_pitch_deg;
    debug_.filtered_pitch = state.pitch_deg;
    debug_.filtered_pitch_rate = state.pitch_rate_dps;
    debug_.pitch_error = theta_error;
    debug_.pitch_p = fb_theta;
    debug_.pitch_i = i_term;
    debug_.pitch_d = fb_theta_d;
    debug_.pitch_output = base_velocity;
    debug_.target_speed = ref.target_speed_tps;
    debug_.speed_estimate = state.velocity_tps;
    debug_.speed_error = ref.target_speed_tps - state.velocity_tps;
    debug_.gyro_y = state.pitch_rate_dps;
    debug_.gyro_y_dot = 0.0;
    debug_.pos_disp = displacement_turns_;
    debug_.observed_displacement = state.displacement_turns;
    debug_.filtered_velocity = state.velocity_tps;
    debug_.pos_target = ref.target_displacement_turns;
    debug_.pos_out = ref.constrained_pitch_deg - (balance_point_deg_ + cfg_.pitch_offset + adaptive_trim_deg_);
    debug_.balance_point = balance_point_deg_ + cfg_.pitch_offset;
    debug_.balance_trim = adaptive_trim_deg_;
    debug_.throttle_filtered = throttle_filtered_;
    debug_.steering_filtered = steering_filtered_;
    debug_.nominal_pitch = ref.nominal_pitch_deg;
    debug_.theta_error = theta_error;
    debug_.theta_d_error = theta_d_error;
    debug_.x_error = x_error;
    debug_.v_error = v_error;
    debug_.fb_theta = fb_theta;
    debug_.fb_theta_d = fb_theta_d;
    debug_.fb_x = fb_x;
    debug_.fb_v = fb_v;
    debug_.ff_term = ff_term + friction_ff;
    debug_.integral_term = i_term;
    debug_.base_cmd = base_velocity;
    prev_base_velocity_cmd_tps_ = base_velocity;
    debug_.cmd_left = output.left_velocity;
    debug_.cmd_right = output.right_velocity;

    return output;
}

void BalanceController::update_debug_common(const EstimatedState& state, const ReferenceState& ref) {
    (void)state;
    (void)ref;
}

MotorOutput BalanceController::update(const ImuData& imu, const RemoteCommand& cmd, double dt) {
    MotorOutput output{};
    if (dt <= 0.0 || dt > 0.1) return output;

    ++control_tick_;
    const EstimatedState state = estimate_state(imu, dt);
    if (!state.valid || !is_tilt_safe(state.pitch_deg)) {
        reset();
        return output;
    }

    ReferenceState ref = generate_reference(cmd, state, dt);
    ref = apply_nmpc_if_valid(state, ref, dt);
    final_target_pitch_deg_ = ref.constrained_pitch_deg;
    output = compute_stabilizing_control(state, ref, cmd, dt);
    prev_target_pitch_deg_ = ref.constrained_pitch_deg;
    return output;
}

}  // namespace robot
