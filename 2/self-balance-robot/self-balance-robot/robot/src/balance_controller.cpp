// ═══════════════════════════════════════════════════════════════
//  Balance Controller — Cascaded PID Implementation
// ═══════════════════════════════════════════════════════════════

#include "balance_controller.h"
#include <cmath>
#include <iostream>

namespace robot {

// ── PID ──────────────────────────────────────────────────────

PID::PID(double kp, double ki, double kd, double out_min, double out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

double PID::compute(double setpoint, double measurement, double dt) {
    if (dt <= 0) return 0;
    last_setpoint_ = setpoint;
    double error = setpoint - measurement;
    p_term_ = kp_ * error;

    // Integral with anti-windup: only integrate when output is not saturated
    double pre_output = p_term_ + i_term_ + d_term_;
    bool saturated = (pre_output >= out_max_ && error > 0) ||
                     (pre_output <= out_min_ && error < 0);
    if (!saturated) {
        integral_ += error * dt;
    }
    i_term_ = ki_ * integral_;
    i_term_ = clamp(i_term_, out_min_, out_max_);

    // Derivative on measurement (not error) to avoid setpoint kick,
    // with low-pass filter to reduce noise amplification
    if (first_run_) {
        d_term_ = 0;
        prev_measurement_ = measurement;
        d_filtered_ = 0;
        first_run_ = false;
    } else {
        double d_raw = -kd_ * (measurement - prev_measurement_) / dt;
        d_filtered_ = d_alpha_ * d_raw + (1.0 - d_alpha_) * d_filtered_;
        d_term_ = d_filtered_;
    }
    prev_measurement_ = measurement;
    prev_error_ = error;
    return clamp(p_term_ + i_term_ + d_term_, out_min_, out_max_);
}

double PID::compute_with_gyro(double setpoint, double measurement,
                               double gyro_rate, double dt) {
    // Same as compute() but uses gyro angular rate directly for D-term
    // instead of differentiating the measurement. Gyro has much better
    // signal-to-noise ratio than numerical differentiation.
    if (dt <= 0) return 0;
    last_setpoint_ = setpoint;
    double error = setpoint - measurement;
    p_term_ = kp_ * error;

    // Integral with anti-windup
    double pre_output = p_term_ + i_term_ + d_term_;
    bool saturated = (pre_output >= out_max_ && error > 0) ||
                     (pre_output <= out_min_ && error < 0);
    if (!saturated) {
        integral_ += error * dt;
    }
    i_term_ = ki_ * integral_;
    i_term_ = clamp(i_term_, out_min_, out_max_);

    // D-term from gyro rate (negative sign: gyro_rate > 0 means pitch increasing,
    // which means measurement moving away from setpoint if setpoint < measurement)
    double d_raw = -kd_ * gyro_rate;
    d_filtered_ = d_alpha_ * d_raw + (1.0 - d_alpha_) * d_filtered_;
    d_term_ = d_filtered_;

    prev_measurement_ = measurement;
    prev_error_ = error;
    first_run_ = false;
    return clamp(p_term_ + i_term_ + d_term_, out_min_, out_max_);
}

void PID::reset() {
    integral_ = 0; prev_error_ = 0; prev_measurement_ = 0;
    d_filtered_ = 0; first_run_ = true;
    p_term_ = i_term_ = d_term_ = 0; last_setpoint_ = 0;
}
void PID::set_gains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
void PID::set_limits(double min, double max) { out_min_ = min; out_max_ = max; }

// ── Complementary Filter ─────────────────────────────────────

ComplementaryFilter::ComplementaryFilter(double alpha) : alpha_(alpha) {}

double ComplementaryFilter::update(double acc_pitch_deg, double gyro_rate_dps, double dt) {
    if (!initialized_) { pitch_ = acc_pitch_deg; initialized_ = true; return pitch_; }
    pitch_ = alpha_ * (pitch_ + gyro_rate_dps * dt) + (1.0 - alpha_) * acc_pitch_deg;
    return pitch_;
}

void ComplementaryFilter::reset() { pitch_ = 0; initialized_ = false; }

// ── Balance Controller ───────────────────────────────────────

BalanceController::BalanceController() : BalanceController(Config{}) {}

BalanceController::BalanceController(const Config& cfg) : cfg_(cfg) {
    pitch_pid_ = PID(cfg_.pitch_kp, cfg_.pitch_ki, cfg_.pitch_kd, -50.0, 50.0);
    pitch_pid_.set_d_alpha(cfg_.pitch_d_alpha);
    speed_pid_ = PID(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd, -15.0, 15.0);
    yaw_pid_   = PID(cfg_.yaw_kp, cfg_.yaw_ki, cfg_.yaw_kd, -50.0, 50.0);
    pos_pid_   = PID(cfg_.pos_kp, cfg_.pos_ki, cfg_.pos_kd, -5.0, 5.0);
    comp_filter_ = ComplementaryFilter(cfg_.comp_filter_alpha);
}

void BalanceController::set_config(const Config& cfg) {
    cfg_ = cfg;
    pitch_pid_.set_gains(cfg_.pitch_kp, cfg_.pitch_ki, cfg_.pitch_kd);
    pitch_pid_.set_d_alpha(cfg_.pitch_d_alpha);
    speed_pid_.set_gains(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd);
    yaw_pid_.set_gains(cfg_.yaw_kp, cfg_.yaw_ki, cfg_.yaw_kd);
    pos_pid_.set_gains(cfg_.pos_kp, cfg_.pos_ki, cfg_.pos_kd);
    comp_filter_ = ComplementaryFilter(cfg_.comp_filter_alpha);
}

void BalanceController::reset() {
    pitch_pid_.reset(); speed_pid_.reset(); yaw_pid_.reset();
    pos_pid_.reset(); comp_filter_.reset();
    filtered_pitch_ = target_pitch_ = speed_estimate_ = 0;
    target_speed_ = 0;
    target_yaw_ = 0;
    yaw_initialized_ = false;
    prev_base_velocity_ = 0;
    target_displacement_ = 0;
    pos_output_ = 0;
    // NOTE: balance_point_ is NOT reset — it persists across arm/disarm
    // so the robot remembers where it balanced last time.
    pitch_lpf_ = 0;
    lpf_initialized_ = false;
}

void BalanceController::set_balance_point(double pitch_deg) {
    balance_point_ = pitch_deg;
    displacement_ = 0;
    target_displacement_ = 0;
    pos_pid_.reset();
    pos_output_ = 0;
    std::cout << "[Balance] Balance point set to " << pitch_deg << "°\n";
}

void BalanceController::set_displacement(double turns) {
    displacement_ = turns;
}

bool BalanceController::is_tilt_safe(double pitch_deg) const {
    return std::abs(pitch_deg) < cfg_.max_tilt_angle;
}

double BalanceController::process_throttle(double raw) const { return apply_deadband(raw, cfg_.joystick_deadband); }
double BalanceController::process_steering(double raw) const { return apply_deadband(raw, cfg_.joystick_deadband); }

MotorOutput BalanceController::update(const ImuData& imu, const RemoteCommand& cmd, double dt) {
    MotorOutput output;
    if (dt <= 0 || dt > 0.1) return output;

    // ── Pitch angle estimation ───────────────────────────────
    // Blend FC attitude (70%) with local complementary filter (30%)
    double acc_pitch = rad_to_deg(std::atan2(-imu.acc_x,
        std::sqrt(imu.acc_y * imu.acc_y + imu.acc_z * imu.acc_z)));

    double actual_pitch;
    if (imu.valid && std::abs(imu.pitch) < 90.0) {
        double our_pitch = comp_filter_.update(acc_pitch, imu.gyro_y, dt);
        actual_pitch = 0.7 * imu.pitch + 0.3 * our_pitch;
    } else {
        actual_pitch = comp_filter_.update(acc_pitch, imu.gyro_y, dt);
    }

    if (!is_tilt_safe(actual_pitch)) { reset(); return output; }

    // Low-pass filter actual_pitch to absorb IMU data glitches
    static constexpr double PITCH_LPF_ALPHA = 0.5;
    if (!lpf_initialized_) {
        pitch_lpf_ = actual_pitch;
        lpf_initialized_ = true;
    } else {
        pitch_lpf_ = PITCH_LPF_ALPHA * actual_pitch + (1.0 - PITCH_LPF_ALPHA) * pitch_lpf_;
    }
    actual_pitch = pitch_lpf_;

    double throttle = process_throttle(cmd.throttle);
    double steering = process_steering(cmd.steering);
    double speed_mul = clamp(cmd.speed_limit, 0.0, 1.0);

    // ── Position loop: displacement → pitch correction ───────
    //
    // Architecture:
    //   Position loop (outer): encoder displacement → target_pitch correction
    //     Keeps robot at target position when throttle is zero.
    //     When throttle is active, target position follows throttle.
    //   Pitch PID (inner, 200Hz): pitch error → motor velocity
    //     Full PID with P, I (steady-state correction), D (damping).
    //     D-term uses measurement derivative to avoid setpoint kick.
    //
    // Throttle only acts through target_displacement — no direct pitch offset.
    // This prevents double-counting (position loop + direct throttle).

    if (std::abs(throttle) > 0.01) {
        // Moving: integrate throttle into target position
        target_displacement_ += throttle * cfg_.max_velocity * speed_mul * dt;
        // Only clear I-term during movement (not D-term state)
        pos_pid_.reset_integral();
    }

    // Position PID always runs
    pos_output_ = pos_pid_.compute(target_displacement_, displacement_, dt);
    pos_output_ = clamp(pos_output_, cfg_.pos_out_max_bwd, cfg_.pos_out_max_fwd);

    // Target pitch: balance_point + offset + position correction
    double target_pitch = balance_point_ + cfg_.pitch_offset + pos_output_;

    // Asymmetric clamp on final target pitch
    target_pitch = clamp(target_pitch, cfg_.target_pitch_max_bwd, cfg_.target_pitch_max_fwd);

    // ── Inner loop: pitch PID → motor velocity ───────────────
    // Uses compute_with_gyro: P and I from pitch angle, D from gyro rate.
    // Gyro rate has much better SNR than numerical differentiation of pitch.
    // Inner loop: pitch PID outputs negative when pitch > target (forward lean).
    // But we need positive velocity to drive wheels forward to catch the fall.
    // Negate to get correct direction: forward lean → positive velocity → wheels forward.
    double base_velocity = -pitch_pid_.compute_with_gyro(
        target_pitch, actual_pitch, imu.gyro_y, dt);
    base_velocity = clamp(base_velocity, -cfg_.max_velocity, cfg_.max_velocity);

    // Store for debug telemetry
    target_pitch_ = target_pitch;
    filtered_pitch_ = actual_pitch;
    target_speed_ = pos_output_;
    speed_estimate_ = displacement_;

    // ── Yaw control ──────────────────────────────────────────
    // Disabled during balance tuning
    double yaw_diff = 0;

    prev_base_velocity_ = base_velocity;

    // Motor output: both use unified sign convention (positive = forward)
    // Mirror inversion for axis1 is done in ODrive IO layer (set_velocities/get_position)
    output.left_velocity  = clamp( (base_velocity + yaw_diff), -cfg_.max_velocity, cfg_.max_velocity);
    output.right_velocity = clamp( (base_velocity - yaw_diff), -cfg_.max_velocity, cfg_.max_velocity);

    return output;
}

} // namespace robot
