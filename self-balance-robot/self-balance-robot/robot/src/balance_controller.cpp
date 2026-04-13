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
        d_filtered_ = D_ALPHA * d_raw + (1.0 - D_ALPHA) * d_filtered_;
        d_term_ = d_filtered_;
    }
    prev_measurement_ = measurement;
    prev_error_ = error;
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
    // Pitch PID output range is wider than max_velocity to avoid
    // constant saturation/oscillation. Final motor clamp handles limits.
    pitch_pid_ = PID(cfg_.pitch_kp, cfg_.pitch_ki, cfg_.pitch_kd, -50.0, 50.0);
    speed_pid_ = PID(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd, -15.0, 15.0);
    yaw_pid_   = PID(cfg_.yaw_kp, cfg_.yaw_ki, cfg_.yaw_kd, -50.0, 50.0);
    comp_filter_ = ComplementaryFilter(cfg_.comp_filter_alpha);
}

void BalanceController::set_config(const Config& cfg) {
    cfg_ = cfg;
    pitch_pid_.set_gains(cfg_.pitch_kp, cfg_.pitch_ki, cfg_.pitch_kd);
    // Don't set pitch PID limits to max_velocity — keep wide range
    speed_pid_.set_gains(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd);
    yaw_pid_.set_gains(cfg_.yaw_kp, cfg_.yaw_ki, cfg_.yaw_kd);
    comp_filter_ = ComplementaryFilter(cfg_.comp_filter_alpha);
}

void BalanceController::reset() {
    pitch_pid_.reset(); speed_pid_.reset(); yaw_pid_.reset(); comp_filter_.reset();
    filtered_pitch_ = target_pitch_ = speed_estimate_ = integrated_velocity_ = 0;
    target_yaw_ = 0;
    yaw_initialized_ = false;
    prev_base_velocity_ = 0;
    pitch_integral_ = 0;
    // NOTE: balance_point_ is NOT reset — it persists across arm/disarm
    // so the robot remembers where it balanced last time.
    pitch_p_ = pitch_i_ = pitch_d_ = 0;
    prev_gyro_y_ = 0;
    gyro_y_dot_ = 0;
    gyro_y_lpf_ = 0;
    pitch_lpf_ = 0;
    lpf_initialized_ = false;
}

void BalanceController::set_balance_point(double pitch_deg) {
    balance_point_ = pitch_deg;
    pitch_integral_ = 0;
    displacement_ = 0;
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
    // alpha=0.5 → fast enough to track real tilt, but smooths 1-frame jumps
    static constexpr double PITCH_LPF_ALPHA = 0.5;

    // Low-pass filter for D-term
    static constexpr double D_LPF_ALPHA = 0.3;
    if (!lpf_initialized_) {
        pitch_lpf_ = actual_pitch;  // seed with first reading
        gyro_y_lpf_ = 0;
        prev_gyro_y_ = 0;  // stores previous pitch_error for derivative
        lpf_initialized_ = true;
    } else {
        pitch_lpf_ = PITCH_LPF_ALPHA * actual_pitch + (1.0 - PITCH_LPF_ALPHA) * pitch_lpf_;
    }
    actual_pitch = pitch_lpf_;  // use filtered value for everything below

    double throttle = process_throttle(cmd.throttle);
    double steering = process_steering(cmd.steering);
    double speed_mul = clamp(cmd.speed_limit, 0.0, 1.0);

    // ── Cascade PID Balance Control ──────────────────────────
    //
    // Architecture:
    //   Outer loop: integrates motor output to estimate drift,
    //     adjusts target_pitch to lean back toward starting position.
    //   Inner loop (200Hz): pitch PD keeps robot at target angle.
    //     P: pitch angle error → motor velocity
    //     D: pitch rate of change → damping

    // ── Dynamic balance point estimation ──────────────────
    // Core idea: if the motor is consistently outputting in one direction,
    // the balance point is wrong. Motor output IS the error signal.
    //
    // - Motor consistently positive → robot moving forward → leaning too far forward
    //   → balance_point should decrease (lean back)
    // - Motor consistently negative → robot moving backward → leaning too far back
    //   → balance_point should increase (lean forward)
    //
    // We low-pass filter the motor output and use it to adjust balance_point_.
    // Speed: ~0.5°/s correction per 1.0 motor output unit.
    // This finds the true balance angle in ~2-3 seconds.
    //
    // speed_kp controls the correction rate (higher = faster adaptation,
    // but too high causes oscillation around the balance point).

    // ── Dynamic balance point correction ──────────────────
    // Use REAL encoder displacement to adjust balance_point_.
    // If the robot has drifted from its ARM position, the balance angle is wrong.
    //
    // displacement_ > 0 → robot moved forward → balance_point too far forward
    //   → decrease balance_point (lean back to return)
    //
    // speed_kp controls how aggressively to correct:
    //   0.5 = shift balance by 0.5° per turn of displacement
    //   This is a proportional controller on position.
    //
    // Ki: slow integral correction for fine CoG offset tuning.

    // Position-based correction (proportional to how far we've drifted)
    double pos_correction = cfg_.speed_kp * displacement_;
    pos_correction = clamp(pos_correction, -5.0, 5.0);

    // Ki: slow integral of pitch error
    double pitch_error_raw = (balance_point_ + cfg_.pitch_offset) - actual_pitch;
    double ki_correction = cfg_.pitch_ki * pitch_error_raw * dt;

    // Rate-limit balance_point changes to max 2°/s
    double total_bp_change = ki_correction;
    double max_bp_rate = 2.0 * dt;
    total_bp_change = clamp(total_bp_change, -max_bp_rate, max_bp_rate);
    balance_point_ += total_bp_change;
    balance_point_ = clamp(balance_point_, -15.0, 15.0);

    // Target pitch: balance_point + offset - position correction + throttle
    double target_pitch = balance_point_ + cfg_.pitch_offset
                        - pos_correction
                        + throttle * 5.0 * speed_mul;

    // ── Inner loop: pitch PD → motor velocity ────────────────
    double pitch_error = target_pitch - actual_pitch;

    // P: proportional to pitch angle error
    double p_term = cfg_.pitch_kp * pitch_error;

    // D: derivative of pitch error, low-pass filtered
    // Using error derivative (not pitch derivative) guarantees correct sign:
    //   error shrinking → d_term positive → helps P continue correcting
    //   error growing   → d_term negative → opposes the growth (damping)
    // This is immune to gyro sign convention issues.
    double error_rate = (pitch_error - prev_gyro_y_) / dt;
    // Clamp to reject glitches from IMU data jumps
    error_rate = clamp(error_rate, -200.0, 200.0);
    gyro_y_lpf_ = D_LPF_ALPHA * error_rate + (1.0 - D_LPF_ALPHA) * gyro_y_lpf_;
    double d_term = cfg_.pitch_kd * gyro_y_lpf_;  // positive: damp error growth
    prev_gyro_y_ = pitch_error;  // store for next frame

    double base_velocity = clamp(p_term + d_term,
                                 -cfg_.max_velocity, cfg_.max_velocity);

    // For telemetry
    gyro_y_dot_ = error_rate;

    // Store for debug telemetry
    target_pitch_ = target_pitch;
    filtered_pitch_ = actual_pitch;
    target_speed_ = pos_correction;     // show position correction in telemetry
    speed_estimate_ = displacement_;    // show real encoder displacement
    pitch_p_ = p_term;
    pitch_i_ = balance_point_;          // show dynamic balance point
    pitch_d_ = d_term;

    // ── Yaw control ──────────────────────────────────────────
    // Disabled during balance tuning
    double yaw_diff = 0;

    prev_base_velocity_ = base_velocity;

    output.left_velocity  = clamp(base_velocity + yaw_diff, -cfg_.max_velocity, cfg_.max_velocity);
    output.right_velocity = clamp(-(base_velocity - yaw_diff), -cfg_.max_velocity, cfg_.max_velocity);

    return output;
}

} // namespace robot
