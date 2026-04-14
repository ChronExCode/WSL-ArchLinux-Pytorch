#pragma once
// ═══════════════════════════════════════════════════════════════
//  Balance Controller — Cascaded PID + Complementary Filter
// ═══════════════════════════════════════════════════════════════

#include "types.h"

namespace robot {

class PID {
public:
    PID() = default;
    PID(double kp, double ki, double kd, double out_min, double out_max);
    double compute(double setpoint, double measurement, double dt);
    /// Compute using direct gyro rate for D-term (avoids noisy differentiation)
    double compute_with_gyro(double setpoint, double measurement, double gyro_rate, double dt);
    void reset();
    void reset_integral() { integral_ = 0; i_term_ = 0; }
    void set_gains(double kp, double ki, double kd);
    void set_limits(double min, double max);
    void set_d_alpha(double alpha) { d_alpha_ = alpha; }
    double get_p_term() const { return p_term_; }
    double get_i_term() const { return i_term_; }
    double get_d_term() const { return d_term_; }
    double get_error() const { return prev_error_; }
    double get_setpoint() const { return last_setpoint_; }
private:
    double kp_ = 0, ki_ = 0, kd_ = 0;
    double out_min_ = -100, out_max_ = 100;
    double integral_ = 0, prev_error_ = 0;
    double prev_measurement_ = 0;
    double d_filtered_ = 0;
    double d_alpha_ = 0.2;  // D-term filter coefficient (higher = faster response)
    bool first_run_ = true;
    double p_term_ = 0, i_term_ = 0, d_term_ = 0;
    double last_setpoint_ = 0;
};

class ComplementaryFilter {
public:
    ComplementaryFilter(double alpha = 0.98);
    double update(double acc_pitch_deg, double gyro_rate_dps, double dt);
    void reset();
    double get_pitch() const { return pitch_; }
private:
    double alpha_, pitch_ = 0;
    bool initialized_ = false;
};

class BalanceController {
public:
    BalanceController();
    explicit BalanceController(const Config& cfg);
    MotorOutput update(const ImuData& imu, const RemoteCommand& cmd, double dt);
    void reset();
    void set_config(const Config& cfg);
    void set_balance_point(double pitch_deg);  // call at ARM with current pitch
    void set_displacement(double turns);       // call with encoder displacement
    bool is_tilt_safe(double pitch_deg) const;
    double get_filtered_pitch() const { return filtered_pitch_; }
    double get_target_pitch() const { return target_pitch_; }
    double get_speed_estimate() const { return speed_estimate_; }
    double get_target_speed() const { return target_speed_; }
    const PID& get_pitch_pid() const { return pitch_pid_; }
    const PID& get_speed_pid() const { return speed_pid_; }
    const PID& get_yaw_pid() const { return yaw_pid_; }

    PidDebugData get_debug_data() const {
        PidDebugData d;
        d.target_pitch   = target_pitch_;
        d.filtered_pitch = filtered_pitch_;
        d.pitch_error    = pitch_pid_.get_error();
        d.pitch_p        = pitch_pid_.get_p_term();
        d.pitch_i        = pitch_pid_.get_i_term();
        d.pitch_d        = pitch_pid_.get_d_term();
        d.pitch_output   = d.pitch_p + d.pitch_i + d.pitch_d;
        d.target_speed   = target_speed_;
        d.speed_estimate = speed_estimate_;
        d.speed_error    = target_speed_ - speed_estimate_;
        d.gyro_y         = pitch_pid_.get_d_term(); // D-term as angular rate proxy
        d.gyro_y_dot     = 0;
        d.pos_disp       = displacement_;
        d.pos_target     = target_displacement_;
        d.pos_out        = pos_output_;
        return d;
    }
private:
    Config cfg_;
    PID pitch_pid_, speed_pid_, yaw_pid_, pos_pid_;
    ComplementaryFilter comp_filter_;
    double filtered_pitch_ = 0, target_pitch_ = 0;
    double speed_estimate_ = 0, target_speed_ = 0;
    double target_yaw_ = 0;        // locked heading (degrees, relative to startup)
    bool yaw_initialized_ = false;  // set on first update after arm
    double prev_base_velocity_ = 0; // for rate limiting
    double balance_point_ = 0;      // dynamic balance angle (degrees)
    double displacement_ = 0;       // encoder displacement (turns, from telemetry thread)
    double target_displacement_ = 0; // position target (turns, from throttle integration)
    double pos_output_ = 0;         // position PID output (degrees, for telemetry)
    double pitch_lpf_ = 0;         // low-pass filtered pitch angle
    bool lpf_initialized_ = false;  // first-sample init for LPF
    double process_throttle(double raw) const;
    double process_steering(double raw) const;
};

} // namespace robot
