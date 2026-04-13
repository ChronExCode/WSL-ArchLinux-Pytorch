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
    void reset();
    void set_gains(double kp, double ki, double kd);
    void set_limits(double min, double max);
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
    double d_filtered_ = 0;          // low-pass filtered D-term
    static constexpr double D_ALPHA = 0.05; // D-term filter coefficient (lower = smoother)
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
        d.target_pitch   = target_pitch_;    // target pitch angle (degrees)
        d.filtered_pitch = filtered_pitch_;  // actual pitch angle (degrees)
        d.pitch_error    = target_pitch_ - filtered_pitch_;
        d.pitch_p        = pitch_p_;
        d.pitch_i        = pitch_i_;
        d.pitch_d        = pitch_d_;
        d.pitch_output   = pitch_p_ + pitch_i_ + pitch_d_;
        d.target_speed   = target_speed_;
        d.speed_estimate = speed_estimate_;
        d.speed_error    = target_speed_ - speed_estimate_;
        d.gyro_y         = gyro_y_lpf_;      // filtered angular velocity
        d.gyro_y_dot     = gyro_y_dot_;
        return d;
    }
private:
    Config cfg_;
    PID pitch_pid_, speed_pid_, yaw_pid_;
    ComplementaryFilter comp_filter_;
    double filtered_pitch_ = 0, target_pitch_ = 0;
    double speed_estimate_ = 0, target_speed_ = 0, target_yaw_rate_ = 0;
    double target_yaw_ = 0;        // locked heading (degrees, relative to startup)
    bool yaw_initialized_ = false;  // set on first update after arm
    double prev_base_velocity_ = 0; // for rate limiting
    double pitch_integral_ = 0;     // integral of pitch error
    double balance_point_ = 0;      // dynamic balance angle (degrees)
    double displacement_ = 0;       // encoder displacement (turns, from telemetry thread)
    double pitch_p_ = 0, pitch_i_ = 0, pitch_d_ = 0;  // debug terms
    double prev_gyro_y_ = 0;       // previous gyro_y for angular acceleration
    double gyro_y_dot_ = 0;        // angular acceleration (deg/s²)
    double gyro_y_lpf_ = 0;        // low-pass filtered D-term
    double pitch_lpf_ = 0;         // low-pass filtered pitch angle
    bool lpf_initialized_ = false;  // first-sample init for LPF
    double integrated_velocity_ = 0;
    double process_throttle(double raw) const;
    double process_steering(double raw) const;
};

} // namespace robot
