#pragma once

#include "ekf.h"
#include "nmpc_controller.h"
#include "types.h"

namespace robot {

class PID {
public:
    PID() = default;
    PID(double kp, double ki, double kd, double out_min, double out_max);

    double compute(double setpoint, double measurement, double dt);
    double compute_with_gyro(double setpoint, double measurement, double gyro_rate, double dt);
    void reset();
    void reset_integral() { integral_ = 0.0; i_term_ = 0.0; }
    void set_gains(double kp, double ki, double kd);
    void set_limits(double min, double max);
    void set_d_alpha(double alpha) { d_alpha_ = alpha; }

    double get_p_term() const { return p_term_; }
    double get_i_term() const { return i_term_; }
    double get_d_term() const { return d_term_; }
    double get_error() const { return prev_error_; }
    double get_setpoint() const { return last_setpoint_; }

private:
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double out_min_ = -100.0;
    double out_max_ = 100.0;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double prev_measurement_ = 0.0;
    double d_filtered_ = 0.0;
    double d_alpha_ = 0.2;
    bool first_run_ = true;
    double p_term_ = 0.0;
    double i_term_ = 0.0;
    double d_term_ = 0.0;
    double last_setpoint_ = 0.0;
};

class ComplementaryFilter {
public:
    explicit ComplementaryFilter(double alpha = 0.98);
    double update(double acc_pitch_deg, double gyro_rate_dps, double dt);
    void reset();
    double get_pitch() const { return pitch_; }

private:
    double alpha_;
    double pitch_ = 0.0;
    bool initialized_ = false;
};

class BalanceController {
public:
    BalanceController();
    explicit BalanceController(const Config& cfg);
    ~BalanceController();

    BalanceController(const BalanceController&) = delete;
    BalanceController& operator=(const BalanceController&) = delete;
    BalanceController(BalanceController&&) = delete;
    BalanceController& operator=(BalanceController&&) = delete;

    MotorOutput update(const ImuData& imu, const RemoteCommand& cmd, double dt);
    void reset();
    void set_config(const Config& cfg);
    void set_balance_point(double pitch_deg);
    void set_displacement(double turns);
    void set_wheel_velocity(double turns_per_sec);
    bool is_tilt_safe(double pitch_deg) const;

    double get_filtered_pitch() const { return filtered_pitch_deg_; }
    double get_target_pitch() const { return final_target_pitch_deg_; }
    double get_speed_estimate() const { return filtered_velocity_tps_; }
    double get_target_speed() const { return target_speed_tps_; }
    const PID& get_pitch_pid() const { return legacy_pitch_pid_; }
    const PID& get_speed_pid() const { return legacy_speed_pid_; }
    const PID& get_yaw_pid() const { return legacy_yaw_pid_; }
    PidDebugData get_debug_data() const { return debug_; }

private:
    Config cfg_{};
    PID legacy_pitch_pid_{};
    PID legacy_speed_pid_{};
    PID legacy_yaw_pid_{};
    PID legacy_pos_pid_{};
    ComplementaryFilter comp_filter_{};
    EkfStateEstimator ekf_{};
    NmpcController nmpc_{};

    double balance_point_deg_ = 0.0;
    double adaptive_trim_deg_ = 0.0;

    double displacement_turns_ = 0.0;
    double wheel_velocity_raw_tps_ = 0.0;

    double filtered_pitch_deg_ = 0.0;
    double filtered_pitch_rate_dps_ = 0.0;
    double observed_displacement_turns_ = 0.0;
    double filtered_velocity_tps_ = 0.0;
    bool state_initialized_ = false;

    double throttle_filtered_ = 0.0;
    double steering_filtered_ = 0.0;

    double target_speed_tps_ = 0.0;
    double target_displacement_turns_ = 0.0;
    double nominal_pitch_deg_ = 0.0;
    double final_target_pitch_deg_ = 0.0;
    double prev_target_speed_tps_ = 0.0;
    double prev_target_pitch_deg_ = 0.0;
    double theta_integral_ = 0.0;
    double prev_base_velocity_cmd_tps_ = 0.0;

    uint64_t control_tick_ = 0;
    PidDebugData debug_{};
    double auto_lqr_k_[4] = {0.0,0.0,0.0,0.0};
    bool auto_lqr_valid_ = false;

    double process_throttle(double raw) const;
    double process_steering(double raw) const;
    double gain_schedule(double base_gain, double speed_abs) const;
    void synthesize_auto_lqr();
    EstimatedState estimate_state(const ImuData& imu, double dt);
    ReferenceState generate_reference(const RemoteCommand& cmd, const EstimatedState& state, double dt);
    ReferenceState apply_nmpc_if_valid(const EstimatedState& state, const ReferenceState& ref, double dt);
    MotorOutput compute_stabilizing_control(const EstimatedState& state, const ReferenceState& ref, const RemoteCommand& cmd, double dt);
    void update_debug_common(const EstimatedState& state, const ReferenceState& ref);
};

}  // namespace robot
