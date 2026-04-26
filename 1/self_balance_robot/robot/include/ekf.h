#pragma once

#include "control_model.h"
#include "types.h"
#include <array>

namespace robot {

class EkfStateEstimator {
public:
    EkfStateEstimator() = default;
    explicit EkfStateEstimator(const Config& cfg) : cfg_(cfg) {}

    void set_config(const Config& cfg) { cfg_ = cfg; }
    void reset();
    void initialize(double position_turns, double velocity_tps, double pitch_deg, double pitch_rate_dps);
    void predict(double control_velocity_cmd_tps, double dt);
    void update_position(double z_turns);
    void update_velocity(double z_tps);
    void update_pitch(double z_deg);
    void update_pitch_rate(double z_dps);

    bool initialized() const { return initialized_; }
    EstimatedState get_state() const;

private:
    using Vec4 = robot::Vec4;
    using Mat4 = robot::Mat4;

    Config cfg_{};
    Vec4 x_{}; // [position turns, velocity turns/s, pitch deg, pitch_rate deg/s]
    Mat4 P_ = eye4();
    bool initialized_ = false;

    Vec4 process_model(const Vec4& x, double control_velocity_cmd_tps, double dt) const;
    Mat4 process_jacobian(const Vec4& x, double control_velocity_cmd_tps, double dt) const;
    Mat4 process_noise(double dt) const;
    void scalar_update(int state_index, double measurement, double variance);
};

} // namespace robot
