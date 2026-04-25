#include "ekf.h"

#include <algorithm>
#include <cmath>

namespace robot {
namespace {
constexpr double kFiniteDiff = 1e-4;

Mat4 add4(const Mat4& a, const Mat4& b) {
    Mat4 c = zero4();
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) c[i][j] = a[i][j] + b[i][j];
    return c;
}

}

void EkfStateEstimator::reset() {
    x_ = {};
    P_ = eye4();
    initialized_ = false;
}

void EkfStateEstimator::initialize(double position_turns, double velocity_tps, double pitch_deg, double pitch_rate_dps) {
    x_ = {position_turns, velocity_tps, pitch_deg, pitch_rate_dps};
    P_ = zero4();
    P_[0][0] = std::max(1e-6, cfg_.ekf_init_pos_var);
    P_[1][1] = std::max(1e-6, cfg_.ekf_init_vel_var);
    P_[2][2] = std::max(1e-6, cfg_.ekf_init_pitch_var);
    P_[3][3] = std::max(1e-6, cfg_.ekf_init_pitch_rate_var);
    initialized_ = true;
}

EstimatedState EkfStateEstimator::get_state() const {
    EstimatedState s{};
    s.displacement_turns = x_[0];
    s.velocity_tps = x_[1];
    s.pitch_deg = x_[2];
    s.pitch_rate_dps = x_[3];
    s.valid = initialized_;
    return s;
}

EkfStateEstimator::Vec4 EkfStateEstimator::process_model(const Vec4& x, double control_velocity_cmd_tps, double dt) const {
    Mat4 A = zero4();
    Vec4 B{};
    build_velocity_model(cfg_, dt, x[1], x[2], cfg_.lqr_speed_gain_scale, cfg_.nmpc_ltv_pitch_scale, A, B);
    Vec4 next = mul4v(A, x);
    for (int i = 0; i < 4; ++i) next[i] += B[i] * control_velocity_cmd_tps;
    return next;
}

EkfStateEstimator::Mat4 EkfStateEstimator::process_jacobian(const Vec4& x, double control_velocity_cmd_tps, double dt) const {
    Mat4 F = zero4();
    const Vec4 f0 = process_model(x, control_velocity_cmd_tps, dt);
    for (int j = 0; j < 4; ++j) {
        Vec4 xp = x;
        xp[j] += kFiniteDiff;
        const Vec4 fp = process_model(xp, control_velocity_cmd_tps, dt);
        for (int i = 0; i < 4; ++i) F[i][j] = (fp[i] - f0[i]) / kFiniteDiff;
    }
    return F;
}

EkfStateEstimator::Mat4 EkfStateEstimator::process_noise(double dt) const {
    Mat4 Q = zero4();
    Q[0][0] = std::max(1e-9, cfg_.ekf_q_position) * dt;
    Q[1][1] = std::max(1e-9, cfg_.ekf_q_velocity) * dt;
    Q[2][2] = std::max(1e-9, cfg_.ekf_q_pitch) * dt;
    Q[3][3] = std::max(1e-9, cfg_.ekf_q_pitch_rate) * dt;
    return Q;
}

void EkfStateEstimator::predict(double control_velocity_cmd_tps, double dt) {
    if (!initialized_) return;
    const Mat4 F = process_jacobian(x_, control_velocity_cmd_tps, dt);
    x_ = process_model(x_, control_velocity_cmd_tps, dt);
    P_ = add4(mul4(F, mul4(P_, transpose4(F))), process_noise(dt));
}

void EkfStateEstimator::scalar_update(int idx, double measurement, double variance) {
    if (!initialized_) return;
    variance = std::max(1e-9, variance);
    const double innovation = measurement - x_[idx];
    const double S = P_[idx][idx] + variance;
    if (S <= 1e-12) return;
    std::array<double,4> K{};
    for (int i = 0; i < 4; ++i) K[i] = P_[i][idx] / S;
    for (int i = 0; i < 4; ++i) x_[i] += K[i] * innovation;
    Mat4 Pnew = P_;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Pnew[i][j] -= K[i] * P_[idx][j];
        }
    }
    P_ = Pnew;
}

void EkfStateEstimator::update_position(double z_turns) { scalar_update(0, z_turns, cfg_.ekf_r_position); }
void EkfStateEstimator::update_velocity(double z_tps) { scalar_update(1, z_tps, cfg_.ekf_r_velocity); }
void EkfStateEstimator::update_pitch(double z_deg) { scalar_update(2, z_deg, cfg_.ekf_r_pitch); }
void EkfStateEstimator::update_pitch_rate(double z_dps) { scalar_update(3, z_dps, cfg_.ekf_r_pitch_rate); }

} // namespace robot
