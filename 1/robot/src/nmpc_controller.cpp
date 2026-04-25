#include "nmpc_controller.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "control_model.h"

namespace robot {
namespace {

using Vec5 = std::array<double,5>;
using Mat5 = std::array<std::array<double,5>,5>;

struct TrackingReference {
    double pitch_deg = 0.0;
    double disp_turns = 0.0;
    double vel_tps = 0.0;
};

struct ErrorStepModel {
    Mat4 A{};
    Vec4 B{};
    Vec4 d{};          // explicit reference-state mismatch term
    Vec4 ref_k{};
    Vec4 ref_next{};
    double u_ref_deg = 0.0;
    double u_ref_next_deg = 0.0;
};

struct AugmentedStepModel {
    Mat5 A{};
    Vec5 B{};
    Vec5 c{};
    double u_ref_deg = 0.0;
    double u_ref_next_deg = 0.0;
};

struct BeamNode {
    Vec5 z{};          // augmented error-state [e_x, e_v, e_theta, e_rate, delta_u_prev]^T
    double first_u_deg = 0.0;
    double cost = std::numeric_limits<double>::max();
    double score = std::numeric_limits<double>::max();
    bool has_first = false;
};

inline Mat5 zero5(){ return {{{{0,0,0,0,0}},{{0,0,0,0,0}},{{0,0,0,0,0}},{{0,0,0,0,0}},{{0,0,0,0,0}}}}; }
inline Mat5 transpose5(const Mat5& a){ Mat5 t=zero5(); for(int i=0;i<5;++i) for(int j=0;j<5;++j) t[j][i]=a[i][j]; return t; }
inline Mat5 mul5(const Mat5& a,const Mat5& b){ Mat5 c=zero5(); for(int i=0;i<5;++i) for(int k=0;k<5;++k) for(int j=0;j<5;++j) c[i][j]+=a[i][k]*b[k][j]; return c; }
inline Vec5 mul5v(const Mat5& a,const Vec5& x){ Vec5 y{}; for(int i=0;i<5;++i) for(int j=0;j<5;++j) y[i]+=a[i][j]*x[j]; return y; }
inline Vec5 mulPB5(const Mat5& P,const Vec5& B){ Vec5 r{}; for(int i=0;i<5;++i) for(int j=0;j<5;++j) r[i]+=P[i][j]*B[j]; return r; }
inline Vec5 BtP5(const Vec5& B,const Mat5& P){ Vec5 r{}; for(int j=0;j<5;++j) for(int i=0;i<5;++i) r[j]+=B[i]*P[i][j]; return r; }
inline double dot5(const Vec5& a,const Vec5& b){ double s=0; for(int i=0;i<5;++i) s+=a[i]*b[i]; return s; }
inline Mat5 make_aug_weight(const Config& cfg,double terminal_scale=1.0){ Mat5 Q=zero5(); Q[0][0]=cfg.nmpc_w_x*terminal_scale; Q[1][1]=cfg.nmpc_w_v*terminal_scale; Q[2][2]=cfg.nmpc_w_theta*terminal_scale; Q[3][3]=cfg.nmpc_w_theta_rate*terminal_scale; Q[4][4]=cfg.nmpc_w_u*terminal_scale; return Q; }

inline double clamp_pitch_candidate(double value, const Config& cfg) {
    return clamp(value, cfg.nmpc_pitch_min_deg, cfg.nmpc_pitch_max_deg);
}

inline TrackingReference make_reference_at(const Config& cfg, const NmpcProblem& p, int step, int horizon) {
    const double denom = std::max(1, horizon - 1);
    const double alpha = double(step) / double(denom);
    const double blend = clamp(cfg.nmpc_reference_velocity_blend, 0.0, 1.0);
    const double eased = alpha * (1.0 - 0.5 * blend) + (1.0 - std::cos(alpha * M_PI)) * 0.25 * blend;

    const double vel_ref = p.state.wheel_velocity_tps +
        (p.command.target_speed_tps - p.state.wheel_velocity_tps) * eased;

    const double max_step = cfg.nmpc_pitch_slew_dps * p.dt * double(step + 1);
    const double pitch_ref = clamp(
        p.command.nominal_pitch_deg,
        p.command.previous_pitch_ref_deg - max_step,
        p.command.previous_pitch_ref_deg + max_step);

    double disp_ref = p.command.target_displacement_turns;
    disp_ref += std::max(0.0, cfg.nmpc_reference_position_preview_gain) * vel_ref * p.dt * double(step + 1);
    return {pitch_ref, disp_ref, vel_ref};
}

inline Vec4 ref_to_vec(const TrackingReference& r) {
    return {r.disp_turns, r.vel_tps, r.pitch_deg, 0.0};
}
inline double quad_cost4(const Vec4& x, const Mat4& P) {
    return dot4(x, mul4v(P, x));
}
inline double quad_cost5(const Vec5& x, const Mat5& P) {
    return dot5(x, mul5v(P, x));
}

inline ErrorStepModel build_error_step_model(const Config& cfg,
                                             const NmpcProblem& p,
                                             const TrackingReference& ref_k,
                                             const TrackingReference& ref_next) {
    ErrorStepModel m;
    m.ref_k = ref_to_vec(ref_k);
    m.ref_next = ref_to_vec(ref_next);
    m.u_ref_deg = ref_k.pitch_deg;
    m.u_ref_next_deg = ref_next.pitch_deg;

    if (cfg.nmpc_use_physical_model >= 0.5) {
        build_velocity_model(cfg, p.dt, ref_k.vel_tps, ref_k.pitch_deg,
                             cfg.nmpc_ltv_velocity_scale,
                             cfg.nmpc_ltv_pitch_scale,
                             m.A, m.B);
    } else {
        const double dt = p.dt;
        const double max_vel = std::max(0.1, cfg.max_velocity);
        const double pitch_span = std::max(1.0, 0.5 * (cfg.target_pitch_max_fwd - cfg.target_pitch_max_bwd));
        const double vel_scale = 1.0 + std::max(0.0, cfg.nmpc_ltv_velocity_scale) *
            std::min(2.0, std::abs(ref_k.vel_tps) / max_vel);
        const double pitch_scale = 1.0 + std::max(0.0, cfg.nmpc_ltv_pitch_scale) *
            std::min(2.0, std::abs(ref_k.pitch_deg - p.command.balance_bias_deg) / pitch_span);
        const double a_u = cfg.nmpc_model_accel_gain * vel_scale;
        const double d_v = cfg.nmpc_model_accel_damping;
        const double k_s = cfg.nmpc_model_theta_stiffness * pitch_scale;
        const double k_d = cfg.nmpc_model_theta_damping;
        const double cpl = cfg.nmpc_model_couple_gain;
        const double r_vel = cpl * d_v * dt;
        const double r_pitch = -(k_s + cpl * a_u) * dt;
        const double r_rate = 1.0 - k_d * dt;
        const double r_u = cpl * a_u * dt;
        const double v_vel = 1.0 - d_v * dt;
        const double v_pitch = -a_u * dt;
        const double v_u = a_u * dt;
        m.A = zero4();
        m.A[0][0] = 1.0;
        m.A[0][1] = dt * v_vel;
        m.A[0][2] = dt * v_pitch;
        m.A[1][1] = v_vel;
        m.A[1][2] = v_pitch;
        m.A[2][1] = dt * r_vel;
        m.A[2][2] = 1.0 + dt * r_pitch;
        m.A[2][3] = dt * r_rate;
        m.A[3][1] = r_vel;
        m.A[3][2] = r_pitch;
        m.A[3][3] = r_rate;
        m.B = {dt * v_u, v_u, dt * r_u, r_u};
    }

    m.d = mul4v(m.A, m.ref_k);
    for (int i = 0; i < 4; ++i) {
        m.d[i] += m.B[i] * m.u_ref_deg - m.ref_next[i];
    }
    return m;
}

inline AugmentedStepModel augment_step_model(const ErrorStepModel& m) {
    AugmentedStepModel a;
    a.u_ref_deg = m.u_ref_deg;
    a.u_ref_next_deg = m.u_ref_next_deg;
    a.A = zero5();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) a.A[i][j] = m.A[i][j];
        a.A[i][4] = m.B[i];
        a.B[i] = m.B[i];
        a.c[i] = m.d[i];
    }
    a.A[4][4] = 1.0;
    a.B[4] = 1.0;
    a.c[4] = m.u_ref_deg - m.u_ref_next_deg;
    return a;
}

inline Vec5 propagate_augmented(const AugmentedStepModel& m, const Vec5& z, double delta_move_deg) {
    Vec5 z_next = mul5v(m.A, z);
    for (int i = 0; i < 5; ++i) z_next[i] += m.B[i] * delta_move_deg + m.c[i];
    return z_next;
}

inline double stage_cost(const Config& cfg,
                         const Vec5& z_next,
                         double delta_move_deg) {
    return cfg.nmpc_w_x * z_next[0] * z_next[0] +
           cfg.nmpc_w_v * z_next[1] * z_next[1] +
           cfg.nmpc_w_theta * z_next[2] * z_next[2] +
           cfg.nmpc_w_theta_rate * z_next[3] * z_next[3] +
           cfg.nmpc_w_u * z_next[4] * z_next[4] +
           cfg.nmpc_w_du * delta_move_deg * delta_move_deg;
}

inline double terminal_input_cost(const Config& cfg, double delta_u_deg, double scale) {
    return cfg.nmpc_w_terminal_u * delta_u_deg * delta_u_deg * scale;
}
}  // namespace

NmpcController::~NmpcController() { stop(); }

void NmpcController::start(const Config& cfg, double dt) {
    stop();
    cfg_ = cfg;
    control_dt_ = dt;
    running_.store(true);
    worker_ = std::thread(&NmpcController::worker_loop, this);
}

void NmpcController::stop() {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
}

void NmpcController::reset() {
    published_tick_.store(0);
    NmpcResult r{};
    write_result(r);
}

void NmpcController::set_config(const Config& cfg, double dt) {
    cfg_ = cfg;
    control_dt_ = dt;
}

void NmpcController::publish(const NmpcState& state, const NmpcCommand& cmd) {
    NmpcProblem p;
    p.state = state;
    p.command = cmd;
    p.dt = control_dt_;
    p.tick = published_tick_.fetch_add(1) + 1;
    p.publish_time = std::chrono::steady_clock::now();
    write_problem(p);
}

bool NmpcController::try_get_latest(NmpcResult& out) const {
    return read_result(out) && out.valid;
}

void NmpcController::write_problem(const NmpcProblem& p) {
    const auto s0 = problem_slot_.seq.load(std::memory_order_relaxed);
    problem_slot_.seq.store(s0 + 1, std::memory_order_release);
    problem_slot_.data = p;
    problem_slot_.seq.store(s0 + 2, std::memory_order_release);
}

bool NmpcController::read_problem(NmpcProblem& p) const {
    for (int i = 0; i < 3; ++i) {
        const auto s1 = problem_slot_.seq.load(std::memory_order_acquire);
        if (s1 & 1ULL) continue;
        p = problem_slot_.data;
        const auto s2 = problem_slot_.seq.load(std::memory_order_acquire);
        if (s1 == s2 && !(s2 & 1ULL)) return true;
    }
    return false;
}

void NmpcController::write_result(const NmpcResult& r) {
    const auto s0 = result_slot_.seq.load(std::memory_order_relaxed);
    result_slot_.seq.store(s0 + 1, std::memory_order_release);
    result_slot_.data = r;
    result_slot_.seq.store(s0 + 2, std::memory_order_release);
}

bool NmpcController::read_result(NmpcResult& r) const {
    for (int i = 0; i < 3; ++i) {
        const auto s1 = result_slot_.seq.load(std::memory_order_acquire);
        if (s1 & 1ULL) continue;
        r = result_slot_.data;
        const auto s2 = result_slot_.seq.load(std::memory_order_acquire);
        if (s1 == s2 && !(s2 & 1ULL)) return true;
    }
    return false;
}

void NmpcController::worker_loop() {
    uint64_t last_tick = 0;
    while (running_.load()) {
        NmpcProblem p;
        if (!read_problem(p) || p.tick == last_tick) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        last_tick = p.tick;
        write_result(solve(p));
    }
}

NmpcResult NmpcController::solve(const NmpcProblem& p) const {
    const int horizon = std::clamp(cfg_.nmpc_horizon_steps, 4, kMaxHorizon);
    const int beam_width = std::clamp(cfg_.nmpc_candidate_count, 3, kMaxCandidates);
    const double du_limit = std::max(1e-3, cfg_.nmpc_pitch_slew_dps * p.dt);
    const std::array<double, 3> delta_actions{-du_limit, 0.0, du_limit};

    std::array<TrackingReference, kMaxHorizon + 1> refs{};
    std::array<ErrorStepModel, kMaxHorizon> err_models{};
    std::array<AugmentedStepModel, kMaxHorizon> aug_models{};
    for (int h = 0; h <= horizon; ++h) refs[h] = make_reference_at(cfg_, p, std::min(h, horizon - 1), horizon);
    for (int h = 0; h < horizon; ++h) {
        err_models[h] = build_error_step_model(cfg_, p, refs[h], refs[h + 1]);
        aug_models[h] = augment_step_model(err_models[h]);
    }

    const double term_scale = std::max(0.0, cfg_.nmpc_terminal_lqr_scale);
    std::array<Mat5, kMaxHorizon + 1> tail_P{};
    tail_P[horizon] = make_aug_weight(cfg_, term_scale);
    tail_P[horizon][4][4] = std::max(tail_P[horizon][4][4], cfg_.nmpc_w_terminal_u * term_scale);

    if (cfg_.nmpc_use_auto_lqr_terminal >= 0.5) {
        Vec4 K{}; Mat4 P{};
        if (solve_velocity_lqr_from_config(cfg_, p.dt, refs[horizon - 1].vel_tps, refs[horizon - 1].pitch_deg,
                                           cfg_.nmpc_ltv_velocity_scale, cfg_.nmpc_ltv_pitch_scale, K, P, true)) {
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) tail_P[horizon][i][j] += P[i][j] * term_scale;
        }
    }

    const Mat5 Q = make_aug_weight(cfg_, 1.0);
    const double R = std::max(1e-6, cfg_.nmpc_w_du);
    const double tail_mix = clamp(cfg_.nmpc_stage_lqr_tail_mix, 0.0, 1.0);

    for (int h = horizon - 1; h >= 0; --h) {
        Mat5 step_tail = tail_P[h + 1];
        if (tail_mix > 1e-6) {
            Vec4 K{}; Mat4 P{};
            if (solve_velocity_lqr_from_config(cfg_, p.dt, refs[h].vel_tps, refs[h].pitch_deg,
                                               cfg_.nmpc_ltv_velocity_scale, cfg_.nmpc_ltv_pitch_scale, K, P, true)) {
                for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) step_tail[i][j] = (1.0 - tail_mix) * step_tail[i][j] + tail_mix * P[i][j] * term_scale;
                step_tail[4][4] = (1.0 - tail_mix) * step_tail[4][4] + tail_mix * cfg_.nmpc_w_terminal_u * term_scale;
            }
        }
        const Mat5& A = aug_models[h].A;
        const Vec5& B = aug_models[h].B;
        const Mat5 At = transpose5(A);
        const Mat5 AtPA = mul5(At, mul5(step_tail, A));
        const Vec5 PB = mulPB5(step_tail, B);
        const Vec5 BtPA = BtP5(B, mul5(step_tail, A));
        double denom = R + dot5(B, PB);
        if (std::abs(denom) < 1e-9) denom = 1e-9;
        Mat5 P = Q;
        for (int i = 0; i < 5; ++i) for (int j = 0; j < 5; ++j) P[i][j] += AtPA[i][j] - (PB[i] * BtPA[j]) / denom;
        tail_P[h] = P;
    }

    std::array<BeamNode, kMaxCandidates> beam{}, next_beam{};
    int beam_count = 1;
    beam[0].z = {
        p.state.displacement_turns - refs[0].disp_turns,
        p.state.wheel_velocity_tps - refs[0].vel_tps,
        p.state.pitch_deg - refs[0].pitch_deg,
        p.state.pitch_rate_dps,
        clamp_pitch_candidate(p.command.previous_pitch_ref_deg, cfg_) - refs[0].pitch_deg
    };
    beam[0].first_u_deg = clamp_pitch_candidate(p.command.previous_pitch_ref_deg, cfg_);
    beam[0].cost = 0.0;
    beam[0].score = quad_cost5(beam[0].z, tail_P[0]);
    beam[0].has_first = false;

    auto maybe_insert = [&](std::array<BeamNode, kMaxCandidates>& arr, int& count, const BeamNode& cand) {
        if (count < beam_width) { arr[count++] = cand; return; }
        int worst_i = 0; double worst_score = arr[0].score;
        for (int i = 1; i < count; ++i) if (arr[i].score > worst_score) { worst_score = arr[i].score; worst_i = i; }
        if (cand.score < worst_score) arr[worst_i] = cand;
    };

    for (int h = 0; h < horizon; ++h) {
        int next_count = 0;
        for (int i = 0; i < beam_count; ++i) {
            const BeamNode& node = beam[i];
            const double u_prev_abs = node.z[4] + refs[h].pitch_deg;
            for (double delta_move : delta_actions) {
                BeamNode cand = node;
                const double u_abs = clamp_pitch_candidate(u_prev_abs + delta_move, cfg_);
                const double effective_delta_move = u_abs - u_prev_abs;
                cand.z = propagate_augmented(aug_models[h], node.z, effective_delta_move);
                cand.cost += stage_cost(cfg_, cand.z, effective_delta_move);
                if (!cand.has_first) { cand.first_u_deg = u_abs; cand.has_first = true; }
                cand.score = cand.cost + quad_cost5(cand.z, tail_P[h + 1]) + terminal_input_cost(cfg_, cand.z[4], term_scale);
                maybe_insert(next_beam, next_count, cand);
            }
        }
        std::sort(next_beam.begin(), next_beam.begin() + next_count, [](const BeamNode& a, const BeamNode& b) { return a.score < b.score; });
        beam_count = next_count;
        for (int i = 0; i < beam_count; ++i) beam[i] = next_beam[i];
    }

    NmpcResult best{}; best.cost = std::numeric_limits<double>::max();
    for (int i = 0; i < beam_count; ++i) {
        const BeamNode& node = beam[i];
        const double total = node.cost + quad_cost5(node.z, tail_P[horizon]) + terminal_input_cost(cfg_, node.z[4], term_scale);
        if (total < best.cost) {
            best.target_pitch_deg = clamp(node.has_first ? node.first_u_deg : p.command.previous_pitch_ref_deg, cfg_.target_pitch_max_bwd, cfg_.target_pitch_max_fwd);
            best.target_velocity_tps = refs[1].vel_tps;
            best.target_displacement_turns = refs[1].disp_turns;
            best.cost = total;
            best.source_tick = p.tick;
            best.publish_time = std::chrono::steady_clock::now();
            best.source_pitch_deg = p.state.pitch_deg;
            best.source_velocity_tps = p.state.wheel_velocity_tps;
            best.source_displacement_turns = p.state.displacement_turns;
            best.valid = true;
        }
    }
    return best;
}
}  // namespace robot
