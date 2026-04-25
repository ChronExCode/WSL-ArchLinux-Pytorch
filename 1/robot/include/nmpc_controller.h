#pragma once

#include "types.h"

#include <atomic>
#include <thread>

namespace robot {

struct NmpcState {
    double pitch_deg = 0.0;
    double pitch_rate_dps = 0.0;
    double displacement_turns = 0.0;
    double wheel_velocity_tps = 0.0;
};

struct NmpcCommand {
    double target_speed_tps = 0.0;
    double target_displacement_turns = 0.0;
    double nominal_pitch_deg = 0.0;
    double balance_bias_deg = 0.0;
    double previous_pitch_ref_deg = 0.0;
};

struct NmpcProblem {
    NmpcState state{};
    NmpcCommand command{};
    double dt = 0.005;
    uint64_t tick = 0;
    std::chrono::steady_clock::time_point publish_time{};
};

struct NmpcResult {
    double target_pitch_deg = 0.0;
    double target_velocity_tps = 0.0;
    double target_displacement_turns = 0.0;
    double cost = 0.0;
    uint64_t source_tick = 0;
    std::chrono::steady_clock::time_point publish_time{};
    double source_pitch_deg = 0.0;
    double source_velocity_tps = 0.0;
    double source_displacement_turns = 0.0;
    bool valid = false;
};

template <typename T>
struct alignas(64) SeqLockSlot {
    std::atomic<uint64_t> seq{0};
    T data{};
};

class NmpcController {
public:
    NmpcController() = default;
    ~NmpcController();

    NmpcController(const NmpcController&) = delete;
    NmpcController& operator=(const NmpcController&) = delete;

    void start(const Config& cfg, double dt);
    void stop();
    void reset();
    void set_config(const Config& cfg, double dt);

    void publish(const NmpcState& state, const NmpcCommand& cmd);
    bool try_get_latest(NmpcResult& out) const;

private:
    void worker_loop();
    void write_problem(const NmpcProblem& p);
    bool read_problem(NmpcProblem& p) const;
    void write_result(const NmpcResult& r);
    bool read_result(NmpcResult& r) const;
    NmpcResult solve(const NmpcProblem& p) const;

    static constexpr int kMaxCandidates = 64;
    static constexpr int kMaxHorizon = 32;

    Config cfg_{};
    double control_dt_ = 0.005;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::atomic<uint64_t> published_tick_{0};
    mutable SeqLockSlot<NmpcProblem> problem_slot_{};
    mutable SeqLockSlot<NmpcResult> result_slot_{};
};

}  // namespace robot
