#pragma once
// ═══════════════════════════════════════════════════════════════
//  TCP Server — Receives remote control commands over WiFi
// ═══════════════════════════════════════════════════════════════

#include "types.h"
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>

namespace robot {

class TcpServer {
public:
    TcpServer();
    ~TcpServer();
    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;

    bool start(uint16_t port);
    void stop();
    bool has_client() const { return client_fd_.load() >= 0; }
    RemoteCommand get_command() const;
    void send_telemetry(double pitch, double roll, double yaw, double speed,
                        double vbus, SystemState state,
                        double left_vel, double right_vel,
                        const ImuData& imu = ImuData{},
                        const PidDebugData& pid_dbg = PidDebugData{});

    using StateCallback = std::function<void(const std::string& type)>;
    void set_state_callback(StateCallback cb);

    using PidCallback = std::function<void(const Config& partial)>;
    void set_pid_callback(PidCallback cb);

    bool is_command_fresh(int max_age_ms = 500) const;

private:
    int server_fd_ = -1;
    std::atomic<int> client_fd_{-1};
    std::atomic<bool> running_{false};
    mutable std::mutex cmd_mtx_;
    RemoteCommand latest_cmd_;
    std::thread accept_thread_, recv_thread_;
    StateCallback state_cb_;
    PidCallback pid_cb_;

    void accept_loop();
    void receive_loop(int client_fd);
    void parse_command(const std::string& json);
    bool send_to_client(const std::string& data);

    static std::string telemetry_json(double pitch, double roll, double yaw,
                                       double speed, double vbus, const char* state,
                                       double left_vel, double right_vel,
                                       const ImuData& imu,
                                       const PidDebugData& pid_dbg);
    static double json_get_double(const std::string& json, const std::string& key, double def = 0.0);
    static std::string json_get_string(const std::string& json, const std::string& key);
};

} // namespace robot
