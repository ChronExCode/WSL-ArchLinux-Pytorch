// ═══════════════════════════════════════════════════════════════
//  TCP Server — WiFi Remote Control Implementation
// ═══════════════════════════════════════════════════════════════

#include "tcp_server.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <iostream>

namespace robot {

TcpServer::TcpServer() = default;
TcpServer::~TcpServer() { stop(); }

bool TcpServer::start(uint16_t port) {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) return false;
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET; addr.sin_addr.s_addr = INADDR_ANY; addr.sin_port = htons(port);
    if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(server_fd_); server_fd_ = -1; return false; }
    if (listen(server_fd_, 1) < 0) { close(server_fd_); server_fd_ = -1; return false; }
    running_.store(true);
    accept_thread_ = std::thread(&TcpServer::accept_loop, this);
    std::cout << "[TCP] Listening on port " << port << "\n";
    return true;
}

void TcpServer::stop() {
    running_.store(false);
    int cfd = client_fd_.exchange(-1); if (cfd >= 0) close(cfd);
    if (server_fd_ >= 0) { close(server_fd_); server_fd_ = -1; }
    if (accept_thread_.joinable()) accept_thread_.join();
    if (recv_thread_.joinable()) recv_thread_.join();
}

void TcpServer::accept_loop() {
    while (running_.load()) {
        struct pollfd pfd = { server_fd_, POLLIN, 0 };
        if (poll(&pfd, 1, 500) <= 0 || !running_.load()) continue;
        struct sockaddr_in ca{}; socklen_t len = sizeof(ca);
        int cfd = accept(server_fd_, (struct sockaddr*)&ca, &len);
        if (cfd < 0) continue;

        // Disconnect previous client if any
        int old = client_fd_.exchange(-1);
        if (old >= 0) {
            close(old);
        }
        // ALWAYS join previous recv thread before starting new one
        // (even if client_fd_ was already -1 from recv_loop self-exit)
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }

        int flag = 1; setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        char ip[INET_ADDRSTRLEN]; inet_ntop(AF_INET, &ca.sin_addr, ip, sizeof(ip));
        std::cout << "[TCP] Client connected: " << ip << "\n";
        client_fd_.store(cfd);
        recv_thread_ = std::thread(&TcpServer::receive_loop, this, cfd);
    }
}

void TcpServer::receive_loop(int client_fd) {
    char buf[4096]; std::string partial;
    while (running_.load() && client_fd_.load() == client_fd) {
        struct pollfd pfd = { client_fd, POLLIN, 0 };
        int ret = poll(&pfd, 1, 200);
        if (ret < 0) break;
        if (ret == 0) continue;
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) break;
        ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) break;
        buf[n] = '\0'; partial += buf;
        size_t pos;
        while ((pos = partial.find('\n')) != std::string::npos) {
            std::string line = partial.substr(0, pos); partial = partial.substr(pos + 1);
            if (!line.empty()) parse_command(line);
        }
    }
    // Mark as disconnected. Only close if we still own this fd
    // (accept_loop may have already exchanged it away for a new client)
    int expected = client_fd;
    if (client_fd_.compare_exchange_strong(expected, -1)) {
        close(client_fd);
    }
    // else: accept_loop already closed it and started a new recv_thread
    std::cout << "[TCP] Client disconnected\n";
}

double TcpServer::json_get_double(const std::string& json, const std::string& key, double def) {
    std::string search = "\"" + key + "\":";
    size_t pos = json.find(search); if (pos == std::string::npos) return def;
    pos += search.size();
    while (pos < json.size() && json[pos] == ' ') pos++;
    try { return std::stod(json.substr(pos)); } catch (...) { return def; }
}

std::string TcpServer::json_get_string(const std::string& json, const std::string& key) {
    std::string search = "\"" + key + "\":\"";
    size_t pos = json.find(search); if (pos == std::string::npos) return {};
    pos += search.size();
    size_t end = json.find('"', pos); if (end == std::string::npos) return {};
    return json.substr(pos, end - pos);
}

void TcpServer::parse_command(const std::string& json) {
    std::string type = json_get_string(json, "type");
    if (type == "control") {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        latest_cmd_.throttle = json_get_double(json, "throttle");
        latest_cmd_.steering = json_get_double(json, "steering");
        latest_cmd_.speed_limit = json_get_double(json, "speed_limit", 1.0);
        latest_cmd_.timestamp = std::chrono::steady_clock::now();
        // Do NOT overwrite arm/emergency_stop — those are edge-triggered
    } else if (type == "arm" || type == "disarm" || type == "estop") {
        if (type == "estop") { std::lock_guard<std::mutex> lock(cmd_mtx_); latest_cmd_.emergency_stop = true; latest_cmd_.timestamp = std::chrono::steady_clock::now(); }
        if (type == "arm") { std::lock_guard<std::mutex> lock(cmd_mtx_); latest_cmd_.arm = true; latest_cmd_.emergency_stop = false; /* clear stale estop on arm */ }
        if (type == "disarm") { std::lock_guard<std::mutex> lock(cmd_mtx_); latest_cmd_.arm = false; }
        if (state_cb_) state_cb_(type);
    } else if (type == "set_pid") {
        Config partial;
        partial.pitch_kp = json_get_double(json, "pitch_kp", -1);
        partial.pitch_ki = json_get_double(json, "pitch_ki", -1);
        partial.pitch_kd = json_get_double(json, "pitch_kd", -1);
        partial.yaw_kp = json_get_double(json, "yaw_kp", -1);
        partial.speed_kp = json_get_double(json, "speed_kp", -1);
        partial.pitch_offset = json_get_double(json, "pitch_offset", -999);
        partial.max_velocity = json_get_double(json, "max_velocity", -1);
        if (pid_cb_) pid_cb_(partial);
    } else if (type == "ping") { send_to_client("{\"type\":\"pong\"}\n"); }
}

RemoteCommand TcpServer::get_command() const { std::lock_guard<std::mutex> lock(cmd_mtx_); return latest_cmd_; }

bool TcpServer::is_command_fresh(int max_age_ms) const {
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    auto age = std::chrono::steady_clock::now() - latest_cmd_.timestamp;
    return std::chrono::duration_cast<std::chrono::milliseconds>(age).count() < max_age_ms;
}

bool TcpServer::send_to_client(const std::string& data) {
    int cfd = client_fd_.load(); if (cfd < 0) return false;
    return send(cfd, data.c_str(), data.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(data.size());
}

std::string TcpServer::telemetry_json(double pitch, double roll, double yaw,
                                       double speed, double vbus, const char* state,
                                       double left_vel, double right_vel,
                                       const ImuData& imu,
                                       const PidDebugData& pid_dbg) {
    char buf[2048];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"telemetry\",\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.1f,\"speed\":%.2f,"
        "\"vbus\":%.1f,\"state\":\"%s\",\"left_vel\":%.2f,\"right_vel\":%.2f,"
        "\"gyro_x\":%.2f,\"gyro_y\":%.2f,\"gyro_z\":%.2f,"
        "\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,"
        "\"tgt_pitch\":%.3f,\"flt_pitch\":%.3f,\"pitch_err\":%.3f,"
        "\"pitch_p\":%.3f,\"pitch_i\":%.3f,\"pitch_d\":%.3f,\"pitch_out\":%.4f,"
        "\"tgt_speed\":%.3f,\"spd_est\":%.3f,\"spd_err\":%.3f,"
        "\"ang_vel\":%.2f,\"ang_acc\":%.1f,"
        "\"pos_l\":%.3f,\"pos_r\":%.3f}\n",
        pitch, roll, yaw, speed, vbus, state, left_vel, right_vel,
        imu.gyro_x, imu.gyro_y, imu.gyro_z,
        imu.acc_x, imu.acc_y, imu.acc_z,
        pid_dbg.target_pitch, pid_dbg.filtered_pitch, pid_dbg.pitch_error,
        pid_dbg.pitch_p, pid_dbg.pitch_i, pid_dbg.pitch_d, pid_dbg.pitch_output,
        pid_dbg.target_speed, pid_dbg.speed_estimate, pid_dbg.speed_error,
        pid_dbg.gyro_y, pid_dbg.gyro_y_dot,
        pid_dbg.pos_left, pid_dbg.pos_right);
    return buf;
}

void TcpServer::send_telemetry(double pitch, double roll, double yaw, double speed,
                                double vbus, SystemState state,
                                double left_vel, double right_vel,
                                const ImuData& imu,
                                const PidDebugData& pid_dbg) {
    send_to_client(telemetry_json(pitch, roll, yaw, speed, vbus, state_name(state),
                                   left_vel, right_vel, imu, pid_dbg));
}

void TcpServer::set_state_callback(StateCallback cb) { state_cb_ = std::move(cb); }
void TcpServer::set_pid_callback(PidCallback cb) { pid_cb_ = std::move(cb); }

} // namespace robot
