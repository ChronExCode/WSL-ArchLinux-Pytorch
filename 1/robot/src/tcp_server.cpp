// ═══════════════════════════════════════════════════════════════
//  TCP Server — WiFi Remote Control Implementation
// ═══════════════════════════════════════════════════════════════

#include "tcp_server.h"
#include "mini_json.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <errno.h>
#include <fcntl.h>

namespace robot {

TcpServer::TcpServer() = default;
TcpServer::~TcpServer() { stop(); }

bool TcpServer::start(uint16_t port) {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) return false;

    int flags = fcntl(server_fd_, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
    }
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[TCP] bind port " << port << " failed: " << strerror(errno) << "\n";
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }
    if (listen(server_fd_, 1) < 0) {
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }
    running_.store(true);
    accept_thread_ = std::thread(&TcpServer::accept_loop, this);
    std::cout << "[TCP] Listening on port " << port << "\n";
    return true;
}

void TcpServer::stop() {
    running_.store(false);

    std::cout << "[TCP] stop: closing client\n";
    int cfd = client_fd_.exchange(-1);
    if (cfd >= 0) {
        shutdown(cfd, SHUT_RDWR);
        close(cfd);
    }

    std::cout << "[TCP] stop: closing server\n";
    int sfd = server_fd_;
    if (sfd >= 0) {
        shutdown(sfd, SHUT_RDWR);
        close(sfd);
        server_fd_ = -1;
    }

    std::cout << "[TCP] stop: joining accept thread\n";
    if (accept_thread_.joinable()) accept_thread_.join();

    std::cout << "[TCP] stop: joining recv thread\n";
    if (recv_thread_.joinable()) recv_thread_.join();

    std::cout << "[TCP] stop: done\n";
}

void TcpServer::accept_loop() {
    while (running_.load()) {
        int sfd = server_fd_;
        if (sfd < 0) break;

        struct pollfd pfd{};
        pfd.fd = sfd;
        pfd.events = POLLIN;

        int pr = poll(&pfd, 1, 200);
        if (!running_.load()) break;

        if (pr < 0) {
            if (errno == EINTR) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        if (pr == 0) continue;

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            break;
        }
        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        struct sockaddr_in caddr{};
        socklen_t len = sizeof(caddr);
        int cfd = accept(sfd, (struct sockaddr*)&caddr, &len);
        if (cfd < 0) {
            if (!running_.load()) break;
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        int flag = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        int cflags = fcntl(cfd, F_GETFL, 0);
        if (cflags >= 0) {
            fcntl(cfd, F_SETFL, cflags | O_NONBLOCK);
        }

        int old = client_fd_.exchange(cfd);
        if (old >= 0) {
            shutdown(old, SHUT_RDWR);
            close(old);
        }

        if (recv_thread_.joinable()) recv_thread_.join();
        recv_thread_ = std::thread(&TcpServer::receive_loop, this, cfd);

        char ip[32]{};
        inet_ntop(AF_INET, &caddr.sin_addr, ip, sizeof(ip));
        std::cout << "[TCP] Client connected from " << ip << "\n";
    }

    std::cout << "[TCP] Accept loop stopped\n";
}

void TcpServer::receive_loop(int client_fd) {
    std::string partial;
    char buf[2048];
    while (running_.load() && client_fd_.load() == client_fd) {
        struct pollfd pfd{client_fd, POLLIN, 0};
        int pr = poll(&pfd, 1, 200);
        if (pr <= 0) continue;
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) break;
        ssize_t n = recv(client_fd, buf, sizeof(buf), 0);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            break;
        }
        if (n == 0) break;
        partial.append(buf, n);
        size_t pos;
        while ((pos = partial.find('\n')) != std::string::npos) {
            std::string line = partial.substr(0, pos);
            partial.erase(0, pos + 1);
            if (!line.empty()) parse_command(line);
        }
    }
    int old = client_fd_.exchange(-1);
    if (old >= 0) {
        shutdown(old, SHUT_RDWR);
        close(old);
    }
    std::cout << "[TCP] Client disconnected\n";
}

double TcpServer::json_get_double(const std::string& json, const std::string& key, double def) {
    return minijson::get_number_or(json, key, def);
}

std::string TcpServer::json_get_string(const std::string& json, const std::string& key) {
    return minijson::get_string_or(json, key);
}

void TcpServer::parse_command(const std::string& json) {
    std::string type = json_get_string(json, "type");
    if (type == "control") {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        latest_cmd_.throttle = json_get_double(json, "throttle");
        latest_cmd_.steering = json_get_double(json, "steering");
        latest_cmd_.speed_limit = json_get_double(json, "speed_limit", 1.0);
        latest_cmd_.timestamp = std::chrono::steady_clock::now();
    } else if (type == "arm" || type == "disarm" || type == "estop") {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        if (type == "estop") latest_cmd_.emergency_stop = true;
        if (type == "arm") { latest_cmd_.arm = true; latest_cmd_.emergency_stop = false; }
        if (type == "disarm") latest_cmd_.arm = false;
        latest_cmd_.timestamp = std::chrono::steady_clock::now();
        if (state_cb_) state_cb_(type);
    } else if (type == "set_pid" || type == "set_params") {
        if (pid_cb_) pid_cb_(json);
    } else if (type == "ping") {
        send_to_client("{\"type\":\"pong\"}\n");
    }
}

RemoteCommand TcpServer::get_command() const {
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    return latest_cmd_;
}

bool TcpServer::is_command_fresh(int max_age_ms) const {
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    auto age = std::chrono::steady_clock::now() - latest_cmd_.timestamp;
    return std::chrono::duration_cast<std::chrono::milliseconds>(age).count() < max_age_ms;
}

bool TcpServer::send_to_client(const std::string& data) {
    int cfd = client_fd_.load();
    if (cfd < 0) return false;
    size_t sent = 0;
    while (sent < data.size()) {
        ssize_t n = send(cfd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

std::string TcpServer::telemetry_json(double pitch, double roll, double yaw,
                                      double speed, double vbus, const char* state,
                                      double left_vel, double right_vel,
                                      bool odrive_usb, bool imu_usb,
                                      const std::string& status_msg,
                                      const ImuData& imu,
                                      const PidDebugData& pid_dbg) {
    minijson::ObjectBuilder obj;
    obj.add_string("type", "telemetry");
    obj.add_number("pitch", pitch); obj.add_number("roll", roll); obj.add_number("yaw", yaw); obj.add_number("speed", speed); obj.add_number("vbus", vbus);
    obj.add_string("state", state); obj.add_number("left_vel", left_vel); obj.add_number("right_vel", right_vel);
    obj.add_bool("odrive_usb", odrive_usb); obj.add_bool("imu_usb", imu_usb);
    obj.add_number("gyro_x", imu.gyro_x); obj.add_number("gyro_y", imu.gyro_y); obj.add_number("gyro_z", imu.gyro_z);
    obj.add_number("acc_x", imu.acc_x); obj.add_number("acc_y", imu.acc_y); obj.add_number("acc_z", imu.acc_z);
    obj.add_number("tgt_pitch", pid_dbg.target_pitch); obj.add_number("flt_pitch", pid_dbg.filtered_pitch); obj.add_number("pitch_err", pid_dbg.pitch_error);
    obj.add_number("pitch_p", pid_dbg.pitch_p); obj.add_number("pitch_i", pid_dbg.pitch_i); obj.add_number("pitch_d", pid_dbg.pitch_d); obj.add_number("pitch_out", pid_dbg.pitch_output);
    obj.add_number("tgt_speed", pid_dbg.target_speed); obj.add_number("spd_est", pid_dbg.speed_estimate); obj.add_number("spd_err", pid_dbg.speed_error);
    obj.add_number("ang_vel", pid_dbg.gyro_y); obj.add_number("ang_acc", pid_dbg.gyro_y_dot);
    obj.add_number("pos_l", pid_dbg.pos_left); obj.add_number("pos_r", pid_dbg.pos_right); obj.add_number("pos_disp", pid_dbg.pos_disp); obj.add_number("pos_target", pid_dbg.pos_target); obj.add_number("pos_out", pid_dbg.pos_out);
    obj.add_number("cmd_l", pid_dbg.cmd_left); obj.add_number("cmd_r", pid_dbg.cmd_right);
    obj.add_number("bal_pt", pid_dbg.balance_point); obj.add_number("bal_trim", pid_dbg.balance_trim);
    obj.add_number("throttle_f", pid_dbg.throttle_filtered); obj.add_number("steering_f", pid_dbg.steering_filtered); obj.add_number("theta_nom", pid_dbg.nominal_pitch); obj.add_number("theta_err", pid_dbg.theta_error); obj.add_number("theta_d_err", pid_dbg.theta_d_error);
    obj.add_number("x_err", pid_dbg.x_error); obj.add_number("v_err", pid_dbg.v_error);
    obj.add_number("fb_theta", pid_dbg.fb_theta); obj.add_number("fb_theta_d", pid_dbg.fb_theta_d); obj.add_number("fb_x", pid_dbg.fb_x); obj.add_number("fb_v", pid_dbg.fb_v);
    obj.add_number("ff_term", pid_dbg.ff_term); obj.add_number("i_term", pid_dbg.integral_term); obj.add_number("base_cmd", pid_dbg.base_cmd);
    obj.add_bool("nmpc_used", pid_dbg.nmpc_used); obj.add_number("nmpc_cost", pid_dbg.nmpc_cost); obj.add_number("nmpc_age_ms", pid_dbg.nmpc_age_ms);
    if (!status_msg.empty()) obj.add_string("msg", status_msg);
    return obj.str() + "\n";
}

void TcpServer::send_telemetry(double pitch, double roll, double yaw, double speed,
                               double vbus, SystemState state,
                               double left_vel, double right_vel,
                               bool odrive_usb, bool imu_usb,
                               const std::string& status_msg,
                               const ImuData& imu,
                               const PidDebugData& pid_dbg) {
    send_to_client(telemetry_json(pitch, roll, yaw, speed, vbus, state_name(state),
                                  left_vel, right_vel, odrive_usb, imu_usb,
                                  status_msg, imu, pid_dbg));
}

void TcpServer::set_state_callback(StateCallback cb) { state_cb_ = std::move(cb); }
void TcpServer::set_pid_callback(PidCallback cb) { pid_cb_ = std::move(cb); }

} // namespace robot
