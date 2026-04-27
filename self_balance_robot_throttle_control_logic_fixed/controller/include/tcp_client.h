#pragma once
// ═══════════════════════════════════════════════════════════════
//  TCP Client — connects to Raspberry Pi robot server
// ═══════════════════════════════════════════════════════════════

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>

#include <string>
#include <vector>

class TcpClient {
public:
    bool connect(const std::string& host, uint16_t port) {
        close_fd();
        partial_.clear();

        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) return false;

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

        fcntl(fd_, F_SETFL, O_NONBLOCK);

        int ret = ::connect(fd_, (struct sockaddr*)&addr, sizeof(addr));
        if (ret < 0 && errno != EINPROGRESS) {
            close_fd();
            return false;
        }

        struct pollfd pfd = { fd_, POLLOUT, 0 };
        ret = ::poll(&pfd, 1, 3000);
        if (ret <= 0) { close_fd(); return false; }

        int err;
        socklen_t len = sizeof(err);
        getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err != 0) { close_fd(); return false; }

        fcntl(fd_, F_SETFL, 0);
        int flag = 1;
        setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        connected_ = true;
        return true;
    }

    void disconnect() {
        close_fd();
        partial_.clear();
    }

    bool send_line(const std::string& line) {
        if (!connected_ || fd_ < 0) return false;
        std::string data = line + "\n";
        size_t sent = 0;
        while (sent < data.size()) {
            ssize_t n = ::send(fd_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
            if (n <= 0) { on_error(); return false; }
            sent += static_cast<size_t>(n);
        }
        return true;
    }

    std::vector<std::string> recv_lines() {
        std::vector<std::string> lines;
        if (!connected_ || fd_ < 0) return lines;

        struct pollfd pfd = { fd_, POLLIN, 0 };
        while (::poll(&pfd, 1, 0) > 0) {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                on_error(); break;
            }
            char buf[4096];
            ssize_t n = ::recv(fd_, buf, sizeof(buf) - 1, 0);
            if (n <= 0) { on_error(); break; }
            buf[n] = '\0';
            partial_ += buf;
        }

        size_t pos;
        while ((pos = partial_.find('\n')) != std::string::npos) {
            lines.push_back(partial_.substr(0, pos));
            partial_ = partial_.substr(pos + 1);
        }
        return lines;
    }

    bool is_connected() const { return connected_; }

private:
    int fd_ = -1;
    bool connected_ = false;
    std::string partial_;

    void close_fd() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        connected_ = false;
    }
    void on_error() {
        close_fd();
        partial_.clear();
    }
};
