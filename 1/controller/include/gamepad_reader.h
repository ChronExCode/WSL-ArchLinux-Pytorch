#pragma once
// ═══════════════════════════════════════════════════════════════
//  Gamepad Reader — evdev + epoll
// ═══════════════════════════════════════════════════════════════

#include <linux/input.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <map>
#include <mutex>
#include <cmath>
#include <iostream>
#include <errno.h>

#define MAX_EPOLL_EVENTS 16
#define MAX_INPUT_EVENTS 32
#define INPUT_EVENT_NUM 3

class GamepadReader {
public:
    int epollfd;
    int fd[INPUT_EVENT_NUM];
    bool is_open;
    struct epoll_event events[MAX_EPOLL_EVENTS];

    GamepadReader() : is_open(false) {}

    int add_fd_to_epoll(int efd, const char *devpath) {
        int f = open(devpath, O_RDONLY | O_NONBLOCK);
        if (f < 0) {
            fprintf(stderr, "open %s failed: %s\n", devpath, strerror(errno));
            return -1;
        }
        struct epoll_event ev;
        memset(&ev, 0, sizeof(ev));
        ev.events = EPOLLIN;
        ev.data.fd = f;
        if (epoll_ctl(efd, EPOLL_CTL_ADD, f, &ev) < 0) {
            fprintf(stderr, "epoll_ctl add %s failed: %s\n", devpath, strerror(errno));
            close(f);
            return -1;
        }
        return f;
    }

    int epollfd_open() {
        static const char *strPtrArray[INPUT_EVENT_NUM] = {
            "/dev/input/event4",
            "/dev/input/event5",
            "/dev/input/event6"
        };
        epollfd = epoll_create1(0);
        if (epollfd < 0) { perror("epoll_create1"); return 1; }
        for (int i = 0; i < INPUT_EVENT_NUM; i++) {
            fd[i] = add_fd_to_epoll(epollfd, strPtrArray[i]);
            if (fd[i] < 0) {
                fprintf(stderr, "epoll_ctl add %s failed: %s\n",
                        strPtrArray[i], strerror(errno));
                close(epollfd);
                return -1;
            }
        }
        is_open = true;
        return 0;
    }

    int epollfd_close() {
        for (int i = 0; i < INPUT_EVENT_NUM; i++) {
            if (fd[i] >= 0) close(fd[i]);
        }
        close(epollfd);
        is_open = false;
        return 0;
    }

    bool button(int num) { return num == 1; }

    const std::string& name() const { return name_; }

    void dump_state() const {
        std::lock_guard<std::mutex> lock(mtx_);
        std::cout << "[Joy] Axes:";
        for (auto& [k, v] : axes_) { printf(" %d=%.2f", k, v); }
        std::cout << "\n[Joy] Btns:";
        for (auto& [k, v] : buttons_) { if (v) printf(" %d=ON", k); }
        std::cout << "\n";
    }

private:
    int fd_ = -1;
    std::string name_;
    mutable std::mutex mtx_;
    std::map<int, double> axes_;
    std::map<int, int> buttons_;
    struct AbsInfo { int min=0, max=0, flat=0; };
    std::map<int, AbsInfo> abs_info_;

    double normalize_axis(int code, int value) const {
        auto it = abs_info_.find(code);
        if (it == abs_info_.end()) return value / 32767.0;
        const auto& info = it->second;
        int range = info.max - info.min;
        if (range == 0) return 0.0;
        if (code == ABS_Z || code == ABS_RZ) {
            return static_cast<double>(value - info.min) / range;
        }
        double center = (info.min + info.max) / 2.0;
        double half = range / 2.0;
        double norm = (value - center) / half;
        if (info.flat > 0 && std::abs(norm) < (info.flat / half)) return 0.0;
        return std::max(-1.0, std::min(1.0, norm));
    }

    static const char* key_name(int code) {
        switch (code) {
            case BTN_SOUTH:  return "A";     case BTN_EAST:   return "B";
            case BTN_NORTH:  return "X";     case BTN_WEST:   return "Y";
            case BTN_TL:     return "LB";    case BTN_TR:     return "RB";
            case BTN_SELECT: return "Back";  case BTN_START:  return "Start";
            case BTN_MODE:   return "Home";  case BTN_THUMBL: return "LS";
            case BTN_THUMBR: return "RS";    default: return "?";
        }
    }
};
