#pragma once
#include <fstream>
#include <mutex>
#include <string>
#include "types.h"

namespace robot {

class CsvLogger {
public:
    bool start(const std::string& directory, bool enabled);
    void stop();
    void log_config_snapshot(const Config& cfg);
    void log_config_update(double t_s, const Config& cfg, const std::string& source);
    void log_event(double t_s, const std::string& name, const std::string& message);
    void log(double t_s, SystemState state, const ImuData& imu, const RemoteCommand& cmd,
             const PidDebugData& dbg, const MotorOutput& out, double enc_disp, double enc_vel);
private:
    void write_comment_line_unlocked(const std::string& line);
    bool enabled_ = false;
    std::ofstream file_;
    std::mutex mtx_;
};

}  // namespace robot
