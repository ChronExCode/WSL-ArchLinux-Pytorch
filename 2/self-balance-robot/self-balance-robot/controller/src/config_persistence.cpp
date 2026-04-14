// ═══════════════════════════════════════════════════════════════
//  Config Persistence — JSON save/load for tuning parameters
//  File: ~/self-balance-robot/controller/params.json
// ═══════════════════════════════════════════════════════════════

#include "wayland_app.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/stat.h>

// Simple JSON writer (no external lib needed)
bool WaylandApp::save_config() const {
    if (config_dir.empty()) return false;

    // Ensure directory exists
    mkdir(config_dir.c_str(), 0755);

    std::string path = config_dir + "/" + CONFIG_FILE;
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[Config] Cannot write " << path << "\n";
        return false;
    }

    f << "{\n";
    f << "  \"pitch_kp\": "  << pid_pitch[0].value << ",\n";
    f << "  \"pitch_ki\": "  << pid_pitch[1].value << ",\n";
    f << "  \"pitch_kd\": "  << pid_pitch[2].value << ",\n";
    f << "  \"pitch_d_alpha\": " << pid_pitch[3].value << ",\n";
    f << "  \"yaw_kp\": "    << pid_yaw[0].value   << ",\n";
    f << "  \"yaw_ki\": "    << pid_yaw[1].value   << ",\n";
    f << "  \"yaw_kd\": "    << pid_yaw[2].value   << ",\n";
    f << "  \"speed_kp\": "  << pid_speed[0].value << ",\n";
    f << "  \"speed_ki\": "  << pid_speed[1].value << ",\n";
    f << "  \"speed_kd\": "  << pid_speed[2].value << ",\n";
    f << "  \"pos_kp\": "    << pid_pos[0].value   << ",\n";
    f << "  \"pos_ki\": "    << pid_pos[1].value   << ",\n";
    f << "  \"pos_kd\": "    << pid_pos[2].value   << ",\n";
    f << "  \"pitch_offset\": "    << pid_other[0].value << ",\n";
    f << "  \"max_velocity\": "    << pid_other[1].value << ",\n";
    f << "  \"pos_out_max_fwd\": " << pid_limits[0].value << ",\n";
    f << "  \"pos_out_max_bwd\": " << pid_limits[1].value << ",\n";
    f << "  \"tgt_pitch_max_fwd\": " << pid_limits[2].value << ",\n";
    f << "  \"tgt_pitch_max_bwd\": " << pid_limits[3].value << ",\n";
    f << "  \"server_ip\": \"" << server_ip << "\"\n";
    f << "}\n";

    f.close();
    std::cout << "[Config] Saved to " << path << "\n";
    return true;
}

// Simple JSON reader using existing json_parse_double helper
bool WaylandApp::load_config() {
    if (config_dir.empty()) return false;

    std::string path = config_dir + "/" + CONFIG_FILE;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cout << "[Config] No config file at " << path << ", using defaults\n";
        return false;
    }

    std::stringstream ss;
    ss << f.rdbuf();
    std::string json = ss.str();
    f.close();

    // Parse each field — use -999 sentinel for "not present"
    auto get = [&](const std::string& key, double def) -> double {
        return json_parse_double(json, key, def);
    };

    pid_pitch[0].value = get("pitch_kp", pid_pitch[0].value);
    pid_pitch[1].value = get("pitch_ki", pid_pitch[1].value);
    pid_pitch[2].value = get("pitch_kd", pid_pitch[2].value);
    pid_pitch[3].value = get("pitch_d_alpha", pid_pitch[3].value);
    pid_yaw[0].value   = get("yaw_kp",   pid_yaw[0].value);
    pid_yaw[1].value   = get("yaw_ki",   pid_yaw[1].value);
    pid_yaw[2].value   = get("yaw_kd",   pid_yaw[2].value);
    pid_speed[0].value = get("speed_kp",  pid_speed[0].value);
    pid_speed[1].value = get("speed_ki",  pid_speed[1].value);
    pid_speed[2].value = get("speed_kd",  pid_speed[2].value);
    pid_pos[0].value   = get("pos_kp",    pid_pos[0].value);
    pid_pos[1].value   = get("pos_ki",    pid_pos[1].value);
    pid_pos[2].value   = get("pos_kd",    pid_pos[2].value);
    pid_other[0].value = get("pitch_offset",    pid_other[0].value);
    pid_other[1].value = get("max_velocity",    pid_other[1].value);
    pid_limits[0].value = get("pos_out_max_fwd", pid_limits[0].value);
    pid_limits[1].value = get("pos_out_max_bwd", pid_limits[1].value);
    pid_limits[2].value = get("tgt_pitch_max_fwd", pid_limits[2].value);
    pid_limits[3].value = get("tgt_pitch_max_bwd", pid_limits[3].value);

    // Server IP
    std::string ip = json_parse_string(json, "server_ip");
    if (!ip.empty()) server_ip = ip;

    std::cout << "[Config] Loaded from " << path << "\n";
    return true;
}
