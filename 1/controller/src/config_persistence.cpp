#include "wayland_app.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/stat.h>

bool WaylandApp::save_config() const {
    if (config_dir.empty()) return false;
    mkdir(config_dir.c_str(), 0755);
    std::ofstream f(config_dir + "/" + CONFIG_FILE);
    if (!f.is_open()) return false;
    minijson::ObjectBuilder obj;
    for (const auto& p : params) obj.add_number(p.key, p.value);
    obj.add_string("server_ip", server_ip);
    f << obj.str() << "\n";
    return true;
}

bool WaylandApp::load_config() {
    if (config_dir.empty()) return false;
    std::ifstream f(config_dir + "/" + CONFIG_FILE);
    if (!f.is_open()) return false;
    std::stringstream ss; ss << f.rdbuf();
    std::string json = ss.str();
    for (auto& p : params) {
        p.value = json_parse_double(json, p.key, p.value);
        if (p.value < p.min) p.value = p.min;
        if (p.value > p.max) p.value = p.max;
    }
    std::string ip = json_parse_string(json, "server_ip");
    if (!ip.empty()) server_ip = ip;
    return true;
}
