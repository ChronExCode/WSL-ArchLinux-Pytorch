#pragma once
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace minijson {

inline std::string escape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (unsigned char c : in) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[7];
                    snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out.push_back(static_cast<char>(c));
                }
                break;
        }
    }
    return out;
}

inline std::string unescape(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        char c = in[i];
        if (c != '\\' || i + 1 >= in.size()) {
            out.push_back(c);
            continue;
        }
        char n = in[++i];
        switch (n) {
            case '\\': out.push_back('\\'); break;
            case '"': out.push_back('"'); break;
            case 'n': out.push_back('\n'); break;
            case 'r': out.push_back('\r'); break;
            case 't': out.push_back('\t'); break;
            default: out.push_back(n); break;
        }
    }
    return out;
}

inline void skip_ws(const std::string& s, size_t& pos) {
    while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) ++pos;
}

inline bool find_key(const std::string& json, const std::string& key, size_t& value_pos) {
    const std::string needle = "\"" + key + "\"";
    size_t pos = 0;
    while ((pos = json.find(needle, pos)) != std::string::npos) {
        // ensure not escaped quote before key start
        if (pos > 0 && json[pos - 1] == '\\') { pos += needle.size(); continue; }
        pos += needle.size();
        skip_ws(json, pos);
        if (pos >= json.size() || json[pos] != ':') continue;
        ++pos;
        skip_ws(json, pos);
        value_pos = pos;
        return true;
    }
    return false;
}

inline bool get_number(const std::string& json, const std::string& key, double& out) {
    size_t pos = 0;
    if (!find_key(json, key, pos)) return false;
    char* end = nullptr;
    const char* begin = json.c_str() + pos;
    errno = 0;
    double v = std::strtod(begin, &end);
    if (begin == end || errno == ERANGE || !std::isfinite(v)) return false;
    out = v;
    return true;
}

inline double get_number_or(const std::string& json, const std::string& key, double def) {
    double v = def;
    (void)get_number(json, key, v);
    return v;
}

inline bool get_bool(const std::string& json, const std::string& key, bool& out) {
    size_t pos = 0;
    if (!find_key(json, key, pos)) return false;
    if (json.compare(pos, 4, "true") == 0) { out = true; return true; }
    if (json.compare(pos, 5, "false") == 0) { out = false; return true; }
    double v = 0.0;
    if (get_number(json, key, v)) { out = v >= 0.5; return true; }
    return false;
}

inline bool get_string(const std::string& json, const std::string& key, std::string& out) {
    size_t pos = 0;
    if (!find_key(json, key, pos) || pos >= json.size() || json[pos] != '"') return false;
    ++pos;
    std::string raw;
    raw.reserve(32);
    bool escaped = false;
    for (; pos < json.size(); ++pos) {
        char c = json[pos];
        if (escaped) {
            raw.push_back('\\');
            raw.push_back(c);
            escaped = false;
            continue;
        }
        if (c == '\\') {
            escaped = true;
            continue;
        }
        if (c == '"') {
            out = unescape(raw);
            return true;
        }
        raw.push_back(c);
    }
    return false;
}

inline std::string get_string_or(const std::string& json, const std::string& key, const std::string& def = {}) {
    std::string v;
    return get_string(json, key, v) ? v : def;
}

class ObjectBuilder {
public:
    void add_number(const std::string& key, double value) {
        std::ostringstream oss;
        oss.precision(10);
        oss << value;
        add_raw(key, oss.str());
    }
    void add_int(const std::string& key, int value) { add_raw(key, std::to_string(value)); }
    void add_bool(const std::string& key, bool value) { add_raw(key, value ? "true" : "false"); }
    void add_string(const std::string& key, const std::string& value) { add_raw(key, "\"" + escape(value) + "\""); }
    void add_raw(const std::string& key, const std::string& value) {
        fields_.push_back("\"" + escape(key) + "\":" + value);
    }
    std::string str() const {
        std::string out = "{";
        for (size_t i = 0; i < fields_.size(); ++i) {
            if (i) out += ',';
            out += fields_[i];
        }
        out += '}';
        return out;
    }
private:
    std::vector<std::string> fields_;
};

}  // namespace minijson
