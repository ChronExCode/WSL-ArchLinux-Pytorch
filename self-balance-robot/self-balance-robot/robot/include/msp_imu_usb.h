#pragma once
// ═══════════════════════════════════════════════════════════════
//  MSP Protocol — IMU Reader via libusb Bulk Transfers
//  SpeedyBee F405 V5 running Betaflight, USB CDC bulk endpoints
//
//  Architecture:
//    • MSP requests sent via async bulk OUT
//    • Responses received via continuous async bulk IN pipeline
//    • Background thread sends periodic MSP_ATTITUDE + MSP_RAW_IMU
//    • RX callback parses MSP frames and updates shared ImuData
// ═══════════════════════════════════════════════════════════════

#include "types.h"
#include "usb_manager.h"

#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <memory>
#include <vector>
#include <cstdint>

namespace robot {

// MSP command IDs (Betaflight)
enum class MSPCommand : uint16_t {
    MSP_STATUS       = 101,
    MSP_RAW_IMU      = 102,
    MSP_RC           = 105,
    MSP_ATTITUDE     = 108,
    MSP_ANALOG       = 110,
};

class MspImuUsb {
public:
    MspImuUsb();
    ~MspImuUsb();

    MspImuUsb(const MspImuUsb&) = delete;
    MspImuUsb& operator=(const MspImuUsb&) = delete;

    /// Attach to an opened UsbDevice (takes ownership)
    bool attach(std::unique_ptr<UsbDevice> dev);

    /// Detach
    void detach();

    /// Check connection
    bool is_connected() const { return dev_ && dev_->is_open(); }

    /// Start background polling
    void start_polling(int interval_us = 4000);

    /// Stop polling
    void stop_polling();

    /// Get latest IMU data (thread-safe)
    ImuData get_imu_data() const;

    /// Check data freshness
    bool is_data_fresh(int max_age_ms = 50) const;

    /// Set callback for new IMU data
    void set_callback(std::function<void(const ImuData&)> cb);

    // Accelerometer/gyro scale factors (Betaflight MSP_RAW_IMU)
    static constexpr double ACC_SCALE  = 1.0 / 2048.0;  // raw / 2048 = g (±16g range, confirmed by acc_z ≈ 1g at rest)
    static constexpr double GYRO_SCALE = 1.0 / 16.4;    // raw / 16.4 = deg/s (±2000 dps)

private:
    std::unique_ptr<UsbDevice> dev_;

    mutable std::mutex data_mtx_;
    ImuData latest_data_;

    std::atomic<bool> polling_{false};
    std::thread poll_thread_;
    std::function<void(const ImuData&)> callback_;

    // Async RX state
    static constexpr int RX_BUF_SIZE = 256;
    static constexpr int RX_QUEUE_DEPTH = 4;
    struct RxSlot { uint8_t buffer[RX_BUF_SIZE]; };
    RxSlot rx_slots_[RX_QUEUE_DEPTH];
    std::atomic<bool> rx_running_{false};

    // MSP frame parser state machine
    enum class MspState {
        IDLE, HEADER_M, HEADER_DIR,
        PAYLOAD_SIZE, COMMAND, PAYLOAD, CHECKSUM
    };
    struct MspParser {
        MspState state = MspState::IDLE;
        uint8_t  size  = 0;
        uint8_t  cmd   = 0;
        uint8_t  crc   = 0;
        int      payload_idx = 0;
        uint8_t  payload[256];
        bool     is_error = false;

        void reset() {
            state = MspState::IDLE;
            size = cmd = crc = 0;
            payload_idx = 0;
            is_error = false;
        }
    };
    MspParser parser_;
    std::mutex parser_mtx_;

    // ── MSP Framing ──────────────────────────────────────────

    /// Build MSP v1 request: $M< <0> <cmd> <crc>
    static std::vector<uint8_t> build_request(MSPCommand cmd);

    /// Feed received bytes into the MSP parser
    void feed_parser(const uint8_t* data, int length);

    /// Called when a complete MSP frame is parsed
    void on_msp_frame(uint8_t cmd, const uint8_t* payload, int size);

    /// Parse MSP_ATTITUDE
    void parse_attitude(const uint8_t* payload, int size);

    /// Parse MSP_RAW_IMU
    void parse_raw_imu(const uint8_t* payload, int size);

    // ── Async I/O ────────────────────────────────────────────

    void submit_async_read(int slot_index);
    void start_rx_pipeline();
    void stop_rx_pipeline();

    /// Send MSP request via async bulk write
    bool send_msp_request(MSPCommand cmd);

    /// Polling thread function
    void poll_loop(int interval_us);
};

} // namespace robot
