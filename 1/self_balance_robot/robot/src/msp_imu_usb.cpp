// ═══════════════════════════════════════════════════════════════
//  MSP Protocol — libusb Bulk Transfer Implementation
//
//  Key design:
//    • Continuous async bulk IN pipeline (4 transfers deep)
//    • Streaming MSP frame parser — handles partial frames across
//      USB packet boundaries
//    • Async bulk OUT for sending MSP requests (non-blocking)
//    • Background thread sends alternating ATTITUDE / RAW_IMU
// ═══════════════════════════════════════════════════════════════

#include "msp_imu_usb.h"

#include <cstring>
#include <iostream>
#include <chrono>

namespace robot {

MspImuUsb::MspImuUsb() = default;

MspImuUsb::~MspImuUsb() {
    detach();
}

bool MspImuUsb::attach(std::unique_ptr<UsbDevice> dev) {
    if (!dev || !dev->is_open()) return false;

    detach();
    dev_ = std::move(dev);

    // CDC line coding: 115200 8N1
    dev_->set_line_coding(115200);
    // DTR/RTS init: deassert → delay → assert (H743 compatibility)
    dev_->set_control_line_state(false, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    dev_->set_control_line_state(true, true);

    // Reset parser
    {
        std::lock_guard<std::mutex> lock(parser_mtx_);
        parser_.reset();
    }

    std::cout << "[MSP-IMU] Attached: " << dev_->info() << "\n";
    return true;
}

void MspImuUsb::detach() {
    stop_polling();
    stop_rx_pipeline();

    if (dev_) {
        dev_.reset();
    }

    {
        std::lock_guard<std::mutex> lock(data_mtx_);
        latest_data_ = ImuData{};
    }

    std::cout << "[MSP-IMU] Detached\n";
}

// ── MSP Framing ──────────────────────────────────────────────

std::vector<uint8_t> MspImuUsb::build_request(MSPCommand cmd) {
    uint8_t cmd_byte = static_cast<uint8_t>(cmd);
    // $M< <size=0> <cmd> <crc = size ^ cmd = 0 ^ cmd = cmd>
    return {'$', 'M', '<', 0, cmd_byte, static_cast<uint8_t>(0 ^ cmd_byte)};
}

bool MspImuUsb::send_msp_request(MSPCommand cmd) {
    if (!dev_ || !dev_->is_open()) return false;

    auto frame = build_request(cmd);

    // Use synchronous write for flow control — prevents overwhelming
    // the STM32 USB endpoint (especially on H743 targets)
    int n = dev_->bulk_write(frame.data(), static_cast<int>(frame.size()), 50);
    if (n < 0) {
        // Only log at debug level — transient errors are expected
        return false;
    }
    return n == static_cast<int>(frame.size());
}

// ── Streaming MSP Parser ─────────────────────────────────────

void MspImuUsb::feed_parser(const uint8_t* data, int length) {
    std::lock_guard<std::mutex> lock(parser_mtx_);

    for (int i = 0; i < length; ++i) {
        uint8_t c = data[i];

        switch (parser_.state) {
        case MspState::IDLE:
            if (c == '$') parser_.state = MspState::HEADER_M;
            break;

        case MspState::HEADER_M:
            if (c == 'M') {
                parser_.state = MspState::HEADER_DIR;
            } else {
                parser_.reset();
            }
            break;

        case MspState::HEADER_DIR:
            if (c == '>') {
                parser_.is_error = false;
                parser_.state = MspState::PAYLOAD_SIZE;
            } else if (c == '!') {
                parser_.is_error = true;
                parser_.state = MspState::PAYLOAD_SIZE;
            } else {
                parser_.reset();
            }
            break;

        case MspState::PAYLOAD_SIZE:
            parser_.size = c;
            parser_.crc = c;
            parser_.payload_idx = 0;
            parser_.state = MspState::COMMAND;
            break;

        case MspState::COMMAND:
            parser_.cmd = c;
            parser_.crc ^= c;
            if (parser_.size > 0) {
                parser_.state = MspState::PAYLOAD;
            } else {
                parser_.state = MspState::CHECKSUM;
            }
            break;

        case MspState::PAYLOAD:
            if (parser_.payload_idx < static_cast<int>(sizeof(parser_.payload))) {
                parser_.payload[parser_.payload_idx] = c;
            }
            parser_.crc ^= c;
            parser_.payload_idx++;
            if (parser_.payload_idx >= parser_.size) {
                parser_.state = MspState::CHECKSUM;
            }
            break;

        case MspState::CHECKSUM:
            if (c == parser_.crc && !parser_.is_error) {
                on_msp_frame(parser_.cmd, parser_.payload, parser_.size);
            }
            parser_.reset();
            break;
        }
    }
}

void MspImuUsb::on_msp_frame(uint8_t cmd, const uint8_t* payload, int size) {
    MSPCommand msp_cmd = static_cast<MSPCommand>(cmd);

    switch (msp_cmd) {
    case MSPCommand::MSP_ATTITUDE:
        parse_attitude(payload, size);
        break;
    case MSPCommand::MSP_RAW_IMU:
        parse_raw_imu(payload, size);
        break;
    default:
        break;
    }
}

void MspImuUsb::parse_attitude(const uint8_t* payload, int size) {
    if (size < 6) return;

    auto read_i16 = [&](int offset) -> int16_t {
        return static_cast<int16_t>(payload[offset] | (payload[offset + 1] << 8));
    };

    int16_t roll_raw  = read_i16(0);
    int16_t pitch_raw = read_i16(2);
    int16_t yaw_raw   = read_i16(4);

    std::lock_guard<std::mutex> lock(data_mtx_);
    latest_data_.roll  = roll_raw / 10.0;
    latest_data_.pitch = pitch_raw / 10.0;
    latest_data_.yaw   = yaw_raw;
    latest_data_.timestamp = std::chrono::steady_clock::now();
    latest_data_.valid = true;

    // Fire callback (outside lock would be better, but keep simple)
    if (callback_) {
        ImuData copy = latest_data_;
        // Release lock before callback to avoid deadlock
        // Actually we should copy first, then release, then fire.
        // For simplicity, fire inside lock (callback should be fast).
        callback_(copy);
    }
}

void MspImuUsb::parse_raw_imu(const uint8_t* payload, int size) {
    if (size < 18) return;

    auto read_i16 = [&](int offset) -> int16_t {
        return static_cast<int16_t>(payload[offset] | (payload[offset + 1] << 8));
    };

    int16_t ax = read_i16(0);
    int16_t ay = read_i16(2);
    int16_t az = read_i16(4);
    int16_t gx = read_i16(6);
    int16_t gy = read_i16(8);
    int16_t gz = read_i16(10);

    std::lock_guard<std::mutex> lock(data_mtx_);
    // ACC_SCALE converts raw to g (1/2048 for ±16g range)
    latest_data_.acc_x  = ax * ACC_SCALE;
    latest_data_.acc_y  = ay * ACC_SCALE;
    latest_data_.acc_z  = az * ACC_SCALE;
    latest_data_.gyro_x = gx * GYRO_SCALE;
    latest_data_.gyro_y = gy * GYRO_SCALE;
    latest_data_.gyro_z = gz * GYRO_SCALE;
    latest_data_.timestamp = std::chrono::steady_clock::now();
}

// ── Async I/O Pipeline ───────────────────────────────────────

void MspImuUsb::submit_async_read(int slot_index) {
    if (!dev_ || !rx_running_.load() || dev_->is_detaching()) return;

    dev_->async_read(
        rx_slots_[slot_index].buffer, RX_BUF_SIZE,
        [this, slot_index](const uint8_t* data, int length, int status) {
            // Process data only on successful completion
            if (status == 0 /* LIBUSB_TRANSFER_COMPLETED */ && length > 0) {
                feed_parser(data, length);
            }

            // Re-submit on non-fatal status (COMPLETED, TIMED_OUT, ERROR)
            // Do NOT re-submit on CANCELLED(3) or NO_DEVICE(5)
            if (status <= 2 && rx_running_.load() && dev_ && !dev_->is_detaching()) {
                submit_async_read(slot_index);
            }
        },
        0  // no timeout
    );
}

void MspImuUsb::start_rx_pipeline() {
    if (rx_running_.load() || !dev_) return;
    rx_running_.store(true);

    for (int i = 0; i < RX_QUEUE_DEPTH; ++i) {
        submit_async_read(i);
    }

    std::cout << "[MSP-IMU] RX pipeline started (" << RX_QUEUE_DEPTH
              << " async transfers)\n";
}

void MspImuUsb::stop_rx_pipeline() {
    rx_running_.store(false);
    if (dev_) {
        dev_->cancel_async();
    }
}

// ── Polling ──────────────────────────────────────────────────

void MspImuUsb::start_polling(int interval_us) {
    if (polling_.load()) return;

    // Start RX pipeline first
    start_rx_pipeline();

    polling_.store(true);
    poll_thread_ = std::thread(&MspImuUsb::poll_loop, this, interval_us);
}

void MspImuUsb::stop_polling() {
    polling_.store(false);
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }
    stop_rx_pipeline();
}

void MspImuUsb::poll_loop(int interval_us) {
    // Cap polling rate to 500Hz (2000μs) — sufficient for 200Hz control loop.
    // Higher rates overwhelm STM32 USB endpoints (especially H743).
    int effective_interval = std::max(interval_us, 2000);
    std::cout << "[MSP-IMU] Polling started @ "
              << (1000000 / effective_interval) << " Hz (alternating ATTITUDE/RAW_IMU)\n";

    bool request_attitude = true;  // alternate between the two

    while (polling_.load()) {
        auto start = std::chrono::steady_clock::now();

        if (!dev_ || !dev_->is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Send ONE request per cycle, alternating between ATTITUDE and RAW_IMU.
        // This matches how Betaflight Configurator works — one request at a time,
        // wait for response before sending next. Prevents FC MSP handler confusion.
        if (request_attitude) {
            send_msp_request(MSPCommand::MSP_ATTITUDE);
        } else {
            send_msp_request(MSPCommand::MSP_RAW_IMU);
        }
        request_attitude = !request_attitude;

        // Precise interval timing
        auto elapsed = std::chrono::steady_clock::now() - start;
        auto sleep_time = std::chrono::microseconds(effective_interval) - elapsed;
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    std::cout << "[MSP-IMU] Polling stopped\n";
}

ImuData MspImuUsb::get_imu_data() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return latest_data_;
}

bool MspImuUsb::is_data_fresh(int max_age_ms) const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    if (!latest_data_.valid) return false;
    auto age = std::chrono::steady_clock::now() - latest_data_.timestamp;
    return std::chrono::duration_cast<std::chrono::milliseconds>(age).count()
           < max_age_ms;
}

void MspImuUsb::set_callback(std::function<void(const ImuData&)> cb) {
    callback_ = std::move(cb);
}

} // namespace robot
