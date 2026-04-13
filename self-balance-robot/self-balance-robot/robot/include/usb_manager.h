#pragma once
// ═══════════════════════════════════════════════════════════════
//  USB Device Manager — libusb Enumeration, Hotplug, CDC Bulk I/O
//
//  Responsibilities:
//    1. Initialize libusb context
//    2. Enumerate USB devices by VID/PID
//    3. Auto-detect CDC ACM data interface & bulk endpoints
//    4. Hotplug monitoring (attach/detach callbacks)
//    5. Provide UsbDevice handles with async bulk transfer API
// ═══════════════════════════════════════════════════════════════

#include "types.h"

#include <libusb-1.0/libusb.h>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <memory>

namespace robot {

// ── USB Device Handle ────────────────────────────────────────
// Wraps a claimed libusb device with discovered bulk endpoints.

struct UsbEndpoints {
    uint8_t  bulk_in   = 0;    // EP address for IN (device→host)
    uint8_t  bulk_out  = 0;    // EP address for OUT (host→device)
    int      iface_num = -1;   // CDC data interface number
    uint16_t max_packet_in  = 64;
    uint16_t max_packet_out = 64;
};

class UsbDevice {
public:
    UsbDevice();
    ~UsbDevice();

    UsbDevice(const UsbDevice&) = delete;
    UsbDevice& operator=(const UsbDevice&) = delete;
    UsbDevice(UsbDevice&& other) noexcept;
    UsbDevice& operator=(UsbDevice&& other) noexcept;

    /// Check if device is open and claimed
    bool is_open() const { return handle_ != nullptr; }

    /// Get device info string
    std::string info() const { return info_; }

    /// Get VID/PID
    uint16_t vid() const { return vid_; }
    uint16_t pid() const { return pid_; }

    /// Get serial number
    std::string serial() const { return serial_; }

    // ── Synchronous Bulk I/O ─────────────────────────────────

    /// Write data to bulk OUT endpoint. Returns bytes written or -1.
    int bulk_write(const uint8_t* data, int length, int timeout_ms = 50);

    /// Read data from bulk IN endpoint. Returns bytes read or -1.
    int bulk_read(uint8_t* data, int max_length, int timeout_ms = 50);

    /// Write a string (convenience for ASCII protocol)
    bool write_string(const std::string& str, int timeout_ms = 50);

    /// Read a line (up to newline or timeout). Returns empty on timeout.
    std::string read_line(int timeout_ms = 100);

    // ── Asynchronous Bulk I/O ────────────────────────────────

    /// Callback types
    using RxCallback = std::function<void(const uint8_t* data, int length)>;
    using TxCallback = std::function<void(int status, int bytes_transferred)>;

    /// Submit an async bulk read. Callback fires when data arrives.
    bool async_read(uint8_t* buffer, int max_length,
                    RxCallback callback, int timeout_ms = 0);

    /// Submit an async bulk write. Callback fires on completion.
    bool async_write(const uint8_t* data, int length,
                     TxCallback callback, int timeout_ms = 0);

    /// Cancel all pending async transfers
    void cancel_async();

    // ── CDC Line Coding (baudrate etc.) ──────────────────────

    /// Set CDC line coding (baudrate, 8N1). Needed for some STM32 VCP.
    bool set_line_coding(uint32_t baudrate, uint8_t data_bits = 8,
                         uint8_t stop_bits = 0, uint8_t parity = 0);

    /// Set DTR/RTS control line state
    bool set_control_line_state(bool dtr, bool rts);

    /// Get the libusb device handle (for event loop integration)
    libusb_device_handle* native_handle() const { return handle_; }

private:
    friend class UsbManager;

    libusb_device_handle* handle_ = nullptr;
    UsbEndpoints ep_;
    uint16_t vid_ = 0, pid_ = 0;
    int cdc_ctrl_iface_ = -1;   // CDC control interface (for SET_LINE_CODING)
    std::string info_;
    std::string serial_;
    bool kernel_detached_ = false;

    // Track active async transfers for cancellation
    std::mutex async_mtx_;
    std::vector<libusb_transfer*> active_transfers_;

    void release();
    void remove_transfer(libusb_transfer* xfr);

    // libusb async callback trampolines
    static void LIBUSB_CALL rx_callback_trampoline(libusb_transfer* xfr);
    static void LIBUSB_CALL tx_callback_trampoline(libusb_transfer* xfr);
};

// ── USB Manager ──────────────────────────────────────────────
// Singleton-like manager that owns the libusb context, runs the
// event loop, and provides enumeration + hotplug.

class UsbManager {
public:
    UsbManager();
    ~UsbManager();

    UsbManager(const UsbManager&) = delete;
    UsbManager& operator=(const UsbManager&) = delete;

    /// Initialize libusb
    bool init();

    /// Shutdown: stop event loop, free context
    void shutdown();

    // ── Enumeration ──────────────────────────────────────────

    /// Find and open a device by VID/PID. Returns nullptr if not found.
    /// Automatically discovers CDC bulk endpoints.
    std::unique_ptr<UsbDevice> open_device(uint16_t vid, uint16_t pid);

    /// Find and open by UsbDeviceId
    std::unique_ptr<UsbDevice> open_device(const UsbDeviceId& id) {
        return open_device(id.vid, id.pid);
    }

    /// List all connected USB devices (for diagnostics)
    struct DeviceInfo {
        uint16_t vid, pid;
        uint8_t  bus, port;
        std::string manufacturer;
        std::string product;
        std::string serial;
    };
    std::vector<DeviceInfo> list_devices();

    // ── Hotplug ──────────────────────────────────────────────

    /// Hotplug event types
    enum class HotplugEvent { ATTACHED, DETACHED };

    /// Register hotplug callback for a specific VID/PID.
    /// callback(event, vid, pid) fires on attach/detach.
    using HotplugCallback = std::function<void(HotplugEvent event,
                                                uint16_t vid, uint16_t pid)>;
    bool register_hotplug(uint16_t vid, uint16_t pid, HotplugCallback cb);

    /// Register hotplug for a UsbDeviceId
    bool register_hotplug(const UsbDeviceId& id, HotplugCallback cb) {
        return register_hotplug(id.vid, id.pid, std::move(cb));
    }

    // ── Event Loop ───────────────────────────────────────────

    /// Start background event loop thread (handles async transfers + hotplug)
    void start_event_loop();

    /// Stop event loop
    void stop_event_loop();

    /// Handle events once (for manual integration)
    void handle_events(int timeout_ms = 100);

    /// Get libusb context (for advanced use)
    libusb_context* context() const { return ctx_; }

private:
    libusb_context* ctx_ = nullptr;
    std::atomic<bool> event_loop_running_{false};
    std::thread event_thread_;

    // Hotplug handles
    struct HotplugReg {
        libusb_hotplug_callback_handle handle;
        HotplugCallback callback;
        uint16_t vid, pid;
    };
    std::mutex hotplug_mtx_;
    std::vector<HotplugReg> hotplug_regs_;

    /// Event loop function
    void event_loop();

    /// Discover CDC ACM bulk endpoints from device descriptors
    static bool discover_cdc_endpoints(libusb_device* dev,
                                        UsbEndpoints& ep,
                                        int& cdc_ctrl_iface);

    /// Hotplug callback trampoline
    static int LIBUSB_CALL hotplug_trampoline(libusb_context* ctx,
                                               libusb_device* dev,
                                               libusb_hotplug_event event,
                                               void* user_data);
};

} // namespace robot
