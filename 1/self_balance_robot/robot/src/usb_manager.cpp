// ═══════════════════════════════════════════════════════════════
//  USB Device Manager — libusb Implementation
//
//  Key design:
//    • Enumerate by VID/PID → no dependency on /dev/ttyACM* naming
//    • Parse USB descriptors to find CDC ACM data interface
//    • Detach kernel cdc_acm driver if attached
//    • Claim CDC data interface, discover bulk IN/OUT endpoints
//    • Async bulk transfers via libusb_submit_transfer + event loop
//    • Hotplug via libusb_hotplug_register_callback
// ═══════════════════════════════════════════════════════════════

#include "usb_manager.h"

#include <cstring>
#include <cstdio>
#include <iostream>
#include <algorithm>

namespace robot {

// ═════════════════════════════════════════════════════════════
//  UsbDevice Implementation
// ═════════════════════════════════════════════════════════════

UsbDevice::UsbDevice() = default;

UsbDevice::~UsbDevice() {
    release();
}

UsbDevice::UsbDevice(UsbDevice&& other) noexcept {
    handle_ = other.handle_; other.handle_ = nullptr;
    ep_ = other.ep_;
    vid_ = other.vid_; pid_ = other.pid_;
    cdc_ctrl_iface_ = other.cdc_ctrl_iface_;
    info_ = std::move(other.info_);
    serial_ = std::move(other.serial_);
    kernel_detached_ = other.kernel_detached_;
}

UsbDevice& UsbDevice::operator=(UsbDevice&& other) noexcept {
    if (this != &other) {
        release();
        handle_ = other.handle_; other.handle_ = nullptr;
        ep_ = other.ep_;
        vid_ = other.vid_; pid_ = other.pid_;
        cdc_ctrl_iface_ = other.cdc_ctrl_iface_;
        info_ = std::move(other.info_);
        serial_ = std::move(other.serial_);
        kernel_detached_ = other.kernel_detached_;
    }
    return *this;
}

void UsbDevice::release() {
    cancel_async();

    if (handle_) {
        if (ep_.iface_num >= 0) {
            libusb_release_interface(handle_, ep_.iface_num);
            if (kernel_detached_) {
                libusb_attach_kernel_driver(handle_, ep_.iface_num);
            }
        }
        if (cdc_ctrl_iface_ >= 0 && cdc_ctrl_iface_ != ep_.iface_num) {
            libusb_release_interface(handle_, cdc_ctrl_iface_);
            if (kernel_detached_) {
                libusb_attach_kernel_driver(handle_, cdc_ctrl_iface_);
            }
        }
        libusb_close(handle_);
        handle_ = nullptr;
    }
}

// ── Synchronous Bulk I/O ─────────────────────────────────────

int UsbDevice::bulk_write(const uint8_t* data, int length, int timeout_ms) {
    if (!handle_ || ep_.bulk_out == 0) return -1;
    int transferred = 0;
    int rc = libusb_bulk_transfer(handle_, ep_.bulk_out,
                                  const_cast<uint8_t*>(data), length,
                                  &transferred, timeout_ms);
    if (rc == LIBUSB_SUCCESS || rc == LIBUSB_ERROR_TIMEOUT) {
        return transferred;
    }
    std::cerr << "[USB " << info_ << "] bulk_write error: "
              << libusb_strerror(static_cast<libusb_error>(rc)) << "\n";
    return -1;
}

int UsbDevice::bulk_read(uint8_t* data, int max_length, int timeout_ms) {
    if (!handle_ || ep_.bulk_in == 0) return -1;
    int transferred = 0;
    int rc = libusb_bulk_transfer(handle_, ep_.bulk_in,
                                  data, max_length,
                                  &transferred, timeout_ms);
    if (rc == LIBUSB_SUCCESS || rc == LIBUSB_ERROR_TIMEOUT) {
        return transferred;
    }
    if (rc == LIBUSB_ERROR_NO_DEVICE || rc == LIBUSB_ERROR_IO) {
        return -1;  // device disconnected
    }
    return -1;
}

bool UsbDevice::write_string(const std::string& str, int timeout_ms) {
    int n = bulk_write(reinterpret_cast<const uint8_t*>(str.data()),
                       static_cast<int>(str.size()), timeout_ms);
    return n == static_cast<int>(str.size());
}

std::string UsbDevice::read_line(int timeout_ms) {
    std::string result;
    uint8_t buf[512];
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        int tmo = std::max(1, static_cast<int>(remaining.count()));

        int n = bulk_read(buf, sizeof(buf), tmo);
        if (n <= 0) {
            if (!result.empty()) return result;
            continue;
        }

        for (int i = 0; i < n; ++i) {
            if (buf[i] == '\n') {
                return result;
            }
            if (buf[i] != '\r') {
                result += static_cast<char>(buf[i]);
            }
        }
    }
    return result;
}

// ── Asynchronous Bulk I/O ────────────────────────────────────

struct AsyncRxContext {
    UsbDevice* dev;
    UsbDevice::RxCallback callback;
};

struct AsyncTxContext {
    UsbDevice* dev;
    UsbDevice::TxCallback callback;
};

void LIBUSB_CALL UsbDevice::rx_callback_trampoline(libusb_transfer* xfr) {
    auto* ctx = static_cast<AsyncRxContext*>(xfr->user_data);

    // Always call user callback with status, let user decide re-submit
    if (ctx->callback) {
        ctx->callback(xfr->buffer, xfr->actual_length, xfr->status);
    }

    ctx->dev->remove_transfer(xfr);
    delete ctx;
    libusb_free_transfer(xfr);
}

void LIBUSB_CALL UsbDevice::tx_callback_trampoline(libusb_transfer* xfr) {
    auto* ctx = static_cast<AsyncTxContext*>(xfr->user_data);

    if (ctx->callback) {
        ctx->callback(xfr->status, xfr->actual_length);
    }

    ctx->dev->remove_transfer(xfr);
    delete ctx;
    // Free the copied buffer
    delete[] xfr->buffer;
    libusb_free_transfer(xfr);
}

bool UsbDevice::async_read(uint8_t* buffer, int max_length,
                            RxCallback callback, int timeout_ms) {
    if (!handle_ || ep_.bulk_in == 0) return false;

    libusb_transfer* xfr = libusb_alloc_transfer(0);
    if (!xfr) return false;

    auto* ctx = new AsyncRxContext{this, std::move(callback)};

    libusb_fill_bulk_transfer(xfr, handle_, ep_.bulk_in,
                               buffer, max_length,
                               rx_callback_trampoline, ctx,
                               timeout_ms);

    int rc = libusb_submit_transfer(xfr);
    if (rc != LIBUSB_SUCCESS) {
        delete ctx;
        libusb_free_transfer(xfr);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(async_mtx_);
        active_transfers_.push_back(xfr);
    }
    return true;
}

bool UsbDevice::async_write(const uint8_t* data, int length,
                             TxCallback callback, int timeout_ms) {
    if (!handle_ || ep_.bulk_out == 0) return false;

    libusb_transfer* xfr = libusb_alloc_transfer(0);
    if (!xfr) return false;

    // Copy data — transfer takes ownership
    uint8_t* buf_copy = new uint8_t[length];
    std::memcpy(buf_copy, data, length);

    auto* ctx = new AsyncTxContext{this, std::move(callback)};

    libusb_fill_bulk_transfer(xfr, handle_, ep_.bulk_out,
                               buf_copy, length,
                               tx_callback_trampoline, ctx,
                               timeout_ms);

    int rc = libusb_submit_transfer(xfr);
    if (rc != LIBUSB_SUCCESS) {
        delete ctx;
        delete[] buf_copy;
        libusb_free_transfer(xfr);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(async_mtx_);
        active_transfers_.push_back(xfr);
    }
    return true;
}

void UsbDevice::cancel_async() {
    {
        std::lock_guard<std::mutex> lock(async_mtx_);
        detaching_.store(true);
        for (auto* xfr : active_transfers_) {
            libusb_cancel_transfer(xfr);
        }
    }

    // Wait for all cancelled transfers to complete their callbacks
    {
        std::unique_lock<std::mutex> lock(async_mtx_);
        async_cv_.wait_for(lock, std::chrono::seconds(2), [this] {
            return active_transfers_.empty();
        });
    }
    detaching_.store(false);
}

void UsbDevice::remove_transfer(libusb_transfer* xfr) {
    std::lock_guard<std::mutex> lock(async_mtx_);
    active_transfers_.erase(
        std::remove(active_transfers_.begin(), active_transfers_.end(), xfr),
        active_transfers_.end());
    async_cv_.notify_all();
}

// ── CDC Line Coding ──────────────────────────────────────────

bool UsbDevice::set_line_coding(uint32_t baudrate, uint8_t data_bits,
                                 uint8_t stop_bits, uint8_t parity) {
    if (!handle_ || cdc_ctrl_iface_ < 0) return false;

    // CDC SET_LINE_CODING: bmRequestType=0x21, bRequest=0x20
    // Payload: 7 bytes (dwDTERate:4, bCharFormat:1, bParityType:1, bDataBits:1)
    uint8_t line_coding[7];
    line_coding[0] = baudrate & 0xFF;
    line_coding[1] = (baudrate >> 8) & 0xFF;
    line_coding[2] = (baudrate >> 16) & 0xFF;
    line_coding[3] = (baudrate >> 24) & 0xFF;
    line_coding[4] = stop_bits;   // 0=1stop, 1=1.5stop, 2=2stop
    line_coding[5] = parity;      // 0=None, 1=Odd, 2=Even
    line_coding[6] = data_bits;

    int rc = libusb_control_transfer(handle_,
        0x21,                           // bmRequestType: host→device, class, interface
        0x20,                           // bRequest: SET_LINE_CODING
        0,                              // wValue
        static_cast<uint16_t>(cdc_ctrl_iface_),  // wIndex: interface
        line_coding, sizeof(line_coding),
        1000);

    return rc == sizeof(line_coding);
}

bool UsbDevice::set_control_line_state(bool dtr, bool rts) {
    if (!handle_ || cdc_ctrl_iface_ < 0) return false;

    // CDC SET_CONTROL_LINE_STATE: bmRequestType=0x21, bRequest=0x22
    uint16_t value = (dtr ? 0x01 : 0x00) | (rts ? 0x02 : 0x00);

    int rc = libusb_control_transfer(handle_,
        0x21, 0x22, value,
        static_cast<uint16_t>(cdc_ctrl_iface_),
        nullptr, 0, 1000);

    return rc >= 0;
}

// ═════════════════════════════════════════════════════════════
//  UsbManager Implementation
// ═════════════════════════════════════════════════════════════

UsbManager::UsbManager() = default;

UsbManager::~UsbManager() {
    shutdown();
}

bool UsbManager::init() {
    int rc = libusb_init(&ctx_);
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "[UsbManager] libusb_init failed: "
                  << libusb_strerror(static_cast<libusb_error>(rc)) << "\n";
        return false;
    }

    // Set log level in debug builds
#ifdef DEBUG
    libusb_set_option(ctx_, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#endif

    std::cout << "[UsbManager] libusb initialized (API "
              << libusb_get_version()->major << "."
              << libusb_get_version()->minor << "."
              << libusb_get_version()->micro << ")\n";

    // Check hotplug capability
    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        std::cout << "[UsbManager] Hotplug supported\n";
    } else {
        std::cout << "[UsbManager] WARNING: No hotplug support, "
                     "using polling fallback\n";
    }

    return true;
}

void UsbManager::shutdown() {
    stop_event_loop();

    // Deregister hotplug callbacks
    {
        std::lock_guard<std::mutex> lock(hotplug_mtx_);
        for (auto& reg : hotplug_regs_) {
            libusb_hotplug_deregister_callback(ctx_, reg.handle);
        }
        hotplug_regs_.clear();
    }

    if (ctx_) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }
}

// ── Enumeration ──────────────────────────────────────────────

std::vector<UsbManager::DeviceInfo> UsbManager::list_devices() {
    std::vector<DeviceInfo> result;
    if (!ctx_) return result;

    libusb_device** dev_list;
    ssize_t count = libusb_get_device_list(ctx_, &dev_list);
    if (count < 0) return result;

    for (ssize_t i = 0; i < count; ++i) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev_list[i], &desc) != 0) continue;

        DeviceInfo info;
        info.vid = desc.idVendor;
        info.pid = desc.idProduct;
        info.bus = libusb_get_bus_number(dev_list[i]);
        info.port = libusb_get_port_number(dev_list[i]);

        // Try to read string descriptors
        libusb_device_handle* h;
        if (libusb_open(dev_list[i], &h) == 0) {
            char buf[256];
            if (desc.iManufacturer &&
                libusb_get_string_descriptor_ascii(h, desc.iManufacturer,
                    reinterpret_cast<uint8_t*>(buf), sizeof(buf)) > 0) {
                info.manufacturer = buf;
            }
            if (desc.iProduct &&
                libusb_get_string_descriptor_ascii(h, desc.iProduct,
                    reinterpret_cast<uint8_t*>(buf), sizeof(buf)) > 0) {
                info.product = buf;
            }
            if (desc.iSerialNumber &&
                libusb_get_string_descriptor_ascii(h, desc.iSerialNumber,
                    reinterpret_cast<uint8_t*>(buf), sizeof(buf)) > 0) {
                info.serial = buf;
            }
            libusb_close(h);
        }

        result.push_back(std::move(info));
    }

    libusb_free_device_list(dev_list, 1);
    return result;
}

// ── CDC Endpoint Discovery ───────────────────────────────────

bool UsbManager::discover_cdc_endpoints(libusb_device* dev,
                                         UsbEndpoints& ep,
                                         int& cdc_ctrl_iface) {
    struct libusb_config_descriptor* config;
    if (libusb_get_active_config_descriptor(dev, &config) != 0) {
        // Try config 0 as fallback
        if (libusb_get_config_descriptor(dev, 0, &config) != 0) {
            return false;
        }
    }

    cdc_ctrl_iface = -1;
    ep.iface_num = -1;
    ep.bulk_in = 0;
    ep.bulk_out = 0;

    // Strategy: scan all interfaces for CDC Data class (0x0A) with
    // bulk endpoints. Also note the CDC Control interface (0x02/0x02)
    // for SET_LINE_CODING.
    for (int i = 0; i < config->bNumInterfaces; ++i) {
        const libusb_interface& iface = config->interface[i];
        for (int j = 0; j < iface.num_altsetting; ++j) {
            const libusb_interface_descriptor& alt = iface.altsetting[j];

            // CDC Control interface (for control requests)
            if (alt.bInterfaceClass == USB_CLASS_CDC &&
                alt.bInterfaceSubClass == USB_SUBCLASS_ACM) {
                cdc_ctrl_iface = alt.bInterfaceNumber;
            }

            // CDC Data interface (has bulk endpoints)
            if (alt.bInterfaceClass == USB_CLASS_CDC_DATA ||
                // Some devices (like ODrive) use vendor-specific class
                // but still have bulk endpoints — fall through to EP scan
                (alt.bNumEndpoints >= 2)) {

                bool found_in = false, found_out = false;

                for (int k = 0; k < alt.bNumEndpoints; ++k) {
                    const libusb_endpoint_descriptor& epd = alt.endpoint[k];

                    // Only bulk endpoints
                    if ((epd.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
                         != LIBUSB_TRANSFER_TYPE_BULK) {
                        continue;
                    }

                    if (epd.bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                        ep.bulk_in = epd.bEndpointAddress;
                        ep.max_packet_in = epd.wMaxPacketSize;
                        found_in = true;
                    } else {
                        ep.bulk_out = epd.bEndpointAddress;
                        ep.max_packet_out = epd.wMaxPacketSize;
                        found_out = true;
                    }
                }

                if (found_in && found_out) {
                    ep.iface_num = alt.bInterfaceNumber;
                    libusb_free_config_descriptor(config);
                    return true;
                }
            }
        }
    }

    // Fallback: scan ALL interfaces for any bulk endpoint pair
    // (handles non-standard CDC implementations)
    for (int i = 0; i < config->bNumInterfaces; ++i) {
        const libusb_interface& iface = config->interface[i];
        for (int j = 0; j < iface.num_altsetting; ++j) {
            const libusb_interface_descriptor& alt = iface.altsetting[j];

            uint8_t found_bulk_in = 0, found_bulk_out = 0;
            uint16_t pkt_in = 0, pkt_out = 0;

            for (int k = 0; k < alt.bNumEndpoints; ++k) {
                const libusb_endpoint_descriptor& epd = alt.endpoint[k];
                if ((epd.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
                     != LIBUSB_TRANSFER_TYPE_BULK) continue;

                if (epd.bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                    found_bulk_in = epd.bEndpointAddress;
                    pkt_in = epd.wMaxPacketSize;
                } else {
                    found_bulk_out = epd.bEndpointAddress;
                    pkt_out = epd.wMaxPacketSize;
                }
            }

            if (found_bulk_in && found_bulk_out) {
                ep.bulk_in = found_bulk_in;
                ep.bulk_out = found_bulk_out;
                ep.max_packet_in = pkt_in;
                ep.max_packet_out = pkt_out;
                ep.iface_num = alt.bInterfaceNumber;
                libusb_free_config_descriptor(config);
                return true;
            }
        }
    }

    libusb_free_config_descriptor(config);
    return false;
}

// ── Open Device ──────────────────────────────────────────────

std::unique_ptr<UsbDevice> UsbManager::open_device(uint16_t vid, uint16_t pid) {
    if (!ctx_) return nullptr;

    libusb_device** dev_list;
    ssize_t count = libusb_get_device_list(ctx_, &dev_list);
    if (count < 0) return nullptr;

    libusb_device* target = nullptr;
    struct libusb_device_descriptor target_desc{};

    for (ssize_t i = 0; i < count; ++i) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev_list[i], &desc) != 0) continue;

        if (desc.idVendor == vid && desc.idProduct == pid) {
            target = dev_list[i];
            target_desc = desc;
            break;
        }
    }

    if (!target) {
        libusb_free_device_list(dev_list, 1);
        return nullptr;
    }

    // Discover endpoints before opening
    UsbEndpoints ep;
    int cdc_ctrl_iface = -1;
    if (!discover_cdc_endpoints(target, ep, cdc_ctrl_iface)) {
        std::cerr << "[UsbManager] No bulk endpoints found for "
                  << std::hex << vid << ":" << pid << std::dec << "\n";
        libusb_free_device_list(dev_list, 1);
        return nullptr;
    }

    // Open device
    libusb_device_handle* handle;
    int rc = libusb_open(target, &handle);
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "[UsbManager] Cannot open "
                  << std::hex << vid << ":" << pid << std::dec
                  << ": " << libusb_strerror(static_cast<libusb_error>(rc))
                  << "\n";
        libusb_free_device_list(dev_list, 1);
        return nullptr;
    }

    auto dev = std::make_unique<UsbDevice>();
    dev->handle_ = handle;
    dev->ep_ = ep;
    dev->vid_ = vid;
    dev->pid_ = pid;
    dev->cdc_ctrl_iface_ = cdc_ctrl_iface;

    // Read string descriptors
    char buf[256];
    std::string product_str, mfg_str;
    if (target_desc.iProduct &&
        libusb_get_string_descriptor_ascii(handle, target_desc.iProduct,
            reinterpret_cast<uint8_t*>(buf), sizeof(buf)) > 0) {
        product_str = buf;
    }
    if (target_desc.iManufacturer &&
        libusb_get_string_descriptor_ascii(handle, target_desc.iManufacturer,
            reinterpret_cast<uint8_t*>(buf), sizeof(buf)) > 0) {
        mfg_str = buf;
    }
    if (target_desc.iSerialNumber &&
        libusb_get_string_descriptor_ascii(handle, target_desc.iSerialNumber,
            reinterpret_cast<uint8_t*>(buf), sizeof(buf)) > 0) {
        dev->serial_ = buf;
    }

    char info_buf[256];
    snprintf(info_buf, sizeof(info_buf), "%04X:%04X %s %s (bus %d)",
             vid, pid, mfg_str.c_str(), product_str.c_str(),
             libusb_get_bus_number(target));
    dev->info_ = info_buf;

    libusb_free_device_list(dev_list, 1);

    // Detach kernel driver if attached (cdc_acm module)
    auto detach_if_needed = [&](int iface) {
        if (iface < 0) return;
        if (libusb_kernel_driver_active(handle, iface) == 1) {
            rc = libusb_detach_kernel_driver(handle, iface);
            if (rc == LIBUSB_SUCCESS) {
                dev->kernel_detached_ = true;
                std::cout << "[UsbManager] Detached kernel driver from iface "
                          << iface << "\n";
            } else {
                std::cerr << "[UsbManager] Cannot detach kernel driver: "
                          << libusb_strerror(static_cast<libusb_error>(rc))
                          << "\n";
            }
        }
    };

    // Detach from both CDC control and data interfaces
    detach_if_needed(cdc_ctrl_iface);
    detach_if_needed(ep.iface_num);

    // Claim CDC data interface
    rc = libusb_claim_interface(handle, ep.iface_num);
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "[UsbManager] Cannot claim interface " << ep.iface_num
                  << ": " << libusb_strerror(static_cast<libusb_error>(rc))
                  << "\n";
        dev->release();
        return nullptr;
    }

    // Claim CDC control interface if different
    if (cdc_ctrl_iface >= 0 && cdc_ctrl_iface != ep.iface_num) {
        rc = libusb_claim_interface(handle, cdc_ctrl_iface);
        if (rc != LIBUSB_SUCCESS) {
            std::cerr << "[UsbManager] WARNING: Cannot claim CDC ctrl iface "
                      << cdc_ctrl_iface << " (SET_LINE_CODING may fail)\n";
            dev->cdc_ctrl_iface_ = -1;  // disable CDC control
        }
    }

    std::cout << "[UsbManager] Opened: " << dev->info_
              << "\n             EP_IN=0x" << std::hex
              << static_cast<int>(ep.bulk_in)
              << " EP_OUT=0x" << static_cast<int>(ep.bulk_out)
              << std::dec
              << " MaxPkt=" << ep.max_packet_in << "/"
              << ep.max_packet_out
              << " DataIface=" << ep.iface_num
              << " CtrlIface=" << cdc_ctrl_iface << "\n";

    return dev;
}

// ── Hotplug ──────────────────────────────────────────────────

int LIBUSB_CALL UsbManager::hotplug_trampoline(libusb_context*,
                                                libusb_device* dev,
                                                libusb_hotplug_event event,
                                                void* user_data) {
    auto* mgr = static_cast<UsbManager*>(user_data);

    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev, &desc) != 0) return 0;

    HotplugEvent ev = (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
                    ? HotplugEvent::ATTACHED
                    : HotplugEvent::DETACHED;

    const char* ev_str = (ev == HotplugEvent::ATTACHED) ? "ATTACHED" : "DETACHED";
    std::cout << "[UsbManager] Hotplug: " << ev_str
              << " " << std::hex << desc.idVendor << ":"
              << desc.idProduct << std::dec << "\n";

    // Queue event for dispatch thread (don't block libusb event loop)
    {
        std::lock_guard<std::mutex> lock(mgr->hotplug_queue_mtx_);
        mgr->hotplug_queue_.push_back({ev, desc.idVendor, desc.idProduct});
    }
    mgr->hotplug_cv_.notify_one();

    return 0;  // keep registration active
}

bool UsbManager::register_hotplug(uint16_t vid, uint16_t pid,
                                   HotplugCallback cb) {
    if (!ctx_) return false;

    if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        std::cerr << "[UsbManager] Hotplug not supported on this platform\n";
        return false;
    }

    HotplugReg reg;
    reg.vid = vid;
    reg.pid = pid;
    reg.callback = std::move(cb);

    int rc = libusb_hotplug_register_callback(
        ctx_,
        static_cast<libusb_hotplug_event>(
            LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
            LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
        LIBUSB_HOTPLUG_NO_FLAGS,
        vid, pid,
        LIBUSB_HOTPLUG_MATCH_ANY,  // match any device class
        hotplug_trampoline,
        this,
        &reg.handle);

    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "[UsbManager] Hotplug registration failed: "
                  << libusb_strerror(static_cast<libusb_error>(rc)) << "\n";
        return false;
    }

    std::lock_guard<std::mutex> lock(hotplug_mtx_);
    hotplug_regs_.push_back(std::move(reg));

    std::cout << "[UsbManager] Hotplug registered for "
              << std::hex << vid << ":" << pid << std::dec << "\n";
    return true;
}

// ── Event Loop ───────────────────────────────────────────────

void UsbManager::start_event_loop() {
    if (event_loop_running_.load()) return;
    event_loop_running_.store(true);
    event_thread_ = std::thread(&UsbManager::event_loop, this);

    // Start hotplug dispatch thread
    hotplug_dispatch_running_.store(true);
    hotplug_dispatch_thread_ = std::thread([this] {
        while (hotplug_dispatch_running_.load()) {
            std::vector<HotplugEvent_t> events;
            {
                std::unique_lock<std::mutex> lock(hotplug_queue_mtx_);
                hotplug_cv_.wait_for(lock, std::chrono::milliseconds(200),
                    [this] { return !hotplug_queue_.empty() ||
                             !hotplug_dispatch_running_.load(); });
                events.swap(hotplug_queue_);
            }
            for (auto& evt : events) {
                std::lock_guard<std::mutex> lock(hotplug_mtx_);
                for (auto& reg : hotplug_regs_) {
                    if (reg.vid == evt.vid && reg.pid == evt.pid && reg.callback) {
                        reg.callback(evt.event, evt.vid, evt.pid);
                    }
                }
            }
        }
    });

    std::cout << "[UsbManager] Event loop started\n";
}

void UsbManager::stop_event_loop() {
    event_loop_running_.store(false);
    if (event_thread_.joinable()) {
        event_thread_.join();
    }

    // Stop hotplug dispatch thread
    hotplug_dispatch_running_.store(false);
    hotplug_cv_.notify_all();
    if (hotplug_dispatch_thread_.joinable()) {
        hotplug_dispatch_thread_.join();
    }
}

void UsbManager::event_loop() {
    struct timeval tv;
    while (event_loop_running_.load()) {
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms
        int rc = libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
        if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_TIMEOUT) {
            if (rc == LIBUSB_ERROR_INTERRUPTED) continue;
            std::cerr << "[UsbManager] Event loop error: "
                      << libusb_strerror(static_cast<libusb_error>(rc)) << "\n";
        }
    }
}

void UsbManager::handle_events(int timeout_ms) {
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
}

} // namespace robot
