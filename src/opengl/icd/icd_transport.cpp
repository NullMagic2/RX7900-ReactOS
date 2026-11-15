/*
Module: rx7900.opengl.icd_transport
Purpose: The radeonglicd byte transport to the native process service: version-3
process requests through ExtEscape on the caller's HDC, and window-clipped GPU
presents through DrawEscape.
Created: 2026-10-23
Architecture: User mode, Win32 GDI only. radeondisp DrvEscape forwards exactly the
caller's request bytes as the miniport's process IOCTL (the kernel authenticates
the calling NT process and validates every field); DrvDrawEscape receives
win32k's CLIPOBJ for the DC and issues the kernel-only present IOCTL with those
rectangles. Neither path carries an address, register or command stream. When
radeondisp lacks the escapes, ExtEscape/DrawEscape return 0 and the backend uses
its software renderer or the readback present.
*/
#include <windows.h>

#include <new>

#include "../gl_gfx11_backend.hpp"

namespace {

namespace wire = rx7900::reactos::graphics_abi;

class EscapeTransport final : public rx7900::opengl::GpuTransport {
public:
    explicit EscapeTransport(HDC dc) noexcept : dc_(dc) {}

    void bind(HDC dc) noexcept { dc_ = dc; }

    bool call(const uint8_t* request, uint32_t request_bytes, uint8_t* reply, uint32_t reply_capacity,
              uint32_t& reply_bytes) noexcept override {
        reply_bytes = 0;
        if (dc_ == nullptr || request_bytes > INT32_MAX || reply_capacity > INT32_MAX) return false;
        const int result = ExtEscape(dc_, static_cast<int>(wire::process_escape), static_cast<int>(request_bytes),
                                     reinterpret_cast<LPCSTR>(request), static_cast<int>(reply_capacity),
                                     reinterpret_cast<LPSTR>(reply));
        if (result <= 0 || reply_capacity < 48) return false;
        // The reply header names its own payload length (process_abi::Reply).
        const uint32_t payload = static_cast<uint32_t>(reply[12]) | (static_cast<uint32_t>(reply[13]) << 8) |
                                 (static_cast<uint32_t>(reply[14]) << 16) | (static_cast<uint32_t>(reply[15]) << 24);
        if (payload > reply_capacity - 48) return false;
        reply_bytes = 48 + payload;
        return true;
    }

    bool present(const uint8_t* input, uint32_t input_bytes, uint32_t& outcome, uint32_t& rects) noexcept override {
        outcome = 0;
        rects = 0;
        if (dc_ == nullptr) return false;
        // radeondisp returns 1 + process_abi::Outcome; 0 means not supported.
        const int result = DrawEscape(dc_, static_cast<int>(wire::present_escape), static_cast<int>(input_bytes),
                                      reinterpret_cast<LPCSTR>(input));
        if (result <= 0) return false;
        outcome = static_cast<uint32_t>(result - 1);
        return true;
    }

    void yield() noexcept override { Sleep(0); }

private:
    HDC dc_;
};

} // namespace

// The ICD's transport factory (tests link a fake-device implementation instead).
rx7900::opengl::GpuTransport* rx7900_icd_create_transport(HDC dc) noexcept {
    void* raw = HeapAlloc(GetProcessHeap(), 0, sizeof(EscapeTransport));
    return raw == nullptr ? nullptr : new (raw) EscapeTransport(dc);
}

void rx7900_icd_bind_transport(rx7900::opengl::GpuTransport* transport, HDC dc) noexcept {
    if (transport != nullptr) static_cast<EscapeTransport*>(transport)->bind(dc);
}

void rx7900_icd_destroy_transport(rx7900::opengl::GpuTransport* transport) noexcept {
    if (transport == nullptr) return;
    static_cast<EscapeTransport*>(transport)->~EscapeTransport();
    HeapFree(GetProcessHeap(), 0, transport);
}
