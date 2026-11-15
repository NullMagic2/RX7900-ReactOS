/*
Module: rx7900.tests.framebuffer_resize_guest
Purpose: Detect leftover sizing outlines and failed repaints through real GDI.
Created: 2026-10-22
Architecture: Owns one temporary window and reuses the common Win32 handle owner.
No host-side rasterizer or substitute display path is used.
*/
#include "windows_support.hpp"
#include "../src/reactos/framebuffer_shadow_abi.hpp"
#include "../src/reactos/lightning_gdi_abi.hpp"
#include <stdio.h>

namespace {
constexpr wchar_t class_name[] = L"LightningResizeRegression";
using Object = rx7900::ScopedHandle<HGDIOBJ, DeleteObject>;
using Window = rx7900::ScopedHandle<HWND, DestroyWindow>;

COLORREF color_at(LONG x, LONG y) noexcept {
    return ((x / 16 + y / 16) & 1) == 0 ? RGB(35, 97, 173) : RGB(218, 157, 49);
}

bool paint_pattern(HDC dc, const RECT& client) noexcept {
    Object first(CreateSolidBrush(color_at(0, 0)));
    Object second(CreateSolidBrush(color_at(16, 0)));
    if (!first.valid() || !second.valid()) return false;
    for (LONG y = 0; y < client.bottom; y += 16) {
        for (LONG x = 0; x < client.right; x += 16) {
            const RECT tile{x, y, x + 16, y + 16};
            const auto brush = reinterpret_cast<HBRUSH>(((x / 16 + y / 16) & 1) == 0 ?
                first.get() : second.get());
            if (FillRect(dc, &tile, brush) == 0) return false;
        }
    }
    return true;
}

LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_PAINT) {
        PAINTSTRUCT paint{};
        HDC dc = BeginPaint(window, &paint);
        RECT client{};
        const bool valid = dc != nullptr && GetClientRect(window, &client) != FALSE &&
            paint_pattern(dc, client);
        EndPaint(window, &paint);
        if (!valid) SetPropW(window, L"PaintFailed", reinterpret_cast<HANDLE>(1));
        return 0;
    }
    return DefWindowProcW(window, message, wparam, lparam);
}

// Match UserDrawWindowFrame in the pinned ReactOS nonclient.c. Negative bottom
// and right extents deliberately exercise the same native sizing-frame calls.
bool outline(HDC dc, RECT frame) noexcept {
    constexpr LONG width = 4;
    constexpr LONG height = 4;
    const HGDIOBJ old = SelectObject(dc, GetStockObject(GRAY_BRUSH));
    if (old == nullptr || old == HGDI_ERROR) return false;
    const bool result =
        PatBlt(dc, frame.left, frame.top, frame.right - frame.left - width, height, PATINVERT) != FALSE &&
        PatBlt(dc, frame.left, frame.top + height, width, frame.bottom - frame.top - height, PATINVERT) != FALSE &&
        PatBlt(dc, frame.left + width, frame.bottom, frame.right - frame.left - width, -height, PATINVERT) != FALSE &&
        PatBlt(dc, frame.right, frame.top, -width, frame.bottom - frame.top - height, PATINVERT) != FALSE;
    SelectObject(dc, old);
    return result;
}

bool capture_pixels(HDC dc, LONG width, LONG height, uint32_t* pixels, size_t capacity) noexcept {
    if (dc == nullptr || width <= 0 || height <= 0 || pixels == nullptr ||
        static_cast<uint64_t>(width) * static_cast<uint32_t>(height) > capacity || !GdiFlush()) return false;
    rx7900::ScopedHandle<HDC, DeleteDC> memory(CreateCompatibleDC(dc));
    Object bitmap(CreateCompatibleBitmap(dc, width, height));
    if (!memory.valid() || !bitmap.valid()) return false;
    const HGDIOBJ previous = SelectObject(memory.get(), bitmap.get());
    if (previous == nullptr || previous == HGDI_ERROR) return false;
    const bool copied = BitBlt(memory.get(), 0, 0, width, height,
        dc, 0, 0, SRCCOPY) != FALSE;
    SelectObject(memory.get(), previous);
    if (!copied) return false;
    BITMAPINFO information{};
    information.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    information.bmiHeader.biWidth = width;
    information.bmiHeader.biHeight = -height;
    information.bmiHeader.biPlanes = 1;
    information.bmiHeader.biBitCount = 32;
    information.bmiHeader.biCompression = BI_RGB;
    return GetDIBits(dc, static_cast<HBITMAP>(bitmap.get()), 0,
        static_cast<UINT>(height), pixels, &information, DIB_RGB_COLORS) == height;
}

bool restored(HDC dc, RECT client, unsigned iteration) noexcept {
    constexpr LONG maximum_width = 384;
    constexpr LONG maximum_height = 256;
    if (client.right <= 0 || client.bottom <= 0 || client.right > maximum_width ||
        client.bottom > maximum_height) return false;
    // Bounded user-mode scratch, outside every driver/kernel callback stack.
    uint32_t pixels[maximum_width * maximum_height]{};
    if (!capture_pixels(dc, client.right, client.bottom, pixels, sizeof(pixels) / sizeof(pixels[0]))) return false;
    // Check every client pixel, including the last row/column. Exact comparison
    // catches single-pixel borders rather than relying on a screenshot sample.
    for (LONG y = 0; y < client.bottom; ++y) {
        for (LONG x = 0; x < client.right; ++x) {
            const uint32_t pixel = pixels[static_cast<size_t>(y) * client.right + x];
            const COLORREF actual = RGB((pixel >> 16) & 255, (pixel >> 8) & 255, pixel & 255);
            if (actual != color_at(x, y)) {
                printf("RESIZE_MISMATCH iteration=%u x=%ld y=%ld expected=%08lx actual=%08lx\n",
                    iteration, x, y, static_cast<unsigned long>(color_at(x, y)),
                    static_cast<unsigned long>(actual));
                fflush(stdout);
                return false;
            }
        }
    }
    return true;
}

bool write_all(HANDLE file, const void* bytes, DWORD size) noexcept {
    const auto* data = static_cast<const unsigned char*>(bytes);
    DWORD offset = 0;
    while (offset < size) {
        DWORD written = 0;
        if (!WriteFile(file, data + offset, size - offset, &written, nullptr) ||
            written == 0 || written > size - offset) return false;
        offset += written;
    }
    return true;
}

BOOL release_pixels(void* pixels) noexcept {
    return HeapFree(GetProcessHeap(), 0, pixels);
}

// Passive primary-desktop capture for canvas/consumer comparison. The GPU
// driver's real screen-to-memory readback supplies these pixels; no repaint,
// cursor movement or window creation is requested by this mode.
bool capture_desktop(const char* path) noexcept {
    const LONG width = GetSystemMetrics(SM_CXSCREEN);
    const LONG height = GetSystemMetrics(SM_CYSCREEN);
    constexpr uint64_t maximum_pixels = 4096ULL * 4096;
    if (width <= 0 || height <= 0) return false;
    const uint64_t count = static_cast<uint64_t>(width) * static_cast<uint32_t>(height);
    if (count > maximum_pixels) return false;
    // HeapAlloc supplies natural alignment and this bounded complete extent.
    // Reuse the common scoped owner without adding a native C++ heap runtime.
    rx7900::ScopedHandle<void*, release_pixels> pixels(HeapAlloc(GetProcessHeap(),
        HEAP_ZERO_MEMORY, static_cast<SIZE_T>(count * sizeof(uint32_t))));
    if (!pixels.valid()) return false;
    HDC dc = GetDC(nullptr);
    const bool captured = capture_pixels(dc, width, height, static_cast<uint32_t*>(pixels.get()),
        static_cast<size_t>(count));
    if (dc != nullptr) ReleaseDC(nullptr, dc);
    if (!captured) return false;
    BITMAPFILEHEADER file_header{};
    BITMAPINFOHEADER image{};
    image.biSize = sizeof(image);
    image.biWidth = width;
    image.biHeight = -height;
    image.biPlanes = 1;
    image.biBitCount = 32;
    image.biCompression = BI_RGB;
    image.biSizeImage = static_cast<DWORD>(count * sizeof(uint32_t));
    file_header.bfType = 0x4d42;
    file_header.bfOffBits = sizeof(file_header) + sizeof(image);
    file_header.bfSize = file_header.bfOffBits + image.biSizeImage;
    rx7900::DeviceHandle file(CreateFileA(path, GENERIC_WRITE, 0, nullptr,
        CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr), INVALID_HANDLE_VALUE);
    const bool result = file.valid() && write_all(file.get(), &file_header, sizeof(file_header)) &&
        write_all(file.get(), &image, sizeof(image)) &&
        write_all(file.get(), pixels.get(), image.biSizeImage) && FlushFileBuffers(file.get()) != FALSE;
    printf("LIGHTNING_CAPTURE_%s width=%ld height=%ld\n", result ? "PASS" : "FAIL", width, height);
    return result;
}

bool capture_after_delay(const char* delay_text, const char* path) noexcept {
    if (delay_text == nullptr || path == nullptr || path[0] == '\0') return false;
    DWORD delay = 0;
    unsigned digits = 0;
    while (delay_text[digits] != '\0') {
        if (digits == 5 || delay_text[digits] < '0' || delay_text[digits] > '9') return false;
        delay = delay * 10 + static_cast<DWORD>(delay_text[digits] - '0');
        if (delay > 30000) return false;
        ++digits;
    }
    if (digits == 0) return false;
    // Detach only this process. A shared parent console remains owned by its
    // other clients; a sole capture console closes before the repro is staged.
    FreeConsole();
    Sleep(delay);
    return capture_desktop(path);
}

bool exercise(HWND window) noexcept {
    RECT client{};
    if (!GetClientRect(window, &client)) return false;
    HDC dc = GetDC(window);
    if (dc == nullptr) return false;
    bool result = restored(dc, client, 0);
    for (unsigned iteration = 0; result && iteration < 8; ++iteration) {
        const LONG shift = static_cast<LONG>(iteration) * 3;
        const RECT frame{8 + shift, 9 + shift, client.right - 8, client.bottom - 9};
        const int saved = SaveDC(dc);
        result = saved != 0;
        if (result && (iteration & 1U) != 0) {
            result = ExcludeClipRect(dc, frame.left + 12, frame.top,
                frame.left + 32, frame.bottom) != ERROR;
        }
        if (result) result = outline(dc, frame) && GdiFlush();
        // Require a visible mutation at an unclipped top-edge point, so a
        // driver returning success without drawing cannot pass undo checks.
        if (result) {
            const LONG x = frame.right - 12;
            const COLORREF first = GetPixel(dc, x, frame.top);
            const COLORREF second = GetPixel(dc, x + 1, frame.top);
            result = first != CLR_INVALID && second != CLR_INVALID &&
                (first != color_at(x, frame.top) || second != color_at(x + 1, frame.top));
        }
        if (result) result = outline(dc, frame);
        if (saved != 0 && !RestoreDC(dc, saved)) result = false;
        if (result) result = restored(dc, client, iteration + 1);
    }
    ReleaseDC(window, dc);
    return result;
}

bool diagnostics() noexcept {
    namespace shadow = rx7900::reactos::framebuffer;
    namespace gpu = lightning::reactos::gdi;
    shadow::ShadowStatistics cached{};
    gpu::Statistics canvas{};
    HDC dc = GetDC(nullptr);
    if (dc == nullptr) return false;
    const bool shadow_valid = ExtEscape(dc, shadow::query_shadow_escape, 0, nullptr,
        sizeof(cached), reinterpret_cast<char*>(&cached)) == sizeof(cached) &&
        cached.size == sizeof(cached) && cached.version == shadow::shadow_statistics_version &&
        cached.magic == shadow::shadow_statistics_magic;
    const bool gpu_valid = ExtEscape(dc, gpu::query_statistics_escape, 0, nullptr,
        sizeof(canvas), reinterpret_cast<char*>(&canvas)) == sizeof(canvas) &&
        canvas.size == sizeof(canvas) && canvas.version == gpu::statistics_abi_version &&
        canvas.device.size == sizeof(canvas.device) && canvas.device.version == gpu::native_abi_version &&
        canvas.device.magic == LIGHTNING_GDI_MAGIC;
    ReleaseDC(nullptr, dc);
    printf("SHADOW_QUERY valid=%u active=%u failed=%llu qpc_failures=%llu copies=%llu bitblts=%llu\n",
        shadow_valid ? 1U : 0U, cached.active, static_cast<unsigned long long>(cached.failed_calls),
        static_cast<unsigned long long>(cached.qpc_failures),
        static_cast<unsigned long long>(cached.copy_calls), static_cast<unsigned long long>(cached.bitblt_calls));
    printf("GPU_QUERY valid=%u enabled=%u poisoned=%u quarantined=%u generation=%u epoch=%u "
        "accepted=%u completed=%u revision=%u reset=%u fallback=%llu\n",
        gpu_valid ? 1U : 0U, canvas.enabled, canvas.poisoned, canvas.device.quarantined,
        canvas.device.mode_generation, canvas.device.epoch, canvas.device.accepted_sequence,
        canvas.device.completed_sequence, canvas.device.canvas_revision, canvas.device.reset_generation,
        static_cast<unsigned long long>(canvas.cpu_fallbacks));
    return shadow_valid && gpu_valid;
}
} // namespace

int main(int argc, char** argv) {
    if (argc == 3 && strcmp(argv[1], "--capture") == 0) return capture_desktop(argv[2]) ? 0 : 1;
    if (argc == 4 && strcmp(argv[1], "--capture-delay") == 0)
        return capture_after_delay(argv[2], argv[3]) ? 0 : 1;
    if (argc == 2 && strcmp(argv[1], "--diagnostics") == 0) return diagnostics() ? 0 : 1;
    if (argc == 2 && strcmp(argv[1], "--drag-full") == 0) {
        BOOL enabled = FALSE;
        const bool queried = SystemParametersInfoW(SPI_GETDRAGFULLWINDOWS, 0, &enabled, 0) != FALSE;
        printf("LIGHTNING_DRAG_FULL queried=%u enabled=%u\n", queried ? 1U : 0U, enabled ? 1U : 0U);
        return queried ? 0 : 1;
    }
    if (argc != 1) return 1;
    WNDCLASSW definition{};
    definition.lpfnWndProc = window_proc;
    definition.hInstance = GetModuleHandleW(nullptr);
    definition.lpszClassName = class_name;
    if (RegisterClassW(&definition) == 0) return 1;
    Window window(CreateWindowExW(WS_EX_TOPMOST, class_name, L"Lightning resize regression",
        WS_OVERLAPPEDWINDOW, 64, 64, 256, 192, nullptr, nullptr, definition.hInstance, nullptr));
    if (!window.valid()) return 1;
    ShowWindow(window.get(), SW_SHOW);
    bool result = UpdateWindow(window.get()) != FALSE && exercise(window.get());
    for (unsigned iteration = 0; result && iteration < 6; ++iteration) {
        const int width = 256 + static_cast<int>(iteration % 3) * 17;
        const int height = 192 + static_cast<int>(iteration % 2) * 19;
        result = SetWindowPos(window.get(), HWND_TOPMOST, 64, 64, width, height,
            SWP_NOACTIVATE | SWP_NOMOVE) != FALSE &&
            RedrawWindow(window.get(), nullptr, nullptr, RDW_INVALIDATE | RDW_UPDATENOW) != FALSE &&
            GetPropW(window.get(), L"PaintFailed") == nullptr && exercise(window.get());
    }
    RemovePropW(window.get(), L"PaintFailed");
    printf("LIGHTNING_RESIZE_%s\n", result ? "PASS" : "FAIL");
    fflush(stdout);
    return result ? 0 : 1;
}
