// SPDX-License-Identifier: GPL-2.0-or-later
/*
Module: rx7900.tests.radeon_board_check
Purpose: Record what the installed display driver does on a real card.
Created: 2026-11-15
Architecture: One Win32 program run by a person on the ReactOS desktop. It
draws through GDI on its own window, reads the pixels back and compares them
with the same drawing made on a memory bitmap, moves and reshapes the mouse
pointer, and renders a small OpenGL scene. Every step is written to a report
file beside the program and flushed before the step runs, so a machine that
stops still leaves the name of the step. It installs and changes nothing.
"/auto" skips the questions (for an unattended run in a test guest).
*/
#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif
#include <windows.h>
#include <GL/gl.h>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {
constexpr int canvas_width = 480, canvas_height = 360;
HANDLE report = INVALID_HANDLE_VALUE;
bool unattended = false;

// Report lines are flushed at once: the last line names the step in progress.
void line(const char *format, ...) {
    char text[1024];
    va_list arguments;
    va_start(arguments, format);
    int length = vsnprintf(text, sizeof(text) - 2, format, arguments);
    va_end(arguments);
    if (length < 0) {
        return;
    }
    if (length > static_cast<int>(sizeof(text)) - 3) {
        length = sizeof(text) - 3;
    }
    text[length++] = '\r';
    text[length++] = '\n';
    if (report != INVALID_HANDLE_VALUE) {
        DWORD written = 0;
        WriteFile(report, text, static_cast<DWORD>(length), &written, nullptr);
        FlushFileBuffers(report);
    }
}

std::string narrow(const wchar_t *text) {
    char buffer[512];
    const int count = WideCharToMultiByte(CP_UTF8, 0, text, -1, buffer, sizeof(buffer), nullptr, nullptr);
    return count > 0 ? std::string(buffer) : std::string();
}

std::wstring beside_program(const wchar_t *name) {
    wchar_t path[MAX_PATH];
    const DWORD length = GetModuleFileNameW(nullptr, path, MAX_PATH);
    std::wstring result(path, length);
    const size_t slash = result.find_last_of(L"\\/");
    result.resize(slash == std::wstring::npos ? 0 : slash + 1);
    return result + name;
}

bool open_report() {
    std::wstring path = beside_program(L"RadeonCheck report.txt");
    report = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS,
                         FILE_ATTRIBUTE_NORMAL, nullptr);
    if (report == INVALID_HANDLE_VALUE) {
        wchar_t temporary[MAX_PATH];
        if (GetTempPathW(MAX_PATH, temporary) == 0) {
            return false;
        }
        path = std::wstring(temporary) + L"RadeonCheck report.txt";
        report = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS,
                             FILE_ATTRIBUTE_NORMAL, nullptr);
    }
    if (report != INVALID_HANDLE_VALUE) {
        line("RadeonCheck report");
        line("file: %s", narrow(path.c_str()).c_str());
    }
    return report != INVALID_HANDLE_VALUE;
}

void pump(DWORD milliseconds) {
    const DWORD end = GetTickCount() + milliseconds;
    do {
        MSG message;
        while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&message);
            DispatchMessageW(&message);
        }
        Sleep(5);
    } while (static_cast<LONG>(end - GetTickCount()) > 0);
}

// ---- environment ---------------------------------------------------------

void describe_display() {
    line("== display");
    for (DWORD index = 0; index < 8; ++index) {
        DISPLAY_DEVICEW device{};
        device.cb = sizeof(device);
        if (!EnumDisplayDevicesW(nullptr, index, &device, 0)) {
            break;
        }
        line("device %lu: %s | %s | flags 0x%lx | %s", index, narrow(device.DeviceName).c_str(),
             narrow(device.DeviceString).c_str(), device.StateFlags, narrow(device.DeviceID).c_str());
    }
    DEVMODEW mode{};
    mode.dmSize = sizeof(mode);
    if (EnumDisplaySettingsW(nullptr, ENUM_CURRENT_SETTINGS, &mode)) {
        line("current mode: %lux%lu, %lu bits, %lu Hz", mode.dmPelsWidth, mode.dmPelsHeight,
             mode.dmBitsPerPel, mode.dmDisplayFrequency);
    } else {
        line("current mode: not reported (error %lu)", GetLastError());
    }
    unsigned count = 0;
    std::string listed;
    DWORD last_width = 0, last_height = 0, last_bits = 0;
    for (DWORD index = 0; index < 2000; ++index) {
        DEVMODEW entry{};
        entry.dmSize = sizeof(entry);
        if (!EnumDisplaySettingsW(nullptr, index, &entry)) {
            break;
        }
        if (entry.dmPelsWidth == last_width && entry.dmPelsHeight == last_height &&
            entry.dmBitsPerPel == last_bits) {
            continue;
        }
        last_width = entry.dmPelsWidth;
        last_height = entry.dmPelsHeight;
        last_bits = entry.dmBitsPerPel;
        ++count;
        if (listed.size() < 800) {
            char item[48];
            snprintf(item, sizeof(item), " %lux%lux%lu", entry.dmPelsWidth, entry.dmPelsHeight, entry.dmBitsPerPel);
            listed += item;
        }
    }
    line("modes offered: %u:%s", count, listed.c_str());
}

// Runs radeonctl status beside this program and copies its output.
void installer_status() {
    line("== radeonctl status");
    const std::wstring program = beside_program(L"radeonctl.exe");
    if (GetFileAttributesW(program.c_str()) == INVALID_FILE_ATTRIBUTES) {
        line("radeonctl.exe is not beside this program; skipped");
        return;
    }
    SECURITY_ATTRIBUTES inherit{sizeof(inherit), nullptr, TRUE};
    HANDLE read_end = nullptr, write_end = nullptr;
    if (!CreatePipe(&read_end, &write_end, &inherit, 1 << 16)) {
        line("pipe failed (error %lu)", GetLastError());
        return;
    }
    SetHandleInformation(read_end, HANDLE_FLAG_INHERIT, 0);
    std::wstring command = L"\"" + program + L"\" status";
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
    startup.wShowWindow = SW_HIDE;
    startup.hStdOutput = write_end;
    startup.hStdError = write_end;
    PROCESS_INFORMATION process{};
    if (!CreateProcessW(nullptr, &command[0], nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr,
                        &startup, &process)) {
        line("could not start radeonctl (error %lu)", GetLastError());
        CloseHandle(read_end);
        CloseHandle(write_end);
        return;
    }
    CloseHandle(write_end);
    std::string output;
    char buffer[4096];
    DWORD got = 0;
    while (output.size() < 60000 && ReadFile(read_end, buffer, sizeof(buffer), &got, nullptr) && got != 0) {
        output.append(buffer, got);
    }
    CloseHandle(read_end);
    DWORD code = 0xffffffff;
    WaitForSingleObject(process.hProcess, 30000);
    GetExitCodeProcess(process.hProcess, &code);
    CloseHandle(process.hProcess);
    CloseHandle(process.hThread);
    if (report != INVALID_HANDLE_VALUE && !output.empty()) {
        DWORD written = 0;
        WriteFile(report, output.data(), static_cast<DWORD>(output.size()), &written, nullptr);
    }
    line("radeonctl status exit code %lu, %u bytes of output", code, static_cast<unsigned>(output.size()));
}

// ---- 2D ------------------------------------------------------------------

LRESULT CALLBACK window_procedure(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_ERASEBKGND) {
        return 1; // The checks own every pixel of the client area.
    }
    if (message == WM_PAINT) {
        ValidateRect(window, nullptr);
        return 0;
    }
    return DefWindowProcW(window, message, wparam, lparam);
}

HWND make_window(const wchar_t *class_name, int x, int y) {
    WNDCLASSW window_class{};
    window_class.style = CS_OWNDC;
    window_class.lpfnWndProc = window_procedure;
    window_class.hInstance = GetModuleHandleW(nullptr);
    window_class.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    window_class.lpszClassName = class_name;
    RegisterClassW(&window_class);
    HWND window = CreateWindowExW(WS_EX_TOPMOST | WS_EX_TOOLWINDOW, class_name, L"RadeonCheck", WS_POPUP, x, y,
                                  canvas_width, canvas_height, nullptr, nullptr, window_class.hInstance, nullptr);
    if (window) {
        ShowWindow(window, SW_SHOWNOACTIVATE);
        SetWindowPos(window, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
        pump(300);
    }
    return window;
}

void fill(HDC dc, int left, int top, int right, int bottom, COLORREF color) {
    HBRUSH brush = CreateSolidBrush(color);
    RECT area{left, top, right, bottom};
    FillRect(dc, &area, brush);
    DeleteObject(brush);
}

void draw_solid_fills(HDC dc) {
    const COLORREF colors[] = {RGB(255, 0, 0), RGB(0, 255, 0), RGB(0, 0, 255), RGB(255, 255, 255),
                               RGB(128, 128, 128), RGB(0x12, 0x34, 0x56), RGB(255, 255, 0), RGB(1, 2, 3)};
    for (int index = 0; index < 8; ++index) {
        const int column = index % 4, row = index / 4;
        fill(dc, column * 120 + 3, row * 180 + 5, column * 120 + 117, row * 180 + 171, colors[index]);
    }
    fill(dc, 200, 150, 201, 151, RGB(255, 255, 255)); // one pixel
    fill(dc, 0, 179, canvas_width, 180, RGB(10, 200, 30)); // one row
    fill(dc, 239, 0, 240, canvas_height, RGB(200, 10, 30)); // one column
}

void draw_patterns(HDC dc) {
    SetBkColor(dc, RGB(250, 240, 200));
    SetTextColor(dc, RGB(20, 40, 160));
    SetBkMode(dc, OPAQUE);
    const int hatches[] = {HS_DIAGCROSS, HS_HORIZONTAL, HS_FDIAGONAL, HS_CROSS};
    for (int index = 0; index < 4; ++index) {
        HBRUSH brush = CreateHatchBrush(hatches[index], RGB(180, 30, 30));
        HGDIOBJ previous = SelectObject(dc, brush);
        PatBlt(dc, index * 120 + 5, 7, 110, 100, PATCOPY);
        SelectObject(dc, previous);
        DeleteObject(brush);
    }
    static const WORD bits[8] = {0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81};
    HBITMAP tile = CreateBitmap(8, 8, 1, 1, bits);
    HBRUSH brush = CreatePatternBrush(tile);
    HGDIOBJ previous = SelectObject(dc, brush);
    PatBlt(dc, 9, 120, 301, 97, PATCOPY);
    SetBrushOrgEx(dc, 3, 5, nullptr);
    PatBlt(dc, 320, 120, 151, 97, PATCOPY);
    SetBrushOrgEx(dc, 0, 0, nullptr);
    PatBlt(dc, 40, 230, 400, 110, PATINVERT);
    SelectObject(dc, previous);
    DeleteObject(brush);
    DeleteObject(tile);
}

void draw_raster_operations(HDC dc) {
    for (int band = 0; band < 6; ++band) {
        fill(dc, 0, band * 60, canvas_width, band * 60 + 60, RGB(40 * band + 20, 255 - 40 * band, 90 + 20 * band));
    }
    PatBlt(dc, 20, 20, 100, 320, BLACKNESS);
    PatBlt(dc, 140, 20, 100, 320, WHITENESS);
    PatBlt(dc, 260, 20, 100, 320, DSTINVERT);
    HBRUSH brush = CreateSolidBrush(RGB(0x55, 0xaa, 0x0f));
    HGDIOBJ previous = SelectObject(dc, brush);
    PatBlt(dc, 380, 20, 80, 320, PATINVERT);
    PatBlt(dc, 60, 150, 360, 50, 0x00A000C9); // PATAND
    SelectObject(dc, previous);
    DeleteObject(brush);
}

void draw_stripes(HDC dc) {
    for (int stripe = 0; stripe < 48; ++stripe) {
        fill(dc, stripe * 10, 0, stripe * 10 + 10, canvas_height,
             RGB((stripe * 37) & 255, (stripe * 91) & 255, (stripe * 53) & 255));
    }
    for (int stripe = 0; stripe < 36; stripe += 2) {
        fill(dc, 0, stripe * 10, canvas_width, stripe * 10 + 3, RGB(255 - stripe * 7, stripe * 5, 128));
    }
}

void draw_self_copies(HDC dc) {
    draw_stripes(dc);
    BitBlt(dc, 30, 20, 200, 150, dc, 10, 10, SRCCOPY);    // overlapping, moving down and right
    BitBlt(dc, 250, 10, 200, 150, dc, 270, 30, SRCCOPY);  // overlapping, moving up and left
    BitBlt(dc, 20, 200, 180, 140, dc, 280, 190, SRCCOPY); // disjoint
    BitBlt(dc, 300, 200, 150, 120, dc, 250, 215, SRCINVERT);
    RECT scroll{60, 60, 420, 300};
    ScrollDC(dc, 17, -9, &scroll, &scroll, nullptr, nullptr);
}

HBITMAP make_source(HDC dc, int width, int height) {
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(info.bmiHeader);
    info.bmiHeader.biWidth = width;
    info.bmiHeader.biHeight = -height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    void *pixels = nullptr;
    HBITMAP bitmap = CreateDIBSection(dc, &info, DIB_RGB_COLORS, &pixels, nullptr, 0);
    if (bitmap && pixels) {
        auto *out = static_cast<uint32_t *>(pixels);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                out[y * width + x] = ((x * 255 / width) << 16) | ((y * 255 / height) << 8) | ((x ^ y) & 255);
            }
        }
    }
    return bitmap;
}

void draw_bitmap_blits(HDC dc) {
    fill(dc, 0, 0, canvas_width, canvas_height, RGB(60, 60, 90));
    HDC memory = CreateCompatibleDC(dc);
    HBITMAP source = make_source(dc, 128, 96);
    if (!memory || !source) {
        return;
    }
    HGDIOBJ previous = SelectObject(memory, source);
    BitBlt(dc, 10, 10, 128, 96, memory, 0, 0, SRCCOPY);
    BitBlt(dc, 150, 10, 128, 96, memory, 0, 0, SRCINVERT);
    BitBlt(dc, 290, 10, 128, 96, memory, 0, 0, SRCAND);
    BitBlt(dc, 430, 10, 128, 96, memory, 0, 0, SRCCOPY); // clipped at the right edge
    BitBlt(dc, 10, 120, 100, 70, memory, 20, 20, SRCPAINT);
    BitBlt(dc, 130, 120, 100, 70, memory, 20, 20, NOTSRCCOPY);
    SetStretchBltMode(dc, COLORONCOLOR);
    StretchBlt(dc, 250, 120, 220, 150, memory, 0, 0, 110, 75, SRCCOPY);
    StretchBlt(dc, 10, 210, 64, 48, memory, 0, 0, 128, 96, SRCCOPY);
    SelectObject(memory, previous);
    DeleteObject(source);
    DeleteDC(memory);
}

void draw_shapes(HDC dc) {
    fill(dc, 0, 0, canvas_width, canvas_height, RGB(255, 255, 255));
    for (int index = 0; index < 24; ++index) {
        HPEN pen = CreatePen(PS_SOLID, index % 3 == 0 ? 3 : 1, RGB(index * 10, 255 - index * 10, 100));
        HGDIOBJ previous = SelectObject(dc, pen);
        MoveToEx(dc, 240, 180, nullptr);
        LineTo(dc, 240 + (index - 12) * 19, index % 2 ? 5 : 355);
        MoveToEx(dc, 5, 10 + index * 14, nullptr);
        LineTo(dc, 475, 350 - index * 14);
        SelectObject(dc, previous);
        DeleteObject(pen);
    }
    HBRUSH brush = CreateSolidBrush(RGB(30, 144, 255));
    HPEN pen = CreatePen(PS_DOT, 1, RGB(0, 0, 0));
    HGDIOBJ old_brush = SelectObject(dc, brush), old_pen = SelectObject(dc, pen);
    Rectangle(dc, 20, 20, 150, 110);
    Ellipse(dc, 300, 30, 460, 150);
    RoundRect(dc, 30, 230, 200, 340, 30, 30);
    const POINT polygon[] = {{260, 220}, {450, 250}, {400, 340}, {300, 320}, {330, 270}};
    Polygon(dc, polygon, 5);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(brush);
    DeleteObject(pen);
}

void draw_text(HDC dc) {
    fill(dc, 0, 0, canvas_width, canvas_height, RGB(230, 230, 240));
    HGDIOBJ previous = SelectObject(dc, GetStockObject(DEFAULT_GUI_FONT));
    static const wchar_t sample[] = L"The quick brown fox jumps over the lazy dog 0123456789";
    for (int row = 0; row < 12; ++row) {
        SetTextColor(dc, RGB(row * 20, 30, 200 - row * 15));
        SetBkColor(dc, RGB(255, 255 - row * 10, 200));
        SetBkMode(dc, row % 2 ? TRANSPARENT : OPAQUE);
        TextOutW(dc, 6 + row * 3, 8 + row * 28, sample, lstrlenW(sample));
    }
    SelectObject(dc, previous);
}

void draw_clipped(HDC dc) {
    fill(dc, 0, 0, canvas_width, canvas_height, RGB(20, 20, 20));
    HRGN region = CreateEllipticRgn(40, 30, 440, 330);
    HRGN hole = CreateRectRgn(180, 120, 300, 240);
    CombineRgn(region, region, hole, RGN_DIFF);
    SelectClipRgn(dc, region);
    draw_stripes(dc);
    BitBlt(dc, 60, 60, 300, 200, dc, 90, 80, SRCCOPY);
    SelectClipRgn(dc, nullptr);
    DeleteObject(region);
    DeleteObject(hole);
}

using GradientFunction = BOOL(WINAPI *)(HDC, PTRIVERTEX, ULONG, PVOID, ULONG, ULONG);
GradientFunction gradient_fill = nullptr;

void draw_gradients(HDC dc) {
    fill(dc, 0, 0, canvas_width, canvas_height, RGB(0, 0, 0));
    if (!gradient_fill) {
        return;
    }
    TRIVERTEX corners[4] = {{10, 10, 0xff00, 0, 0, 0},      {470, 170, 0, 0, 0xff00, 0},
                            {10, 190, 0, 0xff00, 0, 0},     {470, 350, 0xff00, 0xff00, 0xff00, 0}};
    GRADIENT_RECT first{0, 1}, second{2, 3};
    gradient_fill(dc, corners, 2, &first, 1, GRADIENT_FILL_RECT_H);
    gradient_fill(dc, corners + 2, 2, &second, 1, GRADIENT_FILL_RECT_V);
}

struct Case {
    const char *name;
    void (*draw)(HDC);
};
const Case cases[] = {{"solid fills", draw_solid_fills},       {"pattern brushes", draw_patterns},
                      {"raster operations", draw_raster_operations}, {"screen-to-screen copies", draw_self_copies},
                      {"bitmap blits and stretches", draw_bitmap_blits}, {"lines and shapes", draw_shapes},
                      {"text", draw_text},                       {"clipped drawing", draw_clipped},
                      {"gradients", draw_gradients}};

bool read_pixels(HDC dc, HBITMAP bitmap, std::vector<uint32_t> &pixels) {
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(info.bmiHeader);
    info.bmiHeader.biWidth = canvas_width;
    info.bmiHeader.biHeight = -canvas_height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    pixels.assign(canvas_width * canvas_height, 0);
    return GetDIBits(dc, bitmap, 0, canvas_height, pixels.data(), &info, DIB_RGB_COLORS) == canvas_height;
}

double milliseconds_since(const LARGE_INTEGER &start) {
    LARGE_INTEGER now, frequency;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&frequency);
    return 1000.0 * static_cast<double>(now.QuadPart - start.QuadPart) / static_cast<double>(frequency.QuadPart);
}

// Captures the window's client area as the driver returns it.
bool capture(HDC window_dc, std::vector<uint32_t> &pixels) {
    HDC memory = CreateCompatibleDC(window_dc);
    HBITMAP copy = CreateCompatibleBitmap(window_dc, canvas_width, canvas_height);
    bool good = memory && copy;
    if (good) {
        HGDIOBJ previous = SelectObject(memory, copy);
        good = BitBlt(memory, 0, 0, canvas_width, canvas_height, window_dc, 0, 0, SRCCOPY) != FALSE;
        SelectObject(memory, previous);
        good = good && read_pixels(window_dc, copy, pixels);
    }
    if (copy) {
        DeleteObject(copy);
    }
    if (memory) {
        DeleteDC(memory);
    }
    return good;
}

unsigned compare(const std::vector<uint32_t> &screen, const std::vector<uint32_t> &reference, const char *what) {
    unsigned different = 0;
    int first_x = -1, first_y = -1;
    uint32_t first_screen = 0, first_reference = 0;
    for (int index = 0; index < canvas_width * canvas_height; ++index) {
        if ((screen[index] ^ reference[index]) & 0x00ffffff) {
            if (different++ == 0) {
                first_x = index % canvas_width;
                first_y = index / canvas_width;
                first_screen = screen[index] & 0x00ffffff;
                first_reference = reference[index] & 0x00ffffff;
            }
        }
    }
    if (different) {
        line("  %s: %u of %d pixels differ; first at (%d,%d): screen %06lx, expected %06lx", what, different,
             canvas_width * canvas_height, first_x, first_y, static_cast<unsigned long>(first_screen),
             static_cast<unsigned long>(first_reference));
    }
    return different;
}

void check_drawing(HWND window, unsigned &failed_cases, unsigned &run_cases) {
    line("== 2D drawing (window pixels compared with the same drawing on a memory bitmap)");
    HDC window_dc = GetDC(window);
    HDC memory = CreateCompatibleDC(window_dc);
    HBITMAP reference_bitmap = CreateCompatibleBitmap(window_dc, canvas_width, canvas_height);
    if (!window_dc || !memory || !reference_bitmap) {
        line("could not create the drawing surfaces (error %lu)", GetLastError());
        ++failed_cases;
        return;
    }
    HGDIOBJ previous = SelectObject(memory, reference_bitmap);
    std::vector<uint32_t> screen, reference;
    // Readback self-check: tells a failed drawing from a failed readback. One
    // pixel is read with GetPixel and the whole area with a block copy.
    const COLORREF probes[] = {RGB(255, 255, 255), RGB(255, 0, 0), RGB(0, 0, 255), RGB(0, 0, 0)};
    for (const COLORREF probe : probes) {
        fill(window_dc, 0, 0, canvas_width, canvas_height, probe);
        GdiFlush();
        const COLORREF single = GetPixel(window_dc, 200, 100);
        const bool captured = capture(window_dc, screen);
        const uint32_t block = captured ? screen[100 * canvas_width + 200] & 0x00ffffff : 0;
        const uint32_t wanted = (GetRValue(probe) << 16) | (GetGValue(probe) << 8) | GetBValue(probe);
        const uint32_t single_rgb = (GetRValue(single) << 16) | (GetGValue(single) << 8) | GetBValue(single);
        line("readback of a %06lx fill: GetPixel %06lx, block copy %s%06lx%s", static_cast<unsigned long>(wanted),
             static_cast<unsigned long>(single_rgb), captured ? "" : "FAILED ", static_cast<unsigned long>(block),
             single_rgb == wanted && block == wanted ? "" : "  <-- unexpected");
    }
    for (const Case &item : cases) {
        line("begin: %s", item.name);
        ++run_cases;
        LARGE_INTEGER start;
        QueryPerformanceCounter(&start);
        fill(window_dc, 0, 0, canvas_width, canvas_height, RGB(96, 96, 96));
        item.draw(window_dc);
        GdiFlush();
        const double screen_time = milliseconds_since(start);
        QueryPerformanceCounter(&start);
        fill(memory, 0, 0, canvas_width, canvas_height, RGB(96, 96, 96));
        item.draw(memory);
        GdiFlush();
        const double memory_time = milliseconds_since(start);
        SelectObject(memory, previous);
        const bool read = capture(window_dc, screen) && read_pixels(window_dc, reference_bitmap, reference);
        previous = SelectObject(memory, reference_bitmap);
        if (!read) {
            line("  %s: could not read the pixels back (error %lu)", item.name, GetLastError());
            ++failed_cases;
            continue;
        }
        const unsigned different = compare(screen, reference, item.name);
        failed_cases += different ? 1 : 0;
        line("end: %s: %s (screen %.1f ms, memory %.1f ms)", item.name, different ? "DIFFERENT" : "identical",
             screen_time, memory_time);
        pump(150);
    }
    SelectObject(memory, previous);
    DeleteObject(reference_bitmap);
    DeleteDC(memory);
    ReleaseDC(window, window_dc);
}

// ---- pointer -------------------------------------------------------------

HCURSOR make_colour_cursor() {
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(info.bmiHeader);
    info.bmiHeader.biWidth = 32;
    info.bmiHeader.biHeight = -32;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    void *pixels = nullptr;
    HDC screen = GetDC(nullptr);
    HBITMAP colour = CreateDIBSection(screen, &info, DIB_RGB_COLORS, &pixels, nullptr, 0);
    ReleaseDC(nullptr, screen);
    BYTE mask_bits[32 * 4];
    for (BYTE &value : mask_bits) {
        value = 0xff;
    }
    if (colour && pixels) {
        auto *out = static_cast<uint32_t *>(pixels);
        for (int y = 0; y < 32; ++y) {
            for (int x = 0; x < 32; ++x) {
                const bool inside = x <= y && y < 28;
                out[y * 32 + x] = inside ? (0xff000000u | ((x * 8) << 16) | ((255 - y * 8) << 8) | 0x40) : 0;
                if (inside) {
                    mask_bits[y * 4 + x / 8] &= static_cast<BYTE>(~(0x80 >> (x % 8)));
                }
            }
        }
    }
    HBITMAP mask = CreateBitmap(32, 32, 1, 1, mask_bits);
    ICONINFO icon{};
    icon.fIcon = FALSE;
    icon.hbmMask = mask;
    icon.hbmColor = colour;
    HCURSOR cursor = colour && mask ? static_cast<HCURSOR>(CreateIconIndirect(&icon)) : nullptr;
    if (mask) {
        DeleteObject(mask);
    }
    if (colour) {
        DeleteObject(colour);
    }
    return cursor;
}

void check_pointer(HWND window, unsigned &position_errors, unsigned &trail_pixels) {
    line("== mouse pointer");
    CURSORINFO info{};
    info.cbSize = sizeof(info);
    if (GetCursorInfo(&info)) {
        line("pointer state: flags 0x%lx at (%ld,%ld), size %dx%d", info.flags, info.ptScreenPos.x,
             info.ptScreenPos.y, GetSystemMetrics(SM_CXCURSOR), GetSystemMetrics(SM_CYCURSOR));
    }
    HDC dc = GetDC(window);
    fill(dc, 0, 0, canvas_width, canvas_height, RGB(245, 245, 245));
    draw_stripes(dc);
    HGDIOBJ previous = SelectObject(dc, GetStockObject(DEFAULT_GUI_FONT));
    SetBkMode(dc, OPAQUE);
    SetBkColor(dc, RGB(255, 255, 255));
    SetTextColor(dc, RGB(0, 0, 0));
    static const wchar_t notice[] = L" Watch the mouse pointer: it moves around this window and changes shape six times. ";
    TextOutW(dc, 20, 170, notice, lstrlenW(notice));
    SelectObject(dc, previous);
    GdiFlush();
    RECT area;
    GetWindowRect(window, &area);
    SetCursorPos(area.right + 60, area.bottom + 60);
    pump(200);
    std::vector<uint32_t> before, after;
    const bool have_before = capture(dc, before);

    POINT saved;
    GetCursorPos(&saved);
    HCURSOR custom = make_colour_cursor();
    const struct {
        const char *name;
        HCURSOR cursor;
    } shapes[] = {{"arrow", LoadCursorW(nullptr, IDC_ARROW)},   {"text", LoadCursorW(nullptr, IDC_IBEAM)},
                  {"wait", LoadCursorW(nullptr, IDC_WAIT)},     {"cross", LoadCursorW(nullptr, IDC_CROSS)},
                  {"move", LoadCursorW(nullptr, IDC_SIZEALL)},  {"colour triangle", custom}};
    for (const auto &shape : shapes) {
        line("begin: pointer shape %s%s", shape.name, shape.cursor ? "" : " (could not be created)");
        if (!shape.cursor) {
            continue;
        }
        SetClassLongPtrW(window, GCLP_HCURSOR, reinterpret_cast<LONG_PTR>(shape.cursor));
        SetCursor(shape.cursor);
        for (int step = 0; step <= 60; ++step) {
            const int side = step / 15, along = step % 15;
            int x = area.left + 40, y = area.top + 40;
            const int width = canvas_width - 80, height = canvas_height - 80;
            if (side == 0) {
                x += width * along / 15;
            } else if (side == 1) {
                x += width;
                y += height * along / 15;
            } else if (side == 2) {
                x += width - width * along / 15;
                y += height;
            } else {
                y += height - height * along / 15;
            }
            SetCursorPos(x, y);
            POINT now;
            if (!GetCursorPos(&now) || now.x != x || now.y != y) {
                ++position_errors;
            }
            pump(12);
        }
        line("end: pointer shape %s", shape.name);
    }
    SetCursorPos(area.right + 60, area.bottom + 60);
    pump(300);
    if (have_before && capture(dc, after)) {
        trail_pixels = compare(after, before, "window after the pointer left");
        line("pixels changed under the pointer's path: %u", trail_pixels);
    } else {
        line("could not read the window back for the trail check");
    }
    SetClassLongPtrW(window, GCLP_HCURSOR, reinterpret_cast<LONG_PTR>(LoadCursorW(nullptr, IDC_ARROW)));
    SetCursorPos(saved.x, saved.y);
    if (custom) {
        DestroyCursor(custom);
    }
    ReleaseDC(window, dc);
    line("pointer positions not honoured: %u", position_errors);
}

// ---- OpenGL --------------------------------------------------------------

bool near_colour(const unsigned char *pixel, int red, int green, int blue) {
    return pixel[0] >= red - 2 && pixel[0] <= red + 2 && pixel[1] >= green - 2 && pixel[1] <= green + 2 &&
           pixel[2] >= blue - 2 && pixel[2] <= blue + 2;
}

void check_opengl(unsigned &failures) {
    line("== OpenGL");
    line("begin: OpenGL window and pixel format");
    HWND window = make_window(L"RadeonCheckOpenGL", 40, 40); // the 2D window is gone by now
    HDC dc = window ? GetDC(window) : nullptr;
    PIXELFORMATDESCRIPTOR wanted{};
    wanted.nSize = sizeof(wanted);
    wanted.nVersion = 1;
    wanted.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    wanted.iPixelType = PFD_TYPE_RGBA;
    wanted.cColorBits = 32;
    wanted.cDepthBits = 24;
    const int format = dc ? ChoosePixelFormat(dc, &wanted) : 0;
    if (!format || !SetPixelFormat(dc, format, &wanted)) {
        line("no usable pixel format (error %lu)", GetLastError());
        ++failures;
        return;
    }
    PIXELFORMATDESCRIPTOR chosen{};
    DescribePixelFormat(dc, format, sizeof(chosen), &chosen);
    const bool generic = (chosen.dwFlags & PFD_GENERIC_FORMAT) != 0;
    line("pixel format %d: colour %d, depth %d, %s", format, chosen.cColorBits, chosen.cDepthBits,
         generic ? ((chosen.dwFlags & PFD_GENERIC_ACCELERATED) ? "generic accelerated" : "generic software")
                 : "installable client driver");
    line("begin: OpenGL context");
    HGLRC context = wglCreateContext(dc);
    if (!context || !wglMakeCurrent(dc, context)) {
        line("could not create or select a context (error %lu)", GetLastError());
        ++failures;
        return;
    }
    const auto text = [](GLenum name) {
        const GLubyte *value = glGetString(name);
        return value ? reinterpret_cast<const char *>(value) : "(none)";
    };
    line("vendor: %s", text(GL_VENDOR));
    line("renderer: %s", text(GL_RENDERER));
    line("version: %s", text(GL_VERSION));

    line("begin: OpenGL clear");
    glViewport(0, 0, canvas_width, canvas_height);
    glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    unsigned char pixel[4] = {};
    glReadPixels(canvas_width / 2, canvas_height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    const bool clear_good = near_colour(pixel, 51, 102, 153);
    line("clear colour read back: %u,%u,%u (expected 51,102,153): %s", pixel[0], pixel[1], pixel[2],
         clear_good ? "ok" : "WRONG");
    failures += clear_good ? 0 : 1;

    line("begin: OpenGL triangles with depth test");
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f); // near, red
    glVertex3f(-0.6f, -0.6f, -0.5f);
    glVertex3f(0.6f, -0.6f, -0.5f);
    glVertex3f(0.0f, 0.7f, -0.5f);
    glColor3f(0.0f, 1.0f, 0.0f); // far, green, larger
    glVertex3f(-0.9f, -0.9f, 0.5f);
    glVertex3f(0.9f, -0.9f, 0.5f);
    glVertex3f(0.0f, 0.95f, 0.5f);
    glEnd();
    glFinish();
    glReadPixels(canvas_width / 2, canvas_height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    const bool centre_good = near_colour(pixel, 255, 0, 0);
    line("centre (near red triangle in front of the green one): %u,%u,%u: %s", pixel[0], pixel[1], pixel[2],
         centre_good ? "ok" : "WRONG");
    glReadPixels(canvas_width / 2, canvas_height / 2 - 140, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    const bool ring_good = near_colour(pixel, 0, 255, 0);
    line("below the red triangle (green one only): %u,%u,%u: %s", pixel[0], pixel[1], pixel[2],
         ring_good ? "ok" : "WRONG");
    glReadPixels(4, canvas_height - 4, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    const bool corner_good = near_colour(pixel, 51, 102, 153);
    line("corner (background): %u,%u,%u: %s", pixel[0], pixel[1], pixel[2], corner_good ? "ok" : "WRONG");
    const GLenum error = glGetError();
    line("glGetError: 0x%x", error);
    failures += (centre_good ? 0 : 1) + (ring_good ? 0 : 1) + (corner_good ? 0 : 1) + (error ? 1 : 0);

    line("begin: OpenGL buffer swap and window readback");
    const BOOL swapped = SwapBuffers(dc);
    glFinish();
    pump(300);
    const COLORREF shown = GetPixel(dc, canvas_width / 2, canvas_height / 2);
    const bool shown_good = swapped && GetRValue(shown) >= 253 && GetGValue(shown) <= 2 && GetBValue(shown) <= 2;
    line("window centre after the swap: %u,%u,%u (swap %s): %s", GetRValue(shown), GetGValue(shown),
         GetBValue(shown), swapped ? "ok" : "failed", shown_good ? "ok" : "WRONG");
    failures += shown_good ? 0 : 1;
    pump(1200);
    line("begin: OpenGL context release");
    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(context);
    ReleaseDC(window, dc);
    DestroyWindow(window);
    line("end: OpenGL");
}

int ask(const wchar_t *text, UINT buttons) {
    if (unattended) {
        return IDYES;
    }
    return MessageBoxW(nullptr, text, L"RadeonCheck", buttons | MB_TOPMOST | MB_SETFOREGROUND);
}
} // namespace

int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR command_line, int) {
    unattended = command_line && wcsstr(command_line, L"/auto") != nullptr;
    if (ask(L"This checks the display driver: 2D drawing, the mouse pointer and OpenGL.\n\n"
            L"It takes about half a minute, draws in two small windows and moves the mouse pointer by itself. "
            L"Please do not touch the mouse or keyboard until it asks you a question.\n\n"
            L"It installs and changes nothing. A report is written beside this program.\n\nStart?",
            MB_YESNO | MB_ICONQUESTION) != IDYES) {
        return 0;
    }
    if (!open_report()) {
        ask(L"The report file could not be created.", MB_OK | MB_ICONERROR);
        return 2;
    }
    OSVERSIONINFOW version{};
    version.dwOSVersionInfoSize = sizeof(version);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    GetVersionExW(&version);
#pragma GCC diagnostic pop
    SYSTEMTIME now;
    GetLocalTime(&now);
    line("started %04u-%02u-%02u %02u:%02u:%02u, system %lu.%lu build %lu, %s", now.wYear, now.wMonth, now.wDay,
         now.wHour, now.wMinute, now.wSecond, version.dwMajorVersion, version.dwMinorVersion,
         version.dwBuildNumber, unattended ? "unattended" : "attended");
    describe_display();
    installer_status();

    if (HMODULE gdi = GetModuleHandleW(L"gdi32.dll")) {
        gradient_fill = reinterpret_cast<GradientFunction>(reinterpret_cast<void *>(GetProcAddress(gdi, "GdiGradientFill")));
    }
    line("gradient function: %s", gradient_fill ? "available" : "not available (case draws black only)");

    POINT original;
    GetCursorPos(&original);
    SetCursorPos(GetSystemMetrics(SM_CXSCREEN) - 2, GetSystemMetrics(SM_CYSCREEN) - 2);
    HWND window = make_window(L"RadeonCheck2D", 40, 40);
    unsigned failed_cases = 0, run_cases = 0, position_errors = 0, trail_pixels = 0, opengl_failures = 0;
    if (!window) {
        line("could not create the test window (error %lu)", GetLastError());
        failed_cases = 1;
    } else {
        check_drawing(window, failed_cases, run_cases);
        check_pointer(window, position_errors, trail_pixels);
    }
    SetCursorPos(original.x, original.y);

    int pointer_answer = IDCANCEL;
    if (!unattended) {
        pointer_answer = ask(L"Was the mouse pointer visible the whole time, did it change shape six times, "
                             L"and did it move without leaving marks behind?\n\n"
                             L"Yes = all three. No = something was wrong. Cancel = not sure.",
                             MB_YESNOCANCEL | MB_ICONQUESTION);
    }
    line("person's answer about the pointer: %s",
         unattended ? "not asked (unattended)" : pointer_answer == IDYES ? "yes, visible, changed shape, no marks"
                                                : pointer_answer == IDNO ? "NO, something was wrong" : "not sure");
    if (window) {
        DestroyWindow(window);
    }

    bool opengl_run = false;
    if (ask(L"The last check uses OpenGL. If the OpenGL driver fails, ReactOS may stop responding and need a "
            L"reset; the report will then show how far it got.\n\nRun the OpenGL check?",
            MB_YESNO | MB_ICONQUESTION) == IDYES) {
        opengl_run = true;
        check_opengl(opengl_failures);
    } else {
        line("== OpenGL: skipped by the person");
    }

    line("== summary");
    line("2D drawing: %u of %u cases differ from the memory reference", failed_cases, run_cases);
    line("pointer: %u positions not honoured, %u pixels changed along its path, person: %s", position_errors,
         trail_pixels, unattended ? "not asked" : pointer_answer == IDYES ? "yes" : pointer_answer == IDNO ? "no" : "not sure");
    if (opengl_run) {
        line("OpenGL: %u checks failed", opengl_failures);
    } else {
        line("OpenGL: not run");
    }
    line("finished");
    CloseHandle(report);
    ask(L"Finished. The report \"RadeonCheck report.txt\" is beside this program.", MB_OK | MB_ICONINFORMATION);
    return failed_cases || position_errors || opengl_failures ? 1 : 0;
}
