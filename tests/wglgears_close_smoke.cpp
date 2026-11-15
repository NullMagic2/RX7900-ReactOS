/*
Module: rx7900.tests.wglgears_close_smoke
Purpose: Close one explicitly launched software gears process and test owned GDI output.
Created: 2026-10-21
Architecture: Disposable-guest acceptance only. Flushed disk stages survive a hung
native call; finite user-mode waits cannot recover a kernel/GDI lockup. No kills,
OpenGL ICD changes, display-mode changes or reads of unrelated window pixels.
*/
#include "windows_support.hpp"
#include <stdio.h>

namespace {
constexpr DWORD startup_timeout_ms = 15000;
constexpr DWORD animation_ms = 2000;
constexpr DWORD close_timeout_ms = 10000;
constexpr DWORD response_timeout_ms = 2000;
constexpr DWORD poll_ms = 50;
constexpr unsigned message_budget = 64;
constexpr size_t maximum_windows = 16;
constexpr UINT probe_message = WM_APP + 31;
constexpr DWORD maximum_report_bytes = 65536;
constexpr int probe_width = 160;
constexpr int probe_height = 120;
constexpr wchar_t probe_class[] = L"RX7900OwnedGearsCloseProbe";

class Report final {
public:
    explicit Report(HANDLE file) noexcept : file_(file, INVALID_HANDLE_VALUE) {}
    bool line(const char* text) noexcept {
        size_t length = 0;

        if (!good_ || FAILED(StringCchLengthA(text, 4096, &length)) ||
            length > maximum_report_bytes - bytes_) { good_ = false; return false; }
        DWORD written = 0;

        if (!WriteFile(file_.get(), text, static_cast<DWORD>(length), &written, nullptr) || written != length ||
            !FlushFileBuffers(file_.get())) { good_ = false; return false; }
        bytes_ += written;
        return true;
    }
    bool good() const noexcept { return good_; }
    bool stage(const char* name) noexcept {
        char text[160]{};

        return SUCCEEDED(StringCchPrintfA(text, sizeof(text), "STAGE=%s\r\n", name)) && line(text);
    }
    void number(const char* name, DWORD value) noexcept {
        char text[160]{};

        if (SUCCEEDED(StringCchPrintfA(text, sizeof(text), "%s=%lu\r\n", name,
            static_cast<unsigned long>(value)))) line(text);
    }
private:
    rx7900::DeviceHandle file_;
    DWORD bytes_ = 0;
    bool good_ = true;
};

bool absolute_path(const wchar_t* path, const wchar_t* suffix) noexcept {
    size_t length = 0;

    if (FAILED(StringCchLengthW(path, MAX_PATH, &length)) || length < 7 ||
        !((path[0] >= L'A' && path[0] <= L'Z') || (path[0] >= L'a' && path[0] <= L'z')) ||
        path[1] != L':' || path[2] != L'\\' || lstrcmpiW(path + length - 4, suffix) != 0) return false;

    for (size_t index = 0; index < length; ++index) {
        if (path[index] < L' ' || path[index] == L'"' || (index != 1 && path[index] == L':')) return false;
    }
    return true;
}

struct OwnedWindows final {
    DWORD process_id = 0;
    HWND values[maximum_windows]{};
    size_t count = 0;
    bool overflow = false;
};

BOOL CALLBACK enumerate_owned(HWND window, LPARAM parameter) {
    auto& owned = *reinterpret_cast<OwnedWindows*>(parameter);
    DWORD process_id = 0;
    GetWindowThreadProcessId(window, &process_id);

    if (process_id != owned.process_id || !IsWindowVisible(window)) return TRUE;
    if (owned.count == maximum_windows) {
        owned.overflow = true;
        return FALSE;
    }
    owned.values[owned.count++] = window;
    return TRUE;
}

bool close_child(const wchar_t* application, Report& report) noexcept {
    wchar_t command[MAX_PATH + 4]{};

    if (FAILED(StringCchPrintfW(command, sizeof(command) / sizeof(command[0]), L"\"%s\"", application))) return false;
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION process{};

    if (!report.stage("create_owned_gears_process")) return false;
    if (!CreateProcessW(application, command, nullptr, nullptr, FALSE, 0,
        nullptr, nullptr, &startup, &process)) {
        report.number("CREATE_ERROR", GetLastError());
        return false;
    }
    rx7900::DeviceHandle child(process.hProcess);
    rx7900::DeviceHandle thread(process.hThread);
    report.number("CHILD_PID", process.dwProcessId);
    OwnedWindows owned{};
    owned.process_id = process.dwProcessId;
    const DWORD begin = GetTickCount();

    for (;;) {
        owned.count = 0;
        if (!report.stage("enumerate_child_windows")) return false;

        if (!EnumWindows(enumerate_owned, reinterpret_cast<LPARAM>(&owned)) || owned.overflow) {
            report.number("ENUM_ERROR", GetLastError());
            return false;
        }
        if (owned.count != 0) break;
        if (!report.stage("wait_child_window")) return false;

        if (WaitForSingleObject(child.get(), poll_ms) != WAIT_TIMEOUT ||
            GetTickCount() - begin >= startup_timeout_ms) {
            report.line("CHILD_WINDOW=unavailable; child left untouched\r\n");
            return false;
        }
    }
    report.number("CHILD_WINDOW_COUNT", static_cast<DWORD>(owned.count));
    if (!report.stage("allow_child_animation")) return false;
    if (WaitForSingleObject(child.get(), animation_ms) != WAIT_TIMEOUT) return false;
    bool requested = true;

    for (size_t index = 0; index < owned.count; ++index) {
        DWORD actual = 0;
        if (!report.stage("recheck_child_window_owner")) return false;
        GetWindowThreadProcessId(owned.values[index], &actual);

        if (actual != process.dwProcessId) {
            requested = false;
            continue;
        }
        if (!report.stage("post_owned_window_close")) return false;

        if (!PostMessageW(owned.values[index], WM_CLOSE, 0, 0)) {
            report.number("POST_CLOSE_ERROR", GetLastError());
            requested = false;
        }
    }
    if (!report.stage("wait_normal_child_exit")) return false;
    const DWORD waited = WaitForSingleObject(child.get(), close_timeout_ms);

    if (waited != WAIT_OBJECT_0) {
        report.number("CHILD_WAIT_RESULT", waited);
        report.line("CHILD_CLOSE=timeout_or_failure; no forced termination attempted\r\n");
        return false;
    }
    DWORD exit_code = STILL_ACTIVE;

    if (!GetExitCodeProcess(child.get(), &exit_code)) return false;
    report.number("CHILD_EXIT_CODE", exit_code);
    return requested && exit_code == 0;
}

struct WindowState final { bool responded = false; };

LRESULT CALLBACK probe_procedure(HWND window, UINT message, WPARAM first, LPARAM second) {
    if (message == WM_NCCREATE) {
        const auto* request = reinterpret_cast<const CREATESTRUCTW*>(second);
        SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(request->lpCreateParams));
    }
    if (message == probe_message) {
        auto* state = reinterpret_cast<WindowState*>(GetWindowLongPtrW(window, GWLP_USERDATA));

        if (state != nullptr) state->responded = true;
        return 0;
    }
    return DefWindowProcW(window, message, first, second);
}

class OwnedWindow final {
public:
    explicit OwnedWindow(Report& report) noexcept : report_(report) {}
    bool close() noexcept {
        if (value != nullptr) {
            if (!report_.stage("destroy_owned_probe_window") || !DestroyWindow(value)) return false;
            value = nullptr;
        }
        if (registered) {
            if (!report_.stage("unregister_owned_probe_class") || !UnregisterClassW(probe_class, instance)) return false;
            registered = false;
        }
        return true;
    }
    ~OwnedWindow() {
        if (value != nullptr) {
            report_.stage("destroy_owned_probe_window");
            DestroyWindow(value);
        }
        if (registered) {
            report_.stage("unregister_owned_probe_class");
            UnregisterClassW(probe_class, instance);
        }
    }
    HWND value = nullptr;
    HINSTANCE instance = nullptr;
    bool registered = false;
private:
    Report& report_;
};

bool desktop_probe(Report& report) noexcept {
    WindowState state;
    OwnedWindow window(report);
    window.instance = GetModuleHandleW(nullptr);
    WNDCLASSW description{};
    description.lpfnWndProc = probe_procedure;
    description.hInstance = window.instance;
    description.lpszClassName = probe_class;

    if (!report.stage("register_owned_probe_class")) return false;
    if (RegisterClassW(&description) == 0) return false;
    window.registered = true;
    RECT bounds{0, 0, probe_width, probe_height};
    if (!report.stage("size_owned_probe_window")) return false;
    if (!AdjustWindowRectEx(&bounds, WS_OVERLAPPEDWINDOW, FALSE, 0)) return false;
    if (!report.stage("create_owned_probe_window")) return false;
    window.value = CreateWindowExW(0, probe_class, L"Gears close: owned GDI probe",
        WS_OVERLAPPEDWINDOW, 40, 40, bounds.right - bounds.left, bounds.bottom - bounds.top,
        nullptr, nullptr, window.instance, &state);

    if (window.value == nullptr) return false;
    if (!report.stage("show_owned_probe_window")) return false;
    ShowWindow(window.value, SW_SHOWNORMAL);
    if (!report.stage("paint_owned_probe_window")) return false;
    UpdateWindow(window.value);
    if (!report.stage("post_owned_response_message")) return false;
    if (!PostMessageW(window.value, probe_message, 0, 0)) return false;
    const DWORD begin = GetTickCount();

    while (!state.responded && GetTickCount() - begin < response_timeout_ms) {
        MSG message{};
        if (!report.stage("pump_owned_response_message")) return false;

        for (unsigned count = 0; count < message_budget; ++count) {
            if (!PeekMessageW(&message, window.value, 0, 0, PM_REMOVE)) break;
            if (!report.stage("dispatch_owned_response_message")) return false;
            DispatchMessageW(&message);
        }
        if (!state.responded) Sleep(poll_ms);
    }
    if (!state.responded) return false;
    if (!report.stage("get_owned_probe_dc")) return false;
    HDC dc = GetDC(window.value);

    if (dc == nullptr) return false;
    constexpr COLORREF colors[2]{RGB(37, 83, 149), RGB(193, 47, 79)};
    bool pixels = true;

    for (size_t index = 0; index < sizeof(colors) / sizeof(colors[0]); ++index) {
        if (!report.stage("create_owned_color_brush")) { pixels = false; break; }
        HBRUSH brush = CreateSolidBrush(colors[index]);

        if (brush == nullptr) { pixels = false; break; }
        const RECT rectangle{0, 0, probe_width, probe_height};
        if (!report.stage("fill_owned_probe_pixels")) { DeleteObject(brush); pixels = false; break; }
        const bool filled = FillRect(dc, &rectangle, brush) != 0;
        if (!report.stage("delete_owned_color_brush")) { DeleteObject(brush); pixels = false; break; }
        DeleteObject(brush);
        if (!filled || !report.stage("flush_owned_probe_gdi")) { pixels = false; break; }
        if (!GdiFlush()) { pixels = false; break; }
        if (!report.stage("read_owned_probe_first_pixel")) { pixels = false; break; }
        const COLORREF first = GetPixel(dc, 8, 8);
        if (!report.stage("read_owned_probe_last_pixel")) { pixels = false; break; }
        const COLORREF last = GetPixel(dc, probe_width - 8, probe_height - 8);
        const bool exact = first == colors[index] && last == colors[index];
        report.number(index == 0 ? "PIXEL_ROUND_1" : "PIXEL_ROUND_2", exact ? 1U : 0U);
        pixels = pixels && exact;
    }
    if (!report.stage("release_owned_probe_dc")) pixels = false;
    if (ReleaseDC(window.value, dc) == 0) pixels = false;
    if (!window.close()) pixels = false;
    report.line(pixels ? "DESKTOP_PROBE=PASS\r\n" : "DESKTOP_PROBE=FAIL\r\n");
    return pixels;
}
}

int wmain(int argc, wchar_t** argv) {
    const wchar_t* application = L"C:\\wglgears.exe";
    const wchar_t* output = nullptr;
    bool application_set = false;

    for (int index = 1; index < argc; ++index) {
        if (lstrcmpW(argv[index], L"--output") == 0 && output == nullptr && index + 1 < argc) {
            output = argv[++index];
        } else if (lstrcmpW(argv[index], L"--application") == 0 && !application_set && index + 1 < argc) {
            application = argv[++index];
            application_set = true;
        } else {
            fputs("Usage: wglgears_close_smoke.exe --output C:\\report.log [--application C:\\wglgears.exe]\n", stderr);
            return 2;
        }
    }
    if (output == nullptr || (!absolute_path(output, L".log") && !absolute_path(output, L".txt")) ||
        !absolute_path(application, L".exe")) return 2;
    HANDLE file = CreateFileW(output, GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);

    if (file == INVALID_HANDLE_VALUE) return 2;
    Report report(file);
    if (!report.line("LIMIT=finite user-mode waits cannot recover kernel/GDI lockup; external VM timeout required\r\n")) return 1;
    const bool closed = close_child(application, report);
    report.line(closed ? "NORMAL_CHILD_CLOSE=PASS\r\n" : "NORMAL_CHILD_CLOSE=FAIL\r\n");
    const bool responsive = desktop_probe(report);
    const bool passed = closed && responsive;
    const bool recorded = report.line(passed ? "WGLGEARS_CLOSE_SMOKE=PASS\r\n" : "WGLGEARS_CLOSE_SMOKE=FAIL\r\n");
    return passed && recorded && report.good() ? 0 : 1;
}
