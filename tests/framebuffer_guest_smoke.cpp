/*
Module: rx7900.tests.framebuffer_guest_smoke
Purpose: Finite real GDI desktop acceptance and latency checks in a test guest.
Created: 2026-10-21
Architecture: Owns one window; optional mode cycle restores the saved mode.
*/
#include <windows.h>
#include <strsafe.h>
#include <stdio.h>
#include <stdint.h>

#include "windows_support.hpp"
#include "../src/reactos/framebuffer_shadow_abi.hpp"
#include "../src/reactos/lightning_present_abi.hpp"
#include "../src/reactos/lightning_gdi_abi.hpp"
#include "lightning-memory.h"

namespace {
constexpr unsigned frame_count = 96;
constexpr wchar_t window_class[] = L"RX7900FramebufferAcceptance";
using GdiObject = rx7900::ScopedHandle<HGDIOBJ, DeleteObject>;
using MemoryDC = rx7900::ScopedHandle<HDC, DeleteDC>;
using Window = rx7900::ScopedHandle<HWND, DestroyWindow>;

class Selection final {
public:
    Selection(HDC dc, HGDIOBJ object) noexcept : dc_(dc), old_(SelectObject(dc, object)) {}
    ~Selection() { if (valid()) SelectObject(dc_, old_); }
    bool valid() const noexcept { return old_ != nullptr && old_ != HGDI_ERROR; }
    Selection(const Selection&) = delete;
    Selection& operator=(const Selection&) = delete;
private:
    HDC dc_;
    HGDIOBJ old_;
};

// The GDI bitmap owns the bounded top-down pixel allocation. This wrapper
// restores its DC selection before deleting either native object.
class DibSurface final {
public:
    struct Difference final { LONG x=0,y=0; uint32_t expected=0,actual=0; };
    DibSurface(HDC compatible,LONG width,LONG height,WORD bits=32,const RGBQUAD* palette=nullptr,
        UINT palette_count=0) noexcept
        : dc_(CreateCompatibleDC(compatible)), width_(width), height_(height),bits_(bits) {
        if (!dc_.valid() || width<=0 || height<=0 ||
            static_cast<uint64_t>(width)*height*sizeof(uint32_t)>UINT32_MAX) return;
        if ((bits!=8 && bits!=16 && bits!=32) || palette_count>256 ||
            (palette_count!=0 && palette==nullptr)) return;
        struct Information final {BITMAPINFOHEADER header;RGBQUAD colors[256];};
        Information information{};
        information.header.biSize=sizeof(BITMAPINFOHEADER);
        information.header.biWidth=width;
        information.header.biHeight=-height;
        information.header.biPlanes=1;
        information.header.biBitCount=bits;
        information.header.biCompression=BI_RGB;
        information.header.biClrUsed=palette_count;
        for (UINT index=0;index<palette_count;++index) information.colors[index]=palette[index];
        // Owned native API header plus its complete bounded palette, no file
        // overlay. All formats use positive DWORD-aligned top-down rows.
        bitmap_=CreateDIBSection(compatible,reinterpret_cast<BITMAPINFO*>(&information),DIB_RGB_COLORS,&pixels_,nullptr,0);
        if (bitmap_!=nullptr) previous_=SelectObject(dc_.get(),bitmap_);
    }
    ~DibSurface() {
        if (previous_!=nullptr && previous_!=HGDI_ERROR) SelectObject(dc_.get(),previous_);
        if (bitmap_!=nullptr) DeleteObject(bitmap_);
    }
    DibSurface(const DibSurface&)=delete;
    DibSurface& operator=(const DibSurface&)=delete;
    bool valid() const noexcept {
        return bitmap_!=nullptr && pixels_!=nullptr && previous_!=nullptr && previous_!=HGDI_ERROR;
    }
    HDC get() const noexcept { return dc_.get(); }
    HBITMAP bitmap() const noexcept { return bitmap_; }
    bool write_pixel(LONG x,LONG y,uint32_t value) noexcept {
        if (!valid() || x<0 || y<0 || x>=width_ || y>=height_) return false;
        const size_t stride=((static_cast<size_t>(width_)*bits_+31)/32)*sizeof(uint32_t);
        auto* bytes=static_cast<uint8_t*>(pixels_)+static_cast<size_t>(y)*stride+static_cast<size_t>(x)*(bits_/8);
        for (UINT index=0;index<bits_/8;++index) bytes[index]=static_cast<uint8_t>(value>>(index*8));
        return true;
    }
    bool same_rgb(const DibSurface& other,Difference* difference=nullptr) const noexcept {
        if (!valid() || !other.valid() || bits_!=32 || other.bits_!=32 ||
            width_!=other.width_ || height_!=other.height_) return false;
        const size_t count=static_cast<size_t>(width_)*height_;
        const auto* left=static_cast<const uint32_t*>(pixels_);
        const auto* right=static_cast<const uint32_t*>(other.pixels_);
        for (size_t index=0;index<count;++index) {
            if ((left[index]&0x00ffffffU)!=(right[index]&0x00ffffffU)) {
                if (difference!=nullptr) {
                    difference->x=static_cast<LONG>(index%static_cast<size_t>(width_));
                    difference->y=static_cast<LONG>(index/static_cast<size_t>(width_));
                    difference->expected=left[index]&0x00ffffffU;
                    difference->actual=right[index]&0x00ffffffU;
                }
                return false;
            }
        }
        return true;
    }
private:
    MemoryDC dc_;
    HBITMAP bitmap_=nullptr;
    HGDIOBJ previous_=nullptr;
    void* pixels_=nullptr;
    LONG width_=0,height_=0;
    WORD bits_=32;
};

class WindowDC final {
public:
    explicit WindowDC(HWND window) noexcept : window_(window), dc_(GetDC(window)) {}
    ~WindowDC() { if (dc_ != nullptr) ReleaseDC(window_, dc_); }
    HDC get() const noexcept { return dc_; }
private:
    HWND window_;
    HDC dc_;
};

// File and serial handles are optional and explicitly requested; stdout is always
// emitted. Each write consumes only the initialized bounded formatted message.
class Log final {
public:
    Log(HANDLE file, HANDLE serial) noexcept : file_(file), serial_(serial) {}
    void line(const char* message) const {
        size_t count = 0;
        if (FAILED(StringCchLengthA(message, 512, &count))) return;
        write(file_, message, static_cast<DWORD>(count));
        write(serial_, message, static_cast<DWORD>(count));
        // Console painting exercises the driver too. Preserve the independent
        // file/serial progress first if a console draw blocks in that path.
        fputs(message, stdout);
        fflush(stdout);
    }
private:
    static void write(HANDLE handle, const char* data, DWORD length) {
        if (handle == INVALID_HANDLE_VALUE) return;
        DWORD offset = 0;
        while (offset < length) {
            DWORD written = 0;
            if (!WriteFile(handle, data + offset, length - offset, &written, nullptr) ||
                written == 0 || written > length - offset) return;
            offset += written;
        }
    }
    HANDLE file_;
    HANDLE serial_;
};

struct Scene final {
    HDC source = nullptr;
    bool painted = true;
    HCURSOR cursor = nullptr;
    bool cursor_override = false;
};

bool fill(HDC dc, const RECT& rectangle, COLORREF color) {
    GdiObject brush(CreateSolidBrush(color));
    return brush.valid() && FillRect(dc, &rectangle, reinterpret_cast<HBRUSH>(brush.get())) != 0;
}

bool draw_scene(HDC dc, Scene& scene, unsigned frame) {
    const RECT background{0,0,640,360};
    const RECT text_area{16,12,624,56};
    bool passed = fill(dc, background, RGB(20,30,40)) && fill(dc, text_area, RGB(255,255,255));
    if (SetBkMode(dc, TRANSPARENT) == 0 || SetTextColor(dc, RGB(0,0,0)) == CLR_INVALID) passed = false;
    constexpr wchar_t text[] = L"Framebuffer acceptance: text, overlap, scroll, alpha";
    passed = TextOutW(dc,24,20,text,static_cast<int>(sizeof(text)/sizeof(text[0])-1)) != FALSE && passed;
    passed = BitBlt(dc,32,96,128,64,scene.source,0,0,SRCCOPY) != FALSE && passed;
    // The destination overlaps the source by 96 pixels. Correct memmove order
    // must preserve the original red/green bands while drawing into this window.
    passed = BitBlt(dc,64,96,128,64,dc,32,96,SRCCOPY) != FALSE && passed;
    const RECT scroll{32,96,192,224};
    passed = ScrollDC(dc,0,32,&scroll,&scroll,nullptr,nullptr) != FALSE && passed;
    const BLENDFUNCTION blend{AC_SRC_OVER,0,128,0};
    passed = AlphaBlend(dc,240,96,64,64,scene.source,0,0,64,64,blend) != FALSE && passed;
    const RECT marker{320,100,320 + static_cast<LONG>(frame % 96U) + 1,132};
    passed = fill(dc,marker,RGB(60,100,220)) && passed;
    return GdiFlush() != FALSE && passed;
}

LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_NCCREATE) {
        const auto* create = reinterpret_cast<const CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(window,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(create->lpCreateParams));
    }
    if (message == WM_PAINT) {
        PAINTSTRUCT paint{};
        HDC dc = BeginPaint(window,&paint);
        auto* scene = reinterpret_cast<Scene*>(GetWindowLongPtrW(window,GWLP_USERDATA));
        if (scene != nullptr && scene->source != nullptr && dc != nullptr) {
            scene->painted = draw_scene(dc,*scene,0) && scene->painted;
        }
        EndPaint(window,&paint);
        return 0;
    }
    if (message == WM_SETCURSOR) {
        auto* scene = reinterpret_cast<Scene*>(GetWindowLongPtrW(window,GWLP_USERDATA));
        if (scene!=nullptr && scene->cursor_override) {
            SetCursor(scene->cursor);
            return TRUE;
        }
    }
    return DefWindowProcW(window,message,wparam,lparam);
}

void pump() {
    MSG message{};
    // Only this finite smoke process's queue is read. Other applications remain
    // untouched. Bound the drain if some external source continuously posts.
    for (unsigned count = 0; count < 512 && PeekMessageW(&message,nullptr,0,0,PM_REMOVE); ++count) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
}

class Checks final {
public:
    explicit Checks(const Log& log) noexcept : log_(log) {}
    void test(bool passed, const char* name) {
        ++checks;
        if (!passed) ++failures;
        char message[256]{};
        if (SUCCEEDED(StringCchPrintfA(message,sizeof(message),"CHECK %s %s\r\n",name,passed?"PASS":"FAIL"))) {
            log_.line(message);
        }
    }
    void pixels(bool valid,const DibSurface& reference,const DibSurface& captured,const char* name) {
        DibSurface::Difference difference{};
        const bool equal=reference.same_rgb(captured,&difference);
        test(valid && equal,name);
        if (!equal) {
            char message[256]{};
            if (SUCCEEDED(StringCchPrintfA(message,sizeof(message),
                "PIXEL_DIFFERENCE %s x=%ld y=%ld expected=%06lx actual=%06lx\r\n",name,
                difference.x,difference.y,static_cast<unsigned long>(difference.expected),
                static_cast<unsigned long>(difference.actual)))) log_.line(message);
        }
    }
    unsigned checks = 0;
    unsigned failures = 0;
private:
    const Log& log_;
};

bool near_color(COLORREF value, BYTE red, BYTE green, BYTE blue) {
    if (value == CLR_INVALID) return false;
    const int dr = static_cast<int>(GetRValue(value))-red;
    const int dg = static_cast<int>(GetGValue(value))-green;
    const int db = static_cast<int>(GetBValue(value))-blue;
    return dr >= -2 && dr <= 2 && dg >= -2 && dg <= 2 && db >= -2 && db <= 2;
}

void pixels(HDC dc, Checks& checks) {
    checks.test(GetPixel(dc,600,320)==RGB(20,30,40),"background-readback");
    checks.test(GetPixel(dc,610,40)==RGB(255,255,255),"text-background");
    checks.test(GetPixel(dc,80,144)==RGB(255,0,0),"overlap-scroll-red");
    checks.test(GetPixel(dc,160,144)==RGB(0,255,0),"overlap-scroll-green");
    checks.test(near_color(GetPixel(dc,260,112),138,15,20),"constant-alpha-readback");
    bool ink = false;
    for (int y = 20; y < 36 && !ink; ++y) {
        for (int x = 24; x < 200; x += 2) {
            const COLORREF pixel = GetPixel(dc,x,y);
            if (pixel != CLR_INVALID && GetRValue(pixel)<128 && GetGValue(pixel)<128 && GetBValue(pixel)<128) {
                ink = true;
                break;
            }
        }
    }
    checks.test(ink,"text-ink-readback");
}

bool fetch_shadow(HDC dc, rx7900::reactos::framebuffer::ShadowStatistics& statistics) {
    namespace abi=rx7900::reactos::framebuffer;
    const int returned=ExtEscape(dc,abi::query_shadow_escape,0,nullptr,sizeof(statistics),
        reinterpret_cast<LPSTR>(&statistics));
    return returned==sizeof(statistics) && statistics.size==sizeof(statistics) &&
        statistics.version==abi::shadow_statistics_version && statistics.magic==abi::shadow_statistics_magic;
}

bool shadow_statistics(HDC dc, const Log& log,
    rx7900::reactos::framebuffer::ShadowStatistics& statistics) {
    const bool valid=fetch_shadow(dc,statistics);
    char line[512]{};
    if (!valid) {
        log.line("SHADOW_PROOF unavailable (no compatible driver escape)\r\n");
        return false;
    }
    StringCchPrintfA(line,sizeof(line),
        "SHADOW_PROOF active=%u width=%u height=%u cached_stride=%u physical_stride=%u bitblt=%llu copy=%llu source_only=%llu flush=%llu flush_bytes=%llu failed=%llu\r\n",
        statistics.active,statistics.width,statistics.height,statistics.cached_stride,statistics.physical_stride,
        static_cast<unsigned long long>(statistics.bitblt_calls),static_cast<unsigned long long>(statistics.copy_calls),
        static_cast<unsigned long long>(statistics.source_only_calls),static_cast<unsigned long long>(statistics.flush_calls),
        static_cast<unsigned long long>(statistics.flush_bytes),static_cast<unsigned long long>(statistics.failed_calls));
    log.line(line);
    if (statistics.qpc_frequency!=0) {
        const double frequency=static_cast<double>(statistics.qpc_frequency);
        StringCchPrintfA(line,sizeof(line),
            "SHADOW_QPC render_ms=%.4f flush_ms=%.4f max_flush_ms=%.4f failures=%llu\r\n",
            1000.0*static_cast<double>(statistics.render_ticks)/frequency,
            1000.0*static_cast<double>(statistics.flush_ticks)/frequency,
            1000.0*static_cast<double>(statistics.max_flush_ticks)/frequency,
            static_cast<unsigned long long>(statistics.qpc_failures));
        log.line(line);
    }
    return true;
}

enum class Operation {fill,text,bitmap_copy,screen_copy,scroll,alpha};

bool operation(HDC dc, Scene& scene, Operation kind) {
    const RECT rectangle{32,96,192,224};
    const BLENDFUNCTION blend{AC_SRC_OVER,0,128,0};
    switch (kind) {
    case Operation::fill: return fill(dc,{0,0,640,360},RGB(20,30,40));
    case Operation::text: return TextOutW(dc,24,20,L"Real GDI text profiling",23)!=FALSE;
    case Operation::bitmap_copy: return BitBlt(dc,32,96,128,64,scene.source,0,0,SRCCOPY)!=FALSE;
    case Operation::screen_copy: return BitBlt(dc,64,96,128,64,dc,32,96,SRCCOPY)!=FALSE;
    case Operation::scroll: return ScrollDC(dc,0,1,&rectangle,&rectangle,nullptr,nullptr)!=FALSE;
    case Operation::alpha: return AlphaBlend(dc,240,96,64,64,scene.source,0,0,64,64,blend)!=FALSE;
    }
    return false;
}

void profile_operations(HDC dc, Scene& scene, LONGLONG frequency, const Log& log, Checks& checks) {
    struct Profile final {const char* name; Operation kind;};
    constexpr Profile profiles[]{{"fill",Operation::fill},{"text",Operation::text},
        {"bitmap-copy",Operation::bitmap_copy},{"screen-overlap-copy",Operation::screen_copy},
        {"scroll",Operation::scroll},{"alpha",Operation::alpha}};
    constexpr unsigned count=48;
    for (const auto& profile:profiles) {
        bool passed=draw_scene(dc,scene,0);
        rx7900::reactos::framebuffer::ShadowStatistics before{};
        const bool have_before=fetch_shadow(dc,before);
        double samples[count]{};
        double total=0;
        for (unsigned i=0;i<count;++i) {
            LARGE_INTEGER begin{};
            LARGE_INTEGER end{};
            passed=QueryPerformanceCounter(&begin)!=FALSE && passed;
            passed=operation(dc,scene,profile.kind) && passed;
            passed=GdiFlush()!=FALSE && passed;
            passed=QueryPerformanceCounter(&end)!=FALSE && passed;
            passed=end.QuadPart>=begin.QuadPart && passed;
            samples[i]=1000.0*static_cast<double>(end.QuadPart-begin.QuadPart)/static_cast<double>(frequency);
            total+=samples[i];
        }
        for (unsigned i=1;i<count;++i) {
            const double value=samples[i];
            unsigned at=i;
            while (at!=0 && samples[at-1]>value) {samples[at]=samples[at-1]; --at;}
            samples[at]=value;
        }
        char line[256]{};
        StringCchPrintfA(line,sizeof(line),
            "PROFILE operation=%s calls=%u mean_ms=%.4f p50_ms=%.4f p95_ms=%.4f max_ms=%.4f success=%u\r\n",
            profile.name,count,total/count,samples[count/2],samples[(count*95)/100],samples[count-1],passed?1U:0U);
        log.line(line);
        rx7900::reactos::framebuffer::ShadowStatistics after{};
        if (have_before && fetch_shadow(dc,after) && before.qpc_frequency!=0 &&
            before.qpc_frequency==after.qpc_frequency && after.render_ticks>=before.render_ticks &&
            after.flush_ticks>=before.flush_ticks && after.flush_bytes>=before.flush_bytes) {
            const double denominator=static_cast<double>(before.qpc_frequency)*count;
            StringCchPrintfA(line,sizeof(line),
                "PROFILE_DRIVER operation=%s render_mean_ms=%.4f flush_mean_ms=%.4f flush_bytes=%llu\r\n",
                profile.name,1000.0*static_cast<double>(after.render_ticks-before.render_ticks)/denominator,
                1000.0*static_cast<double>(after.flush_ticks-before.flush_ticks)/denominator,
                static_cast<unsigned long long>(after.flush_bytes-before.flush_bytes));
            log.line(line);
        }
        checks.test(passed,profile.name);
        pump();
    }
}

void mode_cycle(Checks& checks) {
    DEVMODEW original{};
    original.dmSize=sizeof(original);
    const bool saved=EnumDisplaySettingsW(nullptr,ENUM_CURRENT_SETTINGS,&original)!=FALSE;
    checks.test(saved,"mode-cycle-save-current");
    if (!saved) return;
    DEVMODEW alternate=original;
    alternate.dmFields=DM_PELSWIDTH|DM_PELSHEIGHT|DM_BITSPERPEL|DM_DISPLAYFREQUENCY;
    alternate.dmPelsWidth=1280;
    alternate.dmPelsHeight=720;
    alternate.dmBitsPerPel=32;
    alternate.dmDisplayFrequency=60;
    checks.test(ChangeDisplaySettingsW(&alternate,CDS_TEST)==DISP_CHANGE_SUCCESSFUL,"mode-cycle-supported-720p");
    const LONG switched=ChangeDisplaySettingsW(&alternate,CDS_FULLSCREEN);
    checks.test(switched==DISP_CHANGE_SUCCESSFUL,"mode-cycle-commit-720p");
    if (switched==DISP_CHANGE_SUCCESSFUL) {
        pump();
        checks.test(GetSystemMetrics(SM_CXSCREEN)==1280 && GetSystemMetrics(SM_CYSCREEN)==720,
                    "mode-cycle-actual-720p-desktop");
    }
    // Always restore after attempting a transition, including reported failure.
    checks.test(ChangeDisplaySettingsW(&original,0)==DISP_CHANGE_SUCCESSFUL,"mode-cycle-restore-original");
    pump();
    checks.test(GetSystemMetrics(SM_CXSCREEN)==1920 && GetSystemMetrics(SM_CYSCREEN)==1080,
                "mode-cycle-restored-1920x1080");
}

using CursorObject = rx7900::ScopedHandle<HCURSOR, DestroyCursor>;
using CursorState = lightning::reactos::CursorStatistics;

bool cursor_statistics(CursorState& state) {
    WindowDC desktop(nullptr);
    state={};
    return desktop.get()!=nullptr &&
        ExtEscape(desktop.get(),lightning::reactos::query_cursor_escape,0,nullptr,sizeof(state),
            reinterpret_cast<char*>(&state))==sizeof(state) &&
        state.size==sizeof(state) && state.version==lightning::reactos::cpu_scanout_abi_version &&
        state.magic==lightning::reactos::cursor_statistics_magic;
}

void log_cursor(const CursorState& state,const Log& log) {
    char message[512]{};
    if (SUCCEEDED(StringCchPrintfA(message,sizeof(message),
        "LIGHTNING_CURSOR generation=%u defined=%u visible=%u hardware=%u format=%u size=%ux%u hot=%u,%u position=%d,%d definitions=%llu moves=%llu hides=%llu declines=%llu failures=%llu recovery=%u\r\n",
        state.mode_generation,state.defined,state.visible,state.hardware_active,state.format,state.width,state.height,
        state.hot_x,state.hot_y,state.x,state.y,static_cast<unsigned long long>(state.definitions),
        static_cast<unsigned long long>(state.moves),static_cast<unsigned long long>(state.hides),
        static_cast<unsigned long long>(state.declines),static_cast<unsigned long long>(state.failures),
        state.recovery_required))) log.line(message);
}

// Owns only this acceptance window and its cursor selection. No cursor handle
// or bitmap view is retained by the driver after a shape call returns.
class CursorScope final {
public:
    CursorScope(HWND window,Scene& scene) noexcept : window_(window),scene_(scene),cursor_(GetCursor()) {
        position_valid_=GetCursorPos(&position_)!=FALSE;
        rectangle_valid_=GetWindowRect(window_,&rectangle_)!=FALSE;
        style_=GetWindowLongPtrW(window_,GWL_STYLE);
        topmost_=(GetWindowLongPtrW(window_,GWL_EXSTYLE)&WS_EX_TOPMOST)!=0;
        // A borderless owned client area prevents the OS's nonclient resizing
        // cursor from legitimately replacing the acceptance shape at corners.
        SetWindowLongPtrW(window_,GWL_STYLE,WS_POPUP|WS_VISIBLE);
        scene_.cursor_override=true;
        scene_.cursor=cursor_;
    }
    ~CursorScope() {
        scene_.cursor_override=false;
        scene_.cursor=nullptr;
        SetCursor(cursor_);
        SetWindowLongPtrW(window_,GWL_STYLE,style_);
        if (rectangle_valid_) SetWindowPos(window_,topmost_ ? HWND_TOPMOST : HWND_NOTOPMOST,
            rectangle_.left,rectangle_.top,rectangle_.right-rectangle_.left,
            rectangle_.bottom-rectangle_.top,SWP_NOACTIVATE|SWP_FRAMECHANGED);
        if (position_valid_) SetCursorPos(position_.x,position_.y);
    }
    void select(HCURSOR cursor) noexcept {
        scene_.cursor=cursor;
        SetCursor(cursor);
    }
private:
    HWND window_;
    Scene& scene_;
    HCURSOR cursor_;
    LONG_PTR style_=0;
    POINT position_{};
    RECT rectangle_{};
    bool position_valid_=false;
    bool rectangle_valid_=false;
    bool topmost_=false;
};

HCURSOR create_color_cursor(LONG width,bool alpha) {
    constexpr LONG height=32;
    BITMAPINFO information{};
    information.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);
    information.bmiHeader.biWidth=width;
    information.bmiHeader.biHeight=-height;
    information.bmiHeader.biPlanes=1;
    information.bmiHeader.biBitCount=32;
    information.bmiHeader.biCompression=BI_RGB;
    void* bytes=nullptr;
    GdiObject color(CreateDIBSection(nullptr,&information,DIB_RGB_COLORS,&bytes,nullptr,0));
    uint8_t mask_bytes[64*height/8]{};
    GdiObject mask(CreateBitmap(width,height,1,1,mask_bytes));
    if (!color.valid() || !mask.valid() || bytes==nullptr) return nullptr;
    // The DIB owns width*height initialized pixels. User32 constructs the
    // premultiplied alpha surface from this straight-alpha icon source.
    auto* pixels=static_cast<uint32_t*>(bytes);
    for (LONG pixel=0;pixel<width*height;++pixel) pixels[pixel]=alpha ? 0x80e06020U : 0x00e06020U;
    ICONINFO icon{};
    icon.fIcon=FALSE;
    icon.xHotspot=3;
    icon.yHotspot=5;
    icon.hbmMask=reinterpret_cast<HBITMAP>(mask.get());
    icon.hbmColor=reinterpret_cast<HBITMAP>(color.get());
    return reinterpret_cast<HCURSOR>(CreateIconIndirect(&icon));
}

void cursor_cycle(HWND window,Scene& scene,Checks& checks,const Log& log,bool cycle_modes,bool cursor_probe,bool cursor_fault_probe) {
    uint8_t and_mask[32*4]{};
    uint8_t xor_mask[32*4]{};
    memset(and_mask,0xff,sizeof(and_mask));
    and_mask[0]=0x7f;
    CursorObject exact(CreateCursor(GetModuleHandleW(nullptr),3,5,32,32,and_mask,xor_mask));
    and_mask[0]=0xff;
    xor_mask[0]=0x80;
    CursorObject inverted(CreateCursor(GetModuleHandleW(nullptr),3,5,32,32,and_mask,xor_mask));
    CursorObject color(create_color_cursor(32,false));
    CursorObject alpha(create_color_cursor(32,true));
    CursorObject oversized(create_color_cursor(64,false));
    const bool resources=exact.valid() && inverted.valid() && color.valid() && alpha.valid() && oversized.valid();
    checks.test(resources,"cursor-create-mono-invert-color-alpha-oversized");
    if (!resources) return;
    CursorScope scope(window,scene);
    checks.test(SetWindowPos(window,HWND_TOPMOST,0,0,GetSystemMetrics(SM_CXSCREEN),GetSystemMetrics(SM_CYSCREEN),
        SWP_NOACTIVATE|SWP_FRAMECHANGED)!=FALSE &&
        (GetWindowLongPtrW(window,GWL_STYLE)&static_cast<LONG_PTR>(WS_POPUP|WS_VISIBLE|WS_CAPTION|WS_THICKFRAME))==
            static_cast<LONG_PTR>(WS_POPUP|WS_VISIBLE),
        "cursor-owned-window-covers-desktop");
    scope.select(exact.get());
    checks.test(SetCursorPos(400,260)!=FALSE,"cursor-move-owned-window");
    pump();
    CursorState before{},state{};
    const bool query=cursor_statistics(before);
    checks.test(query && before.hardware_active==1 && before.defined==1 && before.visible==1 &&
        (before.format==LIGHTNING_SCANOUT_CURSOR_MONOCHROME ||
         before.format==LIGHTNING_SCANOUT_CURSOR_MASKED_COLOR) && before.hot_x==3 && before.hot_y==5 &&
        before.x==397 && before.y==255,"cursor-real-plane-mono-hotspot-ack");
    log_cursor(before,log);
    if (cursor_probe) {
        log.line("CURSOR_PROBE hotspot=400,260 hold_ms=30000\r\n");
        // Explicit acceptance-only pause lets the host observe the actual GTK
        // pointer, separately from accepted guest coordinates and counters.
        for (unsigned sample=0;sample<300;++sample) { Sleep(100); pump(); }
    }
    if (cursor_fault_probe) {
        bool recovered=false;
        for (unsigned attempt=0;attempt<32;++attempt) {
            SetCursorPos(402+attempt*2,262+attempt*2);
            Sleep(20);
            pump();
            if (cursor_statistics(state) && state.failures>before.failures &&
                state.hardware_active==0 && state.visible==0 && state.recovery_required==0) {
                recovered=true;
                break;
            }
        }
        CURSORINFO selected_cursor{};
        selected_cursor.cbSize=sizeof(selected_cursor);
        checks.test(recovered && GetCursorInfo(&selected_cursor)!=FALSE &&
            selected_cursor.hCursor==exact.get() && (selected_cursor.flags&CURSOR_SHOWING)!=0,
            "cursor-live-ui-failure-engine-owned-software-recovery");
        log_cursor(state,log);
        scope.select(color.get());
        scope.select(exact.get());
        checks.test(cursor_statistics(state) && state.hardware_active==1 && state.visible==1,
            "cursor-hardware-recovery-after-fault-cleared");
    }
    bool positions=true;
    const LONG last_x=GetSystemMetrics(SM_CXSCREEN)-1;
    const LONG last_y=GetSystemMetrics(SM_CYSCREEN)-1;
    const POINT corners[]{{0,0},{last_x,0},{last_x,last_y},{0,last_y},{400,260}};
    for (const auto& point:corners) {
        positions=SetCursorPos(point.x,point.y)!=FALSE && positions;
        Sleep(20);
        pump();
        const bool queried=cursor_statistics(state);
        POINT actual{};
        const bool located=GetCursorPos(&actual)!=FALSE;
        CURSORINFO selected{};
        selected.cbSize=sizeof(selected);
        ICONINFO selected_shape{};
        const bool shape_known=GetCursorInfo(&selected)!=FALSE && selected.hCursor!=nullptr &&
            GetIconInfo(selected.hCursor,&selected_shape)!=FALSE;
        GdiObject selected_mask(selected_shape.hbmMask),selected_color(selected_shape.hbmColor);
        // The shell may legitimately select its own cursor above a topmost
        // taskbar. Compare with the actual engine-owned shape at this point,
        // rather than mistaking its different hotspot for a driver offset.
        char message[160]{};
        StringCchPrintfA(message,sizeof(message),"CURSOR_CORNER hotspot=%ld,%ld actual=%ld,%ld queried=%u located=%u shape=%u hot=%lu,%lu\r\n",
            point.x,point.y,actual.x,actual.y,queried ? 1U : 0U,located ? 1U : 0U,
            shape_known ? 1U : 0U,selected_shape.xHotspot,selected_shape.yHotspot);
        log.line(message);
        log_cursor(state,log);
        positions=queried && located && actual.x==point.x && actual.y==point.y && shape_known &&
            state.hardware_active==1 && state.visible==1 &&
            state.hot_x==selected_shape.xHotspot && state.hot_y==selected_shape.yHotspot &&
            state.x==point.x-static_cast<LONG>(selected_shape.xHotspot) &&
            state.y==point.y-static_cast<LONG>(selected_shape.yHotspot) && positions;
    }
    checks.test(positions,"cursor-real-plane-corners-negative-top-left");
    checks.test(state.moves>before.moves,"cursor-native-move-ack-growth");
    scope.select(nullptr);
    checks.test(cursor_statistics(state) && state.visible==0,"cursor-real-plane-hide-ack");
    scope.select(exact.get());
    checks.test(cursor_statistics(state) && state.hardware_active==1 && state.visible==1,
        "cursor-real-plane-show-after-hide");
    scope.select(inverted.get());
    checks.test(cursor_statistics(state) && state.hardware_active==0 && state.visible==0,
        "cursor-background-inversion-exact-software-fallback");
    scope.select(color.get());
    checks.test(cursor_statistics(state) && state.hardware_active==1 && state.visible==1 &&
        state.format==LIGHTNING_SCANOUT_CURSOR_MASKED_COLOR,"cursor-real-plane-masked-color");
    scope.select(alpha.get());
    checks.test(cursor_statistics(state) && state.hardware_active==1 && state.visible==1 &&
        state.format==LIGHTNING_SCANOUT_CURSOR_COLOR,"cursor-real-plane-alpha-color");
    scope.select(oversized.get());
    checks.test(cursor_statistics(state) && state.hardware_active==0 && state.visible==0,
        "cursor-oversized-exact-software-fallback");
    scope.select(exact.get());
    checks.test(cursor_statistics(state) && state.hardware_active==1 && state.visible==1,
        "cursor-software-to-hardware-transition");
    if (cycle_modes) {
        DEVMODEW original{};
        original.dmSize=sizeof(original);
        const bool saved=EnumDisplaySettingsW(nullptr,ENUM_CURRENT_SETTINGS,&original)!=FALSE;
        checks.test(saved,"cursor-mode-save");
        if (saved) {
            const uint32_t generation=state.mode_generation;
            DEVMODEW smaller{};
            smaller.dmSize=sizeof(smaller);
            smaller.dmPelsWidth=1280;
            smaller.dmPelsHeight=720;
            smaller.dmBitsPerPel=32;
            smaller.dmFields=DM_PELSWIDTH|DM_PELSHEIGHT|DM_BITSPERPEL;
            checks.test(ChangeDisplaySettingsW(&smaller,0)==DISP_CHANGE_SUCCESSFUL,"cursor-live-mode-720p");
            pump();
            checks.test(cursor_statistics(state) && state.mode_generation!=generation &&
                state.hardware_active==1 && state.visible==1 && state.width==32 && state.height==32,
                "cursor-owned-shape-restored-after-mode-change");
            checks.test(ChangeDisplaySettingsW(&original,0)==DISP_CHANGE_SUCCESSFUL,"cursor-live-mode-restore");
            pump();
            checks.test(cursor_statistics(state) && state.hardware_active==1 && state.visible==1,
                "cursor-owned-shape-restored-1080p");
        }
    }
    log_cursor(state,log);
}

void complex_overlap_pixels(HDC device,Checks& checks) {
    constexpr LONG width=192,height=160,x=32,y=32;
    constexpr LONG clip_count=72,clip_step=2,clip_top=8,clip_bottom=136;
    DibSurface seed(device,width,height),reference(device,width,height),captured(device,width,height);
    bool initialized=seed.valid() && reference.valid() && captured.valid();
    for (LONG row=0;row<height && initialized;++row) {
        for (LONG column=0;column<width;++column) {
            const uint32_t value=((static_cast<uint32_t>(row*13+column*3)&255U)<<16) |
                ((static_cast<uint32_t>(row*7+column*17)&255U)<<8) |
                (static_cast<uint32_t>(row*19+column*5)&255U);
            initialized=seed.write_pixel(column,row,value) && initialized;
        }
    }
    GdiObject region(CreateRectRgn(0,0,0,0));
    initialized=region.valid() && initialized;
    for (LONG index=0;index<clip_count && initialized;++index) {
        const LONG left=8+index*clip_step;
        GdiObject strip(CreateRectRgn(left,clip_top,left+1,clip_bottom));
        initialized=strip.valid() && CombineRgn(reinterpret_cast<HRGN>(region.get()),
            reinterpret_cast<HRGN>(region.get()),reinterpret_cast<HRGN>(strip.get()),RGN_OR)!=ERROR;
    }
    struct Direction final {LONG x,y;const char* name;};
    const Direction directions[]{
        {2,2,"complex-overlap-right-down-original-source"},
        {-2,2,"complex-overlap-left-down-original-source"},
        {2,-2,"complex-overlap-right-up-original-source"},
        {-2,-2,"complex-overlap-left-up-original-source"}
    };
    for (const auto& direction:directions) {
        bool valid=initialized && BitBlt(reference.get(),0,0,width,height,seed.get(),0,0,SRCCOPY)!=FALSE &&
            BitBlt(device,x,y,width,height,seed.get(),0,0,SRCCOPY)!=FALSE;
        GdiObject actual_region(CreateRectRgn(0,0,0,0));
        valid=actual_region.valid() && CombineRgn(reinterpret_cast<HRGN>(actual_region.get()),
            reinterpret_cast<HRGN>(region.get()),nullptr,RGN_COPY)!=ERROR && valid;
        valid=OffsetRgn(reinterpret_cast<HRGN>(actual_region.get()),x,y)!=ERROR && valid;
        const int saved_device=SaveDC(device),saved_reference=SaveDC(reference.get());
        valid=saved_device!=0 && saved_reference!=0 && valid;
        if (valid) {
            valid=SelectClipRgn(reference.get(),reinterpret_cast<HRGN>(region.get()))!=ERROR &&
                SelectClipRgn(device,reinterpret_cast<HRGN>(actual_region.get()))!=ERROR;
            // The reference always reads the immutable seed. The display reads
            // its overlapping own pixels through the real native GDI fallback.
            valid=BitBlt(reference.get(),8,8,clip_count*clip_step,clip_bottom-clip_top,
                seed.get(),8-direction.x,8-direction.y,SRCCOPY)!=FALSE && valid;
            valid=BitBlt(device,x+8,y+8,clip_count*clip_step,clip_bottom-clip_top,
                device,x+8-direction.x,y+8-direction.y,SRCCOPY)!=FALSE && valid;
        }
        if (saved_device!=0) RestoreDC(device,saved_device);
        if (saved_reference!=0) RestoreDC(reference.get(),saved_reference);
        valid=GdiFlush()!=FALSE && valid;
        valid=BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE && valid;
        checks.pixels(valid,reference,captured,direction.name);
    }
}

void lightning_publication(HDC dc,Checks& checks,const Log& log) {
    constexpr int escape=lightning::reactos::query_display_escape;
    lightning::reactos::CpuScanoutInformation information{};
    bool queried=false;
    bool published=false;
    for (unsigned attempt=0;attempt<2000;++attempt) {
        queried=ExtEscape(dc,escape,0,nullptr,sizeof(information),reinterpret_cast<char*>(&information))==sizeof(information);
        if (!queried) break;
        if (information.accepted_epoch!=0 && information.accepted_sequence!=0 &&
            information.completed_epoch==information.accepted_epoch &&
            information.completed_sequence==information.accepted_sequence) {published=true; break;}
        Sleep(1);
    }
    checks.test(queried && information.size==sizeof(information) && information.version==1 &&
        information.enabled==1 && information.width==1920 && information.height==1080,
        "lightning-actual-private-display-owner");
    checks.test(published,"lightning-actual-cumulative-publication");
    char message[256]{};
    StringCchPrintfA(message,sizeof(message),
        "LIGHTNING_DISPLAY caps=%lu generation=%lu accepted=%lu:%lu completed=%lu:%lu refresh_ns=%lu\r\n",
        static_cast<unsigned long>(information.capabilities),static_cast<unsigned long>(information.mode_generation),
        static_cast<unsigned long>(information.accepted_epoch),static_cast<unsigned long>(information.accepted_sequence),
        static_cast<unsigned long>(information.completed_epoch),static_cast<unsigned long>(information.completed_sequence),
        static_cast<unsigned long>(information.refresh_ns));
    log.line(message);
}

bool gpu_statistics(HDC dc,lightning::reactos::gdi::Statistics& statistics);

void text_batch_pixels(HDC device,Checks& checks) {
    lightning::reactos::gdi::Statistics gpu_before{},gpu_after{};
    const bool measured=gpu_statistics(device,gpu_before);
    constexpr LONG width=320, height=48, x=16, y=240;
    DibSurface reference(device,width,height);
    DibSurface captured(device,width,height);
    LOGFONTW font_description{};
    bool valid=GetObjectW(GetStockObject(DEFAULT_GUI_FONT),sizeof(font_description),&font_description)==sizeof(font_description);
    font_description.lfUnderline=TRUE;
    GdiObject font(CreateFontIndirectW(&font_description));
    valid=valid && reference.valid() && captured.valid() && font.valid();
    checks.test(valid,"text-batch-reference-resources");
    if (!valid) return;
    Selection reference_font(reference.get(),font.get());
    Selection device_font(device,font.get());
    if (!reference_font.valid() || !device_font.valid()) {
        checks.test(false,"text-batch-reference-selection");
        return;
    }
    const int saved_device=SaveDC(device);
    const int saved_reference=SaveDC(reference.get());
    if (saved_device==0 || saved_reference==0) {
        if (saved_device!=0) RestoreDC(device,saved_device);
        if (saved_reference!=0) RestoreDC(reference.get(),saved_reference);
        checks.test(false,"text-batch-reference-state-save");
        return;
    }
    const RECT expected_rectangle{0,0,width,height};
    const RECT device_rectangle{x,y,x+width,y+height};
    valid=fill(reference.get(),expected_rectangle,RGB(30,40,50)) &&
        fill(device,device_rectangle,RGB(30,40,50));
    valid=SetTextAlign(reference.get(),TA_LEFT|TA_TOP|TA_UPDATECP)!=GDI_ERROR && valid;
    valid=SetTextAlign(device,TA_LEFT|TA_TOP|TA_UPDATECP)!=GDI_ERROR && valid;
    valid=SetBkColor(reference.get(),RGB(230,235,240))!=CLR_INVALID && valid;
    valid=SetBkColor(device,RGB(230,235,240))!=CLR_INVALID && valid;
    valid=SetTextColor(reference.get(),RGB(12,90,165))!=CLR_INVALID && valid;
    valid=SetTextColor(device,RGB(12,90,165))!=CLR_INVALID && valid;
    valid=MoveToEx(reference.get(),0,8,nullptr)!=FALSE && valid;
    valid=MoveToEx(device,x,y+8,nullptr)!=FALSE && valid;
    constexpr wchar_t text[]=L"Batch A B\u00a0C";
    valid=ExtTextOutW(reference.get(),0,0,ETO_CLIPPED|ETO_OPAQUE,&expected_rectangle,
        text,sizeof(text)/sizeof(text[0])-1,nullptr)!=FALSE && valid;
    valid=ExtTextOutW(device,x,y,ETO_CLIPPED|ETO_OPAQUE,&device_rectangle,
        text,sizeof(text)/sizeof(text[0])-1,nullptr)!=FALSE && valid;
    POINT expected_position{},actual_position{};
    valid=GetCurrentPositionEx(reference.get(),&expected_position)!=FALSE && valid;
    valid=GetCurrentPositionEx(device,&actual_position)!=FALSE && valid;
    valid=actual_position.x-x==expected_position.x && actual_position.y-y==expected_position.y && valid;
    valid=RestoreDC(device,saved_device)!=FALSE && valid;
    valid=RestoreDC(reference.get(),saved_reference)!=FALSE && valid;
    valid=GdiFlush()!=FALSE && valid;
    valid=BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE && valid;
    valid=GdiFlush()!=FALSE && valid;
    valid=reference.same_rgb(captured) && valid;
    checks.test(valid,"text-batch-underlined-clipped-opaque-updatecp-space-pixels");
    checks.test(measured && gpu_statistics(device,gpu_after) &&
        gpu_after.accepted_glyphs>gpu_before.accepted_glyphs && gpu_after.poisoned==0,
        "gpu-text-real-glyph-coverage-accepted");
    valid=PatBlt(device,x,y,32,24,BLACKNESS)!=FALSE && GetPixel(device,x+4,y+4)==RGB(0,0,0);
    valid=PatBlt(device,x,y,32,24,WHITENESS)!=FALSE && GetPixel(device,x+4,y+4)==RGB(255,255,255) && valid;
    checks.test(valid,"gpu-blackness-whiteness-exact-readback");
}

bool gpu_statistics(HDC dc,lightning::reactos::gdi::Statistics& statistics) {
    namespace abi=lightning::reactos::gdi;
    statistics={};
    return ExtEscape(dc,abi::query_statistics_escape,0,nullptr,sizeof(statistics),
        reinterpret_cast<char*>(&statistics))==sizeof(statistics) &&
        statistics.size==sizeof(statistics) && statistics.version==abi::statistics_abi_version &&
        statistics.device.size==sizeof(statistics.device) &&
        statistics.device.version==abi::native_abi_version && statistics.device.magic==LIGHTNING_GDI_MAGIC;
}

void raster_pixels(HDC device,Scene& scene,Checks& checks) {
    constexpr LONG width=64,height=32,x=16,y=296;
    DibSurface reference(device,width,height),captured(device,width,height);
    GdiObject brush(CreateSolidBrush(RGB(53,165,90)));
    checks.test(reference.valid() && captured.valid() && brush.valid(),"gpu-raster-reference-resources");
    if (!reference.valid() || !captured.valid() || !brush.valid()) return;
    Selection native_brush(device,brush.get()),reference_brush(reference.get(),brush.get());
    if (!native_brush.valid() || !reference_brush.valid()) {
        checks.test(false,"gpu-raster-reference-selection");
        return;
    }
    struct Vector final {DWORD operation;const char* name;};
    constexpr Vector vectors[]{
        {DSTINVERT,"gpu-raster-dstinvert-pixels"},
        {SRCINVERT,"gpu-raster-srcinvert-pixels"},
        {SRCAND,"gpu-raster-srcand-pixels"},
        {SRCPAINT,"gpu-raster-srcpaint-pixels"},
        {NOTSRCCOPY,"gpu-raster-notsrccopy-pixels"},
        {PATINVERT,"gpu-raster-patinvert-pixels"},
        {MERGECOPY,"gpu-raster-mergecopy-pixels"},
        {PATPAINT,"gpu-raster-patpaint-pixels"}
    };
    lightning::reactos::gdi::Statistics before{},after{};
    const bool measured=gpu_statistics(device,before);
    for (const auto& vector:vectors) {
        const RECT expected_bounds{0,0,width,height},actual_bounds{x,y,x+width,y+height};
        bool valid=fill(reference.get(),expected_bounds,RGB(37,72,139)) &&
            fill(device,actual_bounds,RGB(37,72,139));
        valid=BitBlt(reference.get(),0,0,width,height,scene.source,32,0,vector.operation)!=FALSE && valid;
        valid=BitBlt(device,x,y,width,height,scene.source,32,0,vector.operation)!=FALSE && valid;
        valid=BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE && valid;
        valid=GdiFlush()!=FALSE && valid;
        checks.pixels(valid,reference,captured,vector.name);
    }
    // Same-surface source and destination overlap. The GPU must snapshot the
    // entire original source before any destination clip can modify it.
    bool valid=BitBlt(reference.get(),0,0,width,height,scene.source,32,0,SRCCOPY)!=FALSE &&
        BitBlt(device,x,y,width,height,scene.source,32,0,SRCCOPY)!=FALSE;
    valid=BitBlt(reference.get(),8,4,width-8,height-4,reference.get(),0,0,SRCINVERT)!=FALSE && valid;
    valid=BitBlt(device,x+8,y+4,width-8,height-4,device,x,y,SRCINVERT)!=FALSE && valid;
    valid=BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE && valid;
    valid=GdiFlush()!=FALSE && valid;
    checks.pixels(valid,reference,captured,"gpu-raster-overlap-original-source-pixels");
    const uint64_t required=sizeof(vectors)/sizeof(vectors[0])+1;
    checks.test(measured && gpu_statistics(device,after) &&
        after.accepted_rasters>=before.accepted_rasters &&
        after.accepted_rasters-before.accepted_rasters>=required &&
        after.cpu_fallbacks==before.cpu_fallbacks && after.poisoned==0,
        "gpu-raster-calls-accepted-without-cpu-raster-fallback");
}

void masked_pattern_pixels(HDC device,Scene& scene,Checks& checks) {
    constexpr LONG width=64,height=32,x=16,y=296;
    DibSurface reference(device,width,height),captured(device,width,height),tile(device,3,2);
    uint8_t mask_bytes[height*(width/8)]{};
    for (LONG row=0;row<height;++row) {
        for (LONG column=0;column<width/8;++column) mask_bytes[row*(width/8)+column]=(row&1)!=0 ? 0x55 : 0xaa;
    }
    GdiObject mask(CreateBitmap(width,height,1,1,mask_bytes));
    const bool ready=reference.valid() && captured.valid() && tile.valid() && mask.valid();
    checks.test(ready,"gpu-masked-tile-reference-resources");
    if (!ready) return;
    const COLORREF colors[]{RGB(255,0,0),RGB(0,255,0),RGB(0,0,255),
        RGB(13,27,49),RGB(241,179,73),RGB(91,133,215)};
    for (LONG row=0;row<2;++row) {
        for (LONG column=0;column<3;++column) SetPixel(tile.get(),column,row,colors[row*3+column]);
    }
    GdiObject pattern(CreatePatternBrush(tile.bitmap()));
    if (!pattern.valid()) {checks.test(false,"gpu-realized-tile-brush");return;}
    Selection actual_brush(device,pattern.get()),reference_brush(reference.get(),pattern.get());
    if (!actual_brush.valid() || !reference_brush.valid()) {
        checks.test(false,"gpu-realized-brush-selection");return;
    }
    lightning::reactos::gdi::Statistics before{},after{};
    const bool measured=gpu_statistics(device,before);
    const RECT expected_bounds{0,0,width,height},actual_bounds{x,y,x+width,y+height};
    auto baseline=[&]() {
        return fill(reference.get(),expected_bounds,RGB(37,72,139)) &&
            fill(device,actual_bounds,RGB(37,72,139));
    };
    auto same=[&]() {
        return BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE &&
            GdiFlush()!=FALSE;
    };
    bool valid=baseline();
    const DWORD masked_rop=MAKEROP4(SRCCOPY,SRCINVERT);
    valid=MaskBlt(reference.get(),0,0,width,height,scene.source,32,0,
        reinterpret_cast<HBITMAP>(mask.get()),0,0,masked_rop)!=FALSE && valid;
    valid=MaskBlt(device,x,y,width,height,scene.source,32,0,
        reinterpret_cast<HBITMAP>(mask.get()),0,0,masked_rop)!=FALSE && valid;
    checks.pixels(valid && same(),reference,captured,"gpu-masked-rop4-source-and-destination-pixels");
    POINT old_reference{},old_actual{};
    const bool origins=SetBrushOrgEx(reference.get(),-5,3,&old_reference)!=FALSE &&
        SetBrushOrgEx(device,x-5,y+3,&old_actual)!=FALSE;
    valid=origins && baseline();
    valid=PatBlt(reference.get(),0,0,width,height,PATINVERT)!=FALSE && valid;
    valid=PatBlt(device,x,y,width,height,PATINVERT)!=FALSE && valid;
    checks.pixels(valid && same(),reference,captured,"gpu-tile-brush-signed-origin-pixels");
    valid=baseline();
    const DWORD masked_pattern=MAKEROP4(PATCOPY,0x00aa0029U);
    valid=MaskBlt(reference.get(),0,0,width,height,scene.source,0,0,
        reinterpret_cast<HBITMAP>(mask.get()),0,0,masked_pattern)!=FALSE && valid;
    valid=MaskBlt(device,x,y,width,height,scene.source,0,0,
        reinterpret_cast<HBITMAP>(mask.get()),0,0,masked_pattern)!=FALSE && valid;
    checks.pixels(valid && same(),reference,captured,"gpu-masked-tiled-pattern-pixels");
    if (origins) {
        SetBrushOrgEx(reference.get(),old_reference.x,old_reference.y,nullptr);
        SetBrushOrgEx(device,old_actual.x,old_actual.y,nullptr);
    }
    checks.test(measured && gpu_statistics(device,after) &&
        after.accepted_rasters>=before.accepted_rasters &&
        after.accepted_rasters-before.accepted_rasters>=3 &&
        after.cpu_fallbacks==before.cpu_fallbacks && after.poisoned==0,
        "gpu-mask-and-tile-accepted-without-cpu-raster-fallback");
}

void stretch_pixels(HDC device,Checks& checks) {
    constexpr LONG width=64,height=32,x=16,y=296;
    DibSurface source(device,11,7),reference(device,width,height),captured(device,width,height);
    const bool ready=source.valid() && reference.valid() && captured.valid();
    checks.test(ready,"gpu-stretch-reference-resources");
    if (!ready) return;
    for (LONG row=0;row<7;++row) {
        for (LONG column=0;column<11;++column) {
            SetPixel(source.get(),column,row,RGB(column*21,row*37,(column*17+row*23)&255));
        }
    }
    const RECT expected_bounds{0,0,width,height},actual_bounds{x,y,x+width,y+height};
    auto baseline=[&]() {
        return fill(reference.get(),expected_bounds,RGB(37,72,139)) &&
            fill(device,actual_bounds,RGB(37,72,139));
    };
    auto same=[&]() {
        return BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE &&
            GdiFlush()!=FALSE;
    };
    const int saved_device=SaveDC(device),saved_reference=SaveDC(reference.get());
    if (saved_device==0 || saved_reference==0) {
        if (saved_device!=0) RestoreDC(device,saved_device);
        if (saved_reference!=0) RestoreDC(reference.get(),saved_reference);
        checks.test(false,"gpu-stretch-reference-state-save");return;
    }
    SetStretchBltMode(reference.get(),COLORONCOLOR);
    SetStretchBltMode(device,COLORONCOLOR);
    lightning::reactos::gdi::Statistics before{},after{};
    const bool measured=gpu_statistics(device,before);
    bool valid=baseline();
    valid=StretchBlt(reference.get(),3,4,53,23,source.get(),0,0,11,7,SRCCOPY)!=FALSE && valid;
    valid=StretchBlt(device,x+3,y+4,53,23,source.get(),0,0,11,7,SRCCOPY)!=FALSE && valid;
    checks.pixels(valid && same(),reference,captured,"gpu-stretch-positive-upload-exact-pixels");

    valid=baseline();
    GdiObject expected_clip(CreateRectRgn(9,7,24,17)),expected_second(CreateRectRgn(32,12,52,25));
    GdiObject actual_clip(CreateRectRgn(x+9,y+7,x+24,y+17)),actual_second(CreateRectRgn(x+32,y+12,x+52,y+25));
    const bool regions=expected_clip.valid() && expected_second.valid() &&
        actual_clip.valid() && actual_second.valid();
    valid=regions && valid;
    if (regions) {
        valid=CombineRgn(reinterpret_cast<HRGN>(expected_clip.get()),reinterpret_cast<HRGN>(expected_clip.get()),
            reinterpret_cast<HRGN>(expected_second.get()),RGN_OR)!=ERROR && valid;
        valid=CombineRgn(reinterpret_cast<HRGN>(actual_clip.get()),reinterpret_cast<HRGN>(actual_clip.get()),
            reinterpret_cast<HRGN>(actual_second.get()),RGN_OR)!=ERROR && valid;
        valid=SelectClipRgn(reference.get(),reinterpret_cast<HRGN>(expected_clip.get()))!=ERROR && valid;
        valid=SelectClipRgn(device,reinterpret_cast<HRGN>(actual_clip.get()))!=ERROR && valid;
        valid=StretchBlt(reference.get(),3,4,53,23,source.get(),0,0,11,7,SRCCOPY)!=FALSE && valid;
        valid=StretchBlt(device,x+3,y+4,53,23,source.get(),0,0,11,7,SRCCOPY)!=FALSE && valid;
    }
    SelectClipRgn(reference.get(),nullptr);
    SelectClipRgn(device,nullptr);
    checks.pixels(valid && same(),reference,captured,"gpu-stretch-complex-clip-endpoint-pixels");

    valid=baseline();
    valid=BitBlt(reference.get(),0,0,11,7,source.get(),0,0,SRCCOPY)!=FALSE && valid;
    valid=BitBlt(device,x,y,11,7,source.get(),0,0,SRCCOPY)!=FALSE && valid;
    // The ReactOS DIB stretcher does not snapshot an aliased source. Use the
    // independent original pixels as the oracle; only the real screen call
    // deliberately overlaps, where the GPU must preserve its source snapshot.
    valid=StretchBlt(reference.get(),3,4,53,23,source.get(),0,0,11,7,SRCCOPY)!=FALSE && valid;
    valid=StretchBlt(device,x+3,y+4,53,23,device,x,y,11,7,SRCCOPY)!=FALSE && valid;
    checks.pixels(valid && same(),reference,captured,"gpu-stretch-screen-overlap-original-source-pixels");
    checks.test(measured && gpu_statistics(device,after) &&
        after.accepted_rasters>=before.accepted_rasters &&
        after.accepted_rasters-before.accepted_rasters>=3 &&
        after.cpu_fallbacks==before.cpu_fallbacks && after.poisoned==0,
        "gpu-positive-stretch-accepted-without-cpu-raster-fallback");

    // The paired kernel guard preserves direction before the engine normalizes
    // its driver rectangles. Compare real native pixels and command receipts.
    lightning::reactos::gdi::Statistics mirror_before{},mirror_after{};
    const bool mirror_measured=gpu_statistics(device,mirror_before);
    valid=baseline();
    valid=StretchBlt(reference.get(),56,4,-53,23,source.get(),0,0,11,7,SRCCOPY)!=FALSE && valid;
    valid=StretchBlt(device,x+56,y+4,-53,23,source.get(),0,0,11,7,SRCCOPY)!=FALSE && valid;
    checks.pixels(valid && same(),reference,captured,"gpu-stretch-mirror-preserves-original-direction");
    checks.test(mirror_measured && gpu_statistics(device,mirror_after) &&
        mirror_after.accepted_rasters>mirror_before.accepted_rasters &&
        mirror_after.cpu_fallbacks==mirror_before.cpu_fallbacks,"gpu-mirror-accepted-without-cpu-fallback");
    RestoreDC(device,saved_device);
    RestoreDC(reference.get(),saved_reference);
}

void extended_raster_pixels(HDC device,Checks& checks) {
    constexpr LONG width=64,height=32,x=16,y=296;
    RGBQUAD palette[4]{{0,0,0,0},{0,0,255,0},{0,0,255,0},{0,255,0,0}};
    DibSurface source(device,11,7),indexed(device,11,7,8,palette,4),low_color(device,11,7,16);
    DibSurface reference(device,width,height),captured(device,width,height);
    const bool ready=source.valid() && indexed.valid() && low_color.valid() && reference.valid() && captured.valid();
    checks.test(ready,"gpu-extended-raster-reference-resources");
    if (!ready) return;
    bool initialized=GdiFlush()!=FALSE;
    for (LONG row=0;row<7;++row) for (LONG column=0;column<11;++column) {
        const uint32_t alpha=64U+static_cast<uint32_t>((column*19+row*23)%192);
        const uint32_t pixel=(alpha<<24)|((alpha/2)<<16)|((alpha/3)<<8)|(alpha/4);
        initialized=source.write_pixel(column,row,pixel) &&
            indexed.write_pixel(column,row,1U+static_cast<uint32_t>((column+row)%3)) &&
            low_color.write_pixel(column,row,((column+row)&1)!=0 ? 0x7c00U : 0x03e0U) && initialized;
    }
    checks.test(initialized,"gpu-extended-raster-initialize-native-8-16-32bpp");
    const RECT expected_bounds{0,0,width,height},actual_bounds{x,y,x+width,y+height};
    auto baseline=[&]() {return fill(reference.get(),expected_bounds,RGB(37,72,139)) &&
        fill(device,actual_bounds,RGB(37,72,139));};
    auto same=[&]() {return BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE && GdiFlush()!=FALSE;};
    lightning::reactos::gdi::Statistics before{},after{};
    const bool measured=gpu_statistics(device,before);
    struct KeyVector final {DibSurface* source;COLORREF key;const char* name;};
    const KeyVector keys[]{
        {&source,RGB(32,21,16),"gpu-transparent-32bpp-scaled-pixels"},
        {&indexed,RGB(255,0,0),"gpu-transparent-palette-collision-raw-index-pixels"},
        {&low_color,RGB(255,0,0),"gpu-transparent-16bpp-raw-key-pixels"}
    };
    for (const auto& vector:keys) {
        bool valid=baseline();
        valid=TransparentBlt(reference.get(),3,4,53,23,vector.source->get(),2,1,7,5,vector.key)!=FALSE && valid;
        valid=TransparentBlt(device,x+3,y+4,53,23,vector.source->get(),2,1,7,5,vector.key)!=FALSE && valid;
        checks.pixels(valid && same(),reference,captured,vector.name);
    }
    struct AlphaVector final {DibSurface* source;BYTE constant;BYTE flags;const char* name;};
    const AlphaVector alphas[]{
        {&source,0,0,"gpu-alpha-stretch-zero-pixels"},
        {&source,1,0,"gpu-alpha-stretch-small-constant-pixels"},
        {&source,128,0,"gpu-alpha-stretch-32bpp-constant-pixels"},
        {&source,255,AC_SRC_ALPHA,"gpu-alpha-stretch-premultiplied-pixels"},
        {&source,128,AC_SRC_ALPHA,"gpu-alpha-stretch-premultiplied-constant-pixels"},
        {&indexed,128,0,"gpu-alpha-stretch-palette-pixels"},
        {&low_color,128,0,"gpu-alpha-stretch-16bpp-pixels"}
    };
    for (const auto& vector:alphas) {
        bool valid=baseline();
        const BLENDFUNCTION blend{AC_SRC_OVER,0,vector.constant,vector.flags};
        valid=AlphaBlend(reference.get(),3,4,53,23,vector.source->get(),2,1,7,5,blend)!=FALSE && valid;
        valid=AlphaBlend(device,x+3,y+4,53,23,vector.source->get(),2,1,7,5,blend)!=FALSE && valid;
        checks.pixels(valid && same(),reference,captured,vector.name);
    }
    const int saved_actual=SaveDC(device),saved_reference=SaveDC(reference.get());
    if (saved_actual!=0 && saved_reference!=0) {
        bool valid=baseline();
        valid=IntersectClipRect(reference.get(),9,7,45,25)!=ERROR && valid;
        valid=IntersectClipRect(device,x+9,y+7,x+45,y+25)!=ERROR && valid;
        const BLENDFUNCTION blend{AC_SRC_OVER,0,128,AC_SRC_ALPHA};
        valid=AlphaBlend(reference.get(),3,4,53,23,source.get(),2,1,7,5,blend)!=FALSE && valid;
        valid=AlphaBlend(device,x+3,y+4,53,23,source.get(),2,1,7,5,blend)!=FALSE && valid;
        SelectClipRgn(reference.get(),nullptr);SelectClipRgn(device,nullptr);
        checks.pixels(valid && same(),reference,captured,"gpu-alpha-stretch-clipped-endpoint-pixels");
    } else checks.test(false,"gpu-alpha-stretch-save-clip-state");
    if (saved_actual!=0) RestoreDC(device,saved_actual);
    if (saved_reference!=0) RestoreDC(reference.get(),saved_reference);
    checks.test(measured && gpu_statistics(device,after) && after.accepted_rasters-before.accepted_rasters>=11 &&
        after.cpu_fallbacks==before.cpu_fallbacks && after.poisoned==0,
        "gpu-key-and-stretched-alpha-accepted-without-cpu-fallback");

    for (unsigned stretched=0;stretched<2;++stretched) {
        const LONG destination_width=stretched!=0 ? 27 : 12;
        const LONG destination_height=stretched!=0 ? 17 : 8;
        const RECT expected_source{2,2,14,10},actual_source{x+2,y+2,x+14,y+10};
        bool valid=baseline() && fill(reference.get(),expected_source,RGB(217,83,41)) &&
            fill(device,actual_source,RGB(217,83,41)) && GdiFlush()!=FALSE;
        lightning::reactos::gdi::Statistics alpha_before{},alpha_after{};
        const bool alpha_measured=gpu_statistics(device,alpha_before);
        const BLENDFUNCTION blend{AC_SRC_OVER,0,128,0};
        valid=AlphaBlend(reference.get(),32,12,destination_width,destination_height,
            reference.get(),2,2,12,8,blend)!=FALSE && valid;
        valid=AlphaBlend(device,x+32,y+12,destination_width,destination_height,
            device,x+2,y+2,12,8,blend)!=FALSE && valid;
        checks.test(alpha_measured && gpu_statistics(device,alpha_after) &&
            alpha_after.accepted_rasters>alpha_before.accepted_rasters &&
            alpha_after.cpu_fallbacks==alpha_before.cpu_fallbacks &&
            alpha_after.readbacks==alpha_before.readbacks && alpha_after.poisoned==0,
            "gpu-screen-alpha-uses-gpu-source-without-readback");
        checks.pixels(valid && same(),reference,captured,stretched!=0 ?
            "gpu-screen-alpha-stretched-pixels" : "gpu-screen-alpha-equal-size-pixels");
    }

    for (LONG origin=0;origin<2;++origin) {
        const int device_state=SaveDC(device),reference_state=SaveDC(reference.get());
        bool valid=device_state!=0 && reference_state!=0 && baseline();
        valid=IntersectClipRect(reference.get(),3,4,4,5)!=ERROR && valid;
        valid=IntersectClipRect(device,x+3,y+4,x+4,y+5)!=ERROR && valid;
        lightning::reactos::gdi::Statistics guard_before{},guard_after{};
        const bool guard_measured=gpu_statistics(device,guard_before);
        valid=StretchBlt(reference.get(),56,4,-53,23,source.get(),origin,origin,7,5,SRCCOPY)!=FALSE && valid;
        valid=StretchBlt(device,x+56,y+4,-53,23,source.get(),origin,origin,7,5,SRCCOPY)!=FALSE && valid;
        SelectClipRgn(reference.get(),nullptr);SelectClipRgn(device,nullptr);
        checks.pixels(valid && same(),reference,captured,origin==0 ?
            "gpu-mirror-zero-extent-missing-edge-preserves-destination" :
            "gpu-mirror-zero-extent-owned-preceding-source-pixels");
        checks.test(guard_measured && gpu_statistics(device,guard_after) &&
            guard_after.accepted_rasters>guard_before.accepted_rasters &&
            guard_after.cpu_fallbacks==guard_before.cpu_fallbacks,
            "gpu-mirror-zero-extent-accepted-without-cpu-fallback");
        if (device_state!=0) RestoreDC(device,device_state);
        if (reference_state!=0) RestoreDC(reference.get(),reference_state);
    }

    GdiObject pen(CreatePen(PS_SOLID,1,RGB(243,81,37)));
    Selection actual_pen(device,pen.get()),reference_pen(reference.get(),pen.get());
    if (!pen.valid() || !actual_pen.valid() || !reference_pen.valid()) {
        checks.test(false,"gpu-cosmetic-line-reference-pen");return;
    }
    bool valid=baseline();
    const POINT ends[]{{55,19},{55,27},{31,27},{3,27},{3,19},{3,3},{31,3},{55,3}};
    lightning::reactos::gdi::Statistics line_before{},line_after{};
    const bool line_measured=gpu_statistics(device,line_before);
    for (const auto& end:ends) {
        valid=MoveToEx(reference.get(),31,15,nullptr)!=FALSE &&
            MoveToEx(device,x+31,y+15,nullptr)!=FALSE && valid;
        valid=LineTo(reference.get(),end.x,end.y)!=FALSE && LineTo(device,x+end.x,y+end.y)!=FALSE && valid;
    }
    checks.pixels(valid && same(),reference,captured,"gpu-cosmetic-line-all-octants-pixels");
    checks.test(line_measured && gpu_statistics(device,line_after) &&
        line_after.accepted_rasters-line_before.accepted_rasters>=8 &&
        line_after.cpu_fallbacks==line_before.cpu_fallbacks && line_after.poisoned==0,
        "gpu-cosmetic-lines-accepted-without-cpu-fallback");
}

void paint_pixels(HDC device,Checks& checks) {
    constexpr LONG width=64,height=32,x=16,y=336;
    constexpr COLORREF foreground=RGB(217,83,41),background=RGB(11,31,91),baseline_color=RGB(37,72,139);
    DibSurface reference(device,width,height),captured(device,width,height);
    GdiObject solid(CreateSolidBrush(foreground)),hatch(CreateHatchBrush(HS_DIAGCROSS,foreground));
    GdiObject region(CreateRectRgn(3,4,53,15)),other(CreateRectRgn(9,15,43,28));
    GdiObject actual(CreateRectRgn(0,0,0,0));
    const bool ready=reference.valid() && captured.valid() && solid.valid() && hatch.valid() &&
        region.valid() && other.valid() && actual.valid() &&
        CombineRgn(reinterpret_cast<HRGN>(region.get()),reinterpret_cast<HRGN>(region.get()),
            reinterpret_cast<HRGN>(other.get()),RGN_OR)!=ERROR &&
        CombineRgn(reinterpret_cast<HRGN>(actual.get()),reinterpret_cast<HRGN>(region.get()),nullptr,RGN_COPY)!=ERROR &&
        OffsetRgn(reinterpret_cast<HRGN>(actual.get()),x,y)!=ERROR;
    checks.test(ready,"gpu-paint-region-reference-resources");
    if (!ready) return;
    for (unsigned patterned=0;patterned<2;++patterned) for (unsigned transparent=0;transparent<2;++transparent) {
        const int reference_state=SaveDC(reference.get()),device_state=SaveDC(device);
        bool valid=reference_state!=0 && device_state!=0 &&
            fill(reference.get(),{0,0,width,height},baseline_color) &&
            fill(device,{x,y,x+width,y+height},baseline_color);
        const auto brush=reinterpret_cast<HBRUSH>(patterned!=0 ? hatch.get() : solid.get());
        SetBkColor(reference.get(),background);SetBkColor(device,background);
        SetBkMode(reference.get(),OPAQUE);SetBkMode(device,transparent!=0 ? TRANSPARENT : OPAQUE);
        SetBrushOrgEx(reference.get(),-5,3,nullptr);SetBrushOrgEx(device,x-5,y+3,nullptr);
        valid=FillRgn(reference.get(),reinterpret_cast<HRGN>(region.get()),brush)!=FALSE && valid;
        if (patterned!=0 && transparent!=0) {
            // Build the transparent reference from the engine's opaque hatch.
            // Avoid its pinned implicit-mask/null-mask-point fallback defect.
            valid=GdiFlush()!=FALSE && valid;
            for (LONG row=0;row<height;++row) for (LONG column=0;column<width;++column)
                if (GetPixel(reference.get(),column,row)==background)
                    valid=SetPixel(reference.get(),column,row,baseline_color)!=CLR_INVALID && valid;
        }
        lightning::reactos::gdi::Statistics before{},after{};
        const bool measured=gpu_statistics(device,before);
        valid=FillRgn(device,reinterpret_cast<HRGN>(actual.get()),brush)!=FALSE && valid;
        valid=GdiFlush()!=FALSE && valid;
        checks.test(measured && gpu_statistics(device,after) &&
            after.accepted_fills+after.accepted_rasters>before.accepted_fills+before.accepted_rasters &&
            after.cpu_fallbacks==before.cpu_fallbacks && after.poisoned==0,
            "gpu-region-paint-accepted-without-cpu-fallback");
        valid=BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE && GdiFlush()!=FALSE && valid;
        checks.pixels(valid,reference,captured,patterned!=0 ?
            (transparent!=0 ? "gpu-transparent-hatch-region-pixels" : "gpu-opaque-hatch-region-pixels") :
            "gpu-solid-region-paint-pixels");
        if (reference_state!=0) RestoreDC(reference.get(),reference_state);
        if (device_state!=0) RestoreDC(device,device_state);
    }
}

void gradient_pixels(HDC device,Checks& checks) {
    constexpr LONG width=64,height=32,x=16,y=296;
    DibSurface reference(device,width,height),captured(device,width,height);
    checks.test(reference.valid() && captured.valid(),"gpu-gradient-reference-resources");
    if (!reference.valid() || !captured.valid()) return;
    const RECT expected_bounds{0,0,width,height},actual_bounds{x,y,x+width,y+height};
    auto baseline=[&]() {return fill(reference.get(),expected_bounds,RGB(37,72,139)) &&
        fill(device,actual_bounds,RGB(37,72,139));};
    auto same=[&]() {return BitBlt(captured.get(),0,0,width,height,device,x,y,SRCCOPY)!=FALSE && GdiFlush()!=FALSE;};
    lightning::reactos::gdi::Statistics before{},after{};
    const bool measured=gpu_statistics(device,before);
    const GRADIENT_RECT mesh{0,1};
    for (ULONG mode=GRADIENT_FILL_RECT_H;mode<=GRADIENT_FILL_RECT_V;++mode) {
        for (unsigned reversed=0;reversed<2;++reversed) {
            TRIVERTEX expected[2]{{3,4,0xef35,0x1321,0x9743,0},{57,28,0x2149,0xdfe3,0x3155,0}};
            if (reversed!=0) {const auto value=expected[0];expected[0]=expected[1];expected[1]=value;}
            TRIVERTEX actual[2]{expected[0],expected[1]};
            for (auto& vertex:actual) {vertex.x+=x;vertex.y+=y;}
            bool valid=baseline();
            valid=GradientFill(reference.get(),expected,2,const_cast<GRADIENT_RECT*>(&mesh),1,mode)!=FALSE && valid;
            valid=GradientFill(device,actual,2,const_cast<GRADIENT_RECT*>(&mesh),1,mode)!=FALSE && valid;
            checks.pixels(valid && same(),reference,captured,mode==GRADIENT_FILL_RECT_H ?
                "gpu-horizontal-gradient-integer-pixels" : "gpu-vertical-gradient-integer-pixels");
        }
    }
    constexpr unsigned orders[6][3]{{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    const GRADIENT_TRIANGLE triangle{0,1,2};
    for (unsigned solid=0;solid<2;++solid) {
        for (const auto& order:orders) {
            const TRIVERTEX vertices[3]{{3,4,0xe131,0x1b17,0x9f23,0},
                {56,11,0x2339,0xeb41,0x3d19,0},{22,28,0x4d53,0x3947,0xd75b,0}};
            TRIVERTEX expected[3]{vertices[order[0]],vertices[order[1]],vertices[order[2]]};
            if (solid!=0) for (auto& vertex:expected) {vertex.Red=0x7311;vertex.Green=0xa333;vertex.Blue=0x4355;}
            TRIVERTEX actual[3]{expected[0],expected[1],expected[2]};
            for (auto& vertex:actual) {vertex.x+=x;vertex.y+=y;}
            const int device_state=SaveDC(device),reference_state=SaveDC(reference.get());
            bool valid=device_state!=0 && reference_state!=0 && baseline();
            valid=IntersectClipRect(reference.get(),9,9,48,25)!=ERROR && valid;
            valid=IntersectClipRect(device,x+9,y+9,x+48,y+25)!=ERROR && valid;
            valid=GradientFill(reference.get(),expected,3,const_cast<GRADIENT_TRIANGLE*>(&triangle),1,GRADIENT_FILL_TRIANGLE)!=FALSE && valid;
            valid=GradientFill(device,actual,3,const_cast<GRADIENT_TRIANGLE*>(&triangle),1,GRADIENT_FILL_TRIANGLE)!=FALSE && valid;
            SelectClipRgn(reference.get(),nullptr);SelectClipRgn(device,nullptr);
            checks.pixels(valid && same(),reference,captured,solid!=0 ?
                "gpu-solid-triangle-clipped-native-pixels" : "gpu-gradient-triangle-ordered-integer-pixels");
            if (device_state!=0) RestoreDC(device,device_state);
            if (reference_state!=0) RestoreDC(reference.get(),reference_state);
        }
    }
    checks.test(measured && gpu_statistics(device,after) &&
        after.accepted_rasters-before.accepted_rasters>=16 && after.cpu_fallbacks==before.cpu_fallbacks &&
        after.poisoned==0,"gpu-gradients-accepted-without-cpu-fallback");
    // GdiFlush must close the engine batch and observe actual device completion.
    checks.test(GdiFlush()!=FALSE && gpu_statistics(device,after) &&
        after.device.accepted_sequence==after.device.completed_sequence && after.device.quarantined==0,
        "gdi-flush-observes-real-gpu-completion");
}

void gpu_progress(HDC dc,Checks& checks,const Log& log,
                  const lightning::reactos::gdi::Statistics& before) {
    lightning::reactos::gdi::Statistics after{};
    bool queried=false;
    bool published=false;
    const DWORD start=GetTickCount();
    constexpr DWORD publication_timeout_ms=5000;

    do {
        queried=gpu_statistics(dc,after);
        if (!queried || after.poisoned!=0 || after.device.quarantined!=0) break;
        const auto& device=after.device;
        published=device.active==1 && device.canvas_revision!=0 &&
            device.completed_sequence>=device.canvas_revision &&
            device.published_sequence>=device.canvas_revision && device.published_epoch==device.epoch &&
            device.published_mode_generation==device.mode_generation;
        if (published) break;
        Sleep(1);
    } while (static_cast<DWORD>(GetTickCount()-start)<publication_timeout_ms);
    checks.test(queried && after.enabled==1 && after.device.active==1 &&
        after.poisoned==0 && after.device.quarantined==0,"gpu-active-without-quarantine");
    checks.test(queried && after.accepted_fills>before.accepted_fills,"gpu-accepted-fill-growth");
    checks.test(queried && after.accepted_copies>before.accepted_copies,"gpu-accepted-screen-copy-growth");
    checks.test(queried && after.accepted_uploads>before.accepted_uploads,"gpu-accepted-upload-growth");
    checks.test(queried && after.accepted_blends>before.accepted_blends,"gpu-accepted-blend-growth");
    checks.test(queried && after.readbacks>before.readbacks,"gpu-synchronized-readback-growth");
    checks.test(published,"gpu-completed-and-console-published-current-revision");
    char message[512]{};
    StringCchPrintfA(message,sizeof(message),
        "LIGHTNING_GDI enabled=%lu poisoned=%lu generation=%lu epoch=%lu accepted=%lu completed=%lu revision=%lu published=%lu:%lu:%lu\r\n",
        static_cast<unsigned long>(after.enabled),static_cast<unsigned long>(after.poisoned),
        static_cast<unsigned long>(after.device.mode_generation),static_cast<unsigned long>(after.device.epoch),
        static_cast<unsigned long>(after.device.accepted_sequence),static_cast<unsigned long>(after.device.completed_sequence),
        static_cast<unsigned long>(after.device.canvas_revision),static_cast<unsigned long>(after.device.published_mode_generation),
        static_cast<unsigned long>(after.device.published_epoch),static_cast<unsigned long>(after.device.published_sequence));
    log.line(message);
    StringCchPrintfA(message,sizeof(message),
        "LIGHTNING_GDI accepted_fills=%I64u accepted_copies=%I64u accepted_uploads=%I64u accepted_blends=%I64u readbacks=%I64u cpu_fallbacks=%I64u accepted_rasters=%I64u accepted_glyphs=%I64u\r\n",
        after.accepted_fills,after.accepted_copies,after.accepted_uploads,after.accepted_blends,
        after.readbacks,after.cpu_fallbacks,after.accepted_rasters,after.accepted_glyphs);
    log.line(message);
}

int exercise(const Log& log, bool require_shadow,bool cycle_modes,bool require_lightning,bool require_gpu,bool cursor_probe,bool cursor_fault_probe) {
    Checks checks(log);
    const int screen_width = GetSystemMetrics(SM_CXSCREEN);
    const int screen_height = GetSystemMetrics(SM_CYSCREEN);
    char message[512]{};
    StringCchPrintfA(message,sizeof(message),"FRAMEBUFFER_SMOKE screen=%dx%d frames=%u\r\n",
                     screen_width,screen_height,frame_count);
    log.line(message);
    checks.test(screen_width==1920 && screen_height==1080,"desktop-1920x1080");
    if (cycle_modes) mode_cycle(checks);

    LARGE_INTEGER frequency{};
    if (!QueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0) {
        log.line("FRAMEBUFFER_SMOKE_FAIL no-QPC\r\n");
        return 1;
    }
    MemoryDC memory(CreateCompatibleDC(nullptr));
    BITMAPINFO bitmap{};
    bitmap.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bitmap.bmiHeader.biWidth = 128;
    bitmap.bmiHeader.biHeight = -64;
    bitmap.bmiHeader.biPlanes = 1;
    bitmap.bmiHeader.biBitCount = 32;
    bitmap.bmiHeader.biCompression = BI_RGB;
    void* raw_pixels = nullptr;
    GdiObject source(CreateDIBSection(nullptr,&bitmap,DIB_RGB_COLORS,&raw_pixels,nullptr,0));
    if (!memory.valid() || !source.valid() || raw_pixels==nullptr) return 1;
    Selection selected(memory.get(),source.get());
    if (!selected.valid()) return 1;
    // DIB owns a checked 128*64*4-byte allocation for this entire window lifetime.
    auto* dib = static_cast<uint32_t*>(raw_pixels);
    for (unsigned y=0; y<64; ++y) {
        for (unsigned x=0; x<128; ++x) dib[y*128+x] = x<64 ? 0x00ff0000U : 0x0000ff00U;
    }
    Scene scene{memory.get(),true};
    WNDCLASSW window_type{};
    window_type.lpfnWndProc = window_proc;
    window_type.hInstance = GetModuleHandleW(nullptr);
    window_type.hCursor = LoadCursorW(nullptr,IDC_ARROW);
    window_type.lpszClassName = window_class;
    if (RegisterClassW(&window_type)==0) return 1;

    int result = 1;
    {
        Window window(CreateWindowExW(0,window_class,L"Finite framebuffer acceptance",
            WS_OVERLAPPEDWINDOW,80,80,760,520,nullptr,nullptr,window_type.hInstance,&scene));
        if (window.valid()) {
            ShowWindow(window.get(),SW_SHOWNORMAL);
            UpdateWindow(window.get());
            pump();
            WindowDC dc(window.get());
            if (dc.get()!=nullptr) {
                if (require_lightning) cursor_cycle(window.get(),scene,checks,log,cycle_modes,cursor_probe,cursor_fault_probe);
                checks.test(draw_scene(dc.get(),scene,0),"initial-real-GDI-drawing");
                rx7900::reactos::framebuffer::ShadowStatistics before{};
                const bool proof_before=shadow_statistics(dc.get(),log,before);
                if (require_shadow) checks.test(proof_before && before.active==1,"shadow-active-owned-surface");
                lightning::reactos::gdi::Statistics gpu_before{};
                if (require_gpu) checks.test(gpu_statistics(dc.get(),gpu_before) && gpu_before.enabled==1,
                    "gpu-hook-statistics-active");
                pixels(dc.get(),checks);
                LARGE_INTEGER transition_start{};
                LARGE_INTEGER transition_end{};
                bool clock_ok = QueryPerformanceCounter(&transition_start) != FALSE;
                bool moved = true;
                for (int step=0; step<8; ++step) {
                    moved = SetWindowPos(window.get(),HWND_TOP,80+step*32,80+step*16,760,520,
                                         SWP_NOACTIVATE) != FALSE && moved;
                    UpdateWindow(window.get());
                    pump();
                }
                ShowWindow(window.get(),SW_MINIMIZE);
                pump();
                checks.test(IsIconic(window.get())!=FALSE,"minimize");
                ShowWindow(window.get(),SW_RESTORE);
                pump();
                ShowWindow(window.get(),SW_MAXIMIZE);
                pump();
                checks.test(IsZoomed(window.get())!=FALSE,"maximize");
                ShowWindow(window.get(),SW_RESTORE);
                UpdateWindow(window.get());
                pump();
                GdiFlush();
                clock_ok = QueryPerformanceCounter(&transition_end) != FALSE && clock_ok;
                clock_ok = transition_end.QuadPart >= transition_start.QuadPart && clock_ok;
                checks.test(moved && !IsIconic(window.get()) && !IsZoomed(window.get()),"move-restore");

                double milliseconds[frame_count]{};
                double total = 0;
                bool rendered = true;
                for (unsigned frame=0; frame<frame_count; ++frame) {
                    LARGE_INTEGER begin{};
                    LARGE_INTEGER end{};
                    clock_ok = QueryPerformanceCounter(&begin) != FALSE && clock_ok;
                    rendered = draw_scene(dc.get(),scene,frame) && rendered;
                    clock_ok = QueryPerformanceCounter(&end) != FALSE && clock_ok;
                    clock_ok = end.QuadPart >= begin.QuadPart && clock_ok;
                    milliseconds[frame] = 1000.0*static_cast<double>(end.QuadPart-begin.QuadPart)/
                                          static_cast<double>(frequency.QuadPart);
                    total += milliseconds[frame];
                    pump();
                }
                checks.test(rendered && scene.painted,"all-timed-GDI-operations");
                checks.test(clock_ok,"monotonic-QPC-samples");
                pixels(dc.get(),checks);
                for (unsigned i=1; i<frame_count; ++i) {
                    const double value = milliseconds[i];
                    unsigned at = i;
                    while (at!=0 && milliseconds[at-1]>value) {
                        milliseconds[at]=milliseconds[at-1];
                        --at;
                    }
                    milliseconds[at]=value;
                }
                const double transitions = 1000.0*static_cast<double>(transition_end.QuadPart-transition_start.QuadPart)/
                                           static_cast<double>(frequency.QuadPart);
                StringCchPrintfA(message,sizeof(message),
                    "TIMING real-GDI-QPC mean_ms=%.3f p50_ms=%.3f p95_ms=%.3f max_ms=%.3f transitions_ms=%.3f\r\n",
                    total/frame_count,milliseconds[frame_count/2],milliseconds[(frame_count*95)/100],
                    milliseconds[frame_count-1],transitions);
                log.line(message);
                profile_operations(dc.get(),scene,frequency.QuadPart,log,checks);
                if (require_gpu) {
                    complex_overlap_pixels(dc.get(),checks);
                    text_batch_pixels(dc.get(),checks);
                    raster_pixels(dc.get(),scene,checks);
                    masked_pattern_pixels(dc.get(),scene,checks);
                    stretch_pixels(dc.get(),checks);
                    extended_raster_pixels(dc.get(),checks);
                    gradient_pixels(dc.get(),checks);
                    paint_pixels(dc.get(),checks);
                }
                checks.test(draw_scene(dc.get(),scene,0),"profile-restore-pixels");
                pixels(dc.get(),checks);
                rx7900::reactos::framebuffer::ShadowStatistics after{};
                const bool proof_after=shadow_statistics(dc.get(),log,after);
                if (require_shadow) {
                    checks.test(proof_after && after.active==1 && after.flush_calls>before.flush_calls &&
                        after.flush_bytes>before.flush_bytes &&
                        (after.bitblt_calls>before.bitblt_calls || after.copy_calls>before.copy_calls),
                        "shadow-real-draw-and-flush-counter-growth");
                    checks.test(proof_after && after.failed_calls==before.failed_calls,"shadow-no-operation-failures");
                }
                if (require_gpu) gpu_progress(dc.get(),checks,log,gpu_before);
                else if (require_lightning) lightning_publication(dc.get(),checks,log);
                StringCchPrintfA(message,sizeof(message),"FRAMEBUFFER_SMOKE_%s checks=%u failures=%u\r\n",
                    checks.failures==0?"PASS":"FAIL",checks.checks,checks.failures);
                log.line(message);
                result = checks.failures==0 ? 0 : 1;
            }
        }
    }
    UnregisterClassW(window_class,window_type.hInstance);
    return result;
}
} // namespace

int wmain(int argc, wchar_t** argv) {
    const wchar_t* log_path = nullptr;
    bool use_serial = false;
    bool require_shadow = false;
    bool cycle_modes = false;
    bool require_lightning = false;
    bool require_gpu = false;
    bool cursor_probe = false;
    bool cursor_fault_probe = false;
    for (int i=1; i<argc; ++i) {
        if (lstrcmpW(argv[i],L"--log")==0 && i+1<argc && log_path==nullptr) log_path=argv[++i];
        else if (lstrcmpW(argv[i],L"--serial")==0 && !use_serial) use_serial=true;
        else if (lstrcmpW(argv[i],L"--require-shadow")==0 && !require_shadow) require_shadow=true;
        else if (lstrcmpW(argv[i],L"--cycle-modes")==0 && !cycle_modes) cycle_modes=true;
        else if (lstrcmpW(argv[i],L"--require-lightning")==0 && !require_lightning) require_lightning=true;
        else if (lstrcmpW(argv[i],L"--require-gpu")==0 && !require_gpu) require_gpu=true;
        else if (lstrcmpW(argv[i],L"--cursor-probe")==0 && !cursor_probe) cursor_probe=true;
        else if (lstrcmpW(argv[i],L"--cursor-fault-probe")==0 && !cursor_fault_probe) cursor_fault_probe=true;
        else {
            fputs("usage: framebuffer_guest_smoke [--log FILE] [--serial] [--require-shadow] [--cycle-modes] [--require-lightning] [--require-gpu] [--cursor-probe] [--cursor-fault-probe]\n",stderr);
            return 2;
        }
    }
    rx7900::DeviceHandle file(log_path==nullptr ? INVALID_HANDLE_VALUE :
        CreateFileW(log_path,GENERIC_WRITE,FILE_SHARE_READ,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr),
        INVALID_HANDLE_VALUE);
    rx7900::DeviceHandle serial(use_serial ? CreateFileW(L"\\\\.\\COM2",GENERIC_WRITE,0,nullptr,
        OPEN_EXISTING,0,nullptr) : INVALID_HANDLE_VALUE,INVALID_HANDLE_VALUE);
    const Log log(file.get(),serial.get());
    if ((log_path!=nullptr && !file.valid()) || (use_serial && !serial.valid())) {
        const DWORD error=GetLastError();
        char message[256]{};
        StringCchPrintfA(message,sizeof(message),
            "FRAMEBUFFER_SMOKE_FAIL could not open explicitly requested log sink: %lu\r\n",error);
        log.line(message);
        if (file.valid()) FlushFileBuffers(file.get());
        return 1;
    }
    if (serial.valid()) {
        COMMTIMEOUTS timeouts{};
        timeouts.WriteTotalTimeoutConstant = 2000;
        SetCommTimeouts(serial.get(),&timeouts);
    }
    const int result = exercise(log,require_shadow,cycle_modes,require_lightning || cursor_probe || cursor_fault_probe,require_gpu,cursor_probe || cursor_fault_probe,cursor_fault_probe);
    if (file.valid()) FlushFileBuffers(file.get());
    return result;
}
