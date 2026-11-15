/*
Module: rx7900.opengl.icd
Purpose: ReactOS WGL installable client driver (ICD) entry points for the
physical Radeon OpenGL stack: the GFX11 hardware backend when the native process
service answers, with the GDI-presented software renderer as the reported
fallback.
Created: 2026-10-23
Architecture: opengl32 (dll/opengl/opengl32 icdload.c/wgl.c, pinned a97c8a2)
loads this DLL from the OpenGLDrivers key, validates the version, reads the Drv*
exports and installs the dispatch table from DrvSetContext. Each context owns a
Context (gl_context). On the first make-current the ICD opens a process-service
session through its transport (icd_transport: ExtEscape/DrawEscape through
radeondisp) and, when that succeeds, renders with Gfx11Backend; otherwise, or
when RADEONGL_SOFTWARE=1, with SoftwareBackend (gl_raster). The GPU backend keeps
the software backend as its fallback for inexpressible state and reports why
(OutputDebugString and RadeonGlRendererInfo). DrvSwapBuffers presents the GPU
back buffer with a kernel blit clipped by win32k's CLIPOBJ (DrawEscape), else by
an exact GPU readback into a DIB drawn with SetDIBitsToDevice, else the software
buffer. One context is current per thread (the opengl32 contract). Builds for
i386 and AMD64 with GCC13 MinGW. No hardware acceleration is claimed without
runtime evidence on the physical GPU.
*/
#include "radeon_opengl_info.hpp"
#include <windows.h>
#include <GL/gl.h>

#include <new>

#include "icd_table.hpp"
#include "../gl_context.hpp"
#include "../gl_raster.hpp"
#include "../gl_gfx11_backend.hpp"

using namespace rx7900::opengl;

rx7900::opengl::GpuTransport* rx7900_icd_create_transport(HDC dc) noexcept;
void rx7900_icd_bind_transport(rx7900::opengl::GpuTransport* transport, HDC dc) noexcept;
void rx7900_icd_destroy_transport(rx7900::opengl::GpuTransport* transport) noexcept;

namespace {

// One ICD context: its drawable framebuffer and the GL state machine over it.
struct IcdContext final {
    HDC hdc = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint8_t* storage = nullptr;
    uint64_t storage_bytes = 0;
    SoftwareBackend* backend = nullptr;
    Context* context = nullptr;
    int pixel_format = 0;
    // GPU path (null when unavailable).
    GpuTransport* transport = nullptr;
    Gfx11Backend* gpu = nullptr;
    uint8_t* gpu_scratch = nullptr;
    uint32_t* dib = nullptr;
    uint64_t dib_bytes = 0;
    bool gpu_tried = false;
};

thread_local IcdContext* t_current = nullptr;

void* heap_alloc(uint64_t bytes) noexcept {
    return bytes > SIZE_MAX ? nullptr : HeapAlloc(GetProcessHeap(), 0, static_cast<SIZE_T>(bytes));
}

void heap_free(void* p) noexcept {
    if (p != nullptr) HeapFree(GetProcessHeap(), 0, p);
}

bool software_forced() noexcept {
    char value[8] = {};
    return GetEnvironmentVariableA("RADEONGL_SOFTWARE", value, sizeof(value)) > 0 && value[0] == '1';
}

void report(const IcdContext* ctx) noexcept {
    char line[160];
    const bool gpu = ctx->gpu != nullptr && ctx->gpu->gpu_active();
    wsprintfA(line, "radeonglicd: renderer=%s reason=%s\n", gpu ? "GFX11" : "software",
              ctx->gpu != nullptr ? fallback_reason_text(ctx->gpu->reason()) :
              software_forced() ? "RADEONGL_SOFTWARE=1" : "native GPU process service unavailable");
    OutputDebugStringA(line);
}

// First make-current: try the GPU path over this DC.
void try_gpu(IcdContext* ctx, HDC hdc) noexcept {
    ctx->gpu_tried = true;
    if (software_forced()) {
        report(ctx);
        return;
    }
    ctx->transport = rx7900_icd_create_transport(hdc);
    ctx->gpu_scratch = static_cast<uint8_t*>(heap_alloc(gfx11_backend_scratch_bytes));
    void* raw = heap_alloc(sizeof(Gfx11Backend));
    if (ctx->transport == nullptr || ctx->gpu_scratch == nullptr || raw == nullptr) {
        heap_free(raw);
        report(ctx);
        return;
    }
    ctx->gpu = new (raw) Gfx11Backend(*ctx->transport, ctx->gpu_scratch, gfx11_backend_scratch_bytes,
                                      *ctx->backend);
    if (!ctx->gpu->initialize()) {
        // Keep the object only for its reason; render in software.
        report(ctx);
        return;
    }
    ctx->context->set_backend(ctx->gpu);
    report(ctx);
}

// The one pixel format the software fallback advertises: 32-bit BGRA, double
// buffered, 24-bit depth, 8-bit stencil, drawn to a window.
void fill_pixel_format(PIXELFORMATDESCRIPTOR* pfd) noexcept {
    ZeroMemory(pfd, sizeof(*pfd));
    pfd->nSize = sizeof(PIXELFORMATDESCRIPTOR);
    pfd->nVersion = 1;
    pfd->dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER |
                   PFD_GENERIC_ACCELERATED;
    pfd->iPixelType = PFD_TYPE_RGBA;
    pfd->cColorBits = 32;
    pfd->cRedBits = 8; pfd->cRedShift = 16;
    pfd->cGreenBits = 8; pfd->cGreenShift = 8;
    pfd->cBlueBits = 8; pfd->cBlueShift = 0;
    pfd->cAlphaBits = 8; pfd->cAlphaShift = 24;
    pfd->cDepthBits = 24;
    pfd->cStencilBits = 8;
    pfd->iLayerType = PFD_MAIN_PLANE;
}

bool drawable_size(HDC hdc, uint32_t& width, uint32_t& height) noexcept {
    HWND window = WindowFromDC(hdc);
    RECT rect{};
    if (window != nullptr && GetClientRect(window, &rect)) {
        width = static_cast<uint32_t>(rect.right - rect.left);
        height = static_cast<uint32_t>(rect.bottom - rect.top);
    } else {
        const int cx = GetDeviceCaps(hdc, HORZRES);
        const int cy = GetDeviceCaps(hdc, VERTRES);
        width = cx > 0 ? static_cast<uint32_t>(cx) : 0;
        height = cy > 0 ? static_cast<uint32_t>(cy) : 0;
    }
    if (width == 0) { width = 1; }
    if (height == 0) { height = 1; }
    if (width > raster_max_dimension) { width = raster_max_dimension; }
    if (height > raster_max_dimension) { height = raster_max_dimension; }
    return true;
}

// (Re)allocate the framebuffer for the current drawable size.
bool resize_drawable(IcdContext* ctx, HDC hdc) noexcept {
    uint32_t width = 0;
    uint32_t height = 0;
    drawable_size(hdc, width, height);
    if (ctx->backend != nullptr && ctx->width == width && ctx->height == height) {
        return true;
    }

    const uint64_t needed = raster_storage_bytes(width, height, true);
    if (needed == 0) {
        return false;
    }
    if (needed > ctx->storage_bytes) {
        uint8_t* fresh = static_cast<uint8_t*>(HeapAlloc(GetProcessHeap(), 0, needed));
        if (fresh == nullptr) {
            return false;
        }
        if (ctx->storage != nullptr) {
            HeapFree(GetProcessHeap(), 0, ctx->storage);
        }
        ctx->storage = fresh;
        ctx->storage_bytes = needed;
    }

    if (ctx->backend == nullptr) {
        void* raw = HeapAlloc(GetProcessHeap(), 0, sizeof(SoftwareBackend));
        if (raw == nullptr) {
            return false;
        }
        ctx->backend = new (raw) SoftwareBackend(ctx->storage, ctx->storage_bytes,
                                                 FramebufferFormat{true, 24, 8});
        ctx->context->set_backend(ctx->backend);
    } else {
        // The storage may have moved; the software backend re-binds on resize.
        ctx->backend->~SoftwareBackend();
        new (ctx->backend) SoftwareBackend(ctx->storage, ctx->storage_bytes, FramebufferFormat{true, 24, 8});
    }
    if (!ctx->gpu_tried) {
        try_gpu(ctx, hdc);
    }

    // The GPU backend resizes its own buffers and its software fallback.
    Backend* active = ctx->gpu != nullptr && ctx->context->backend() == ctx->gpu ?
        static_cast<Backend*>(ctx->gpu) : static_cast<Backend*>(ctx->backend);
    if (!active->resize(width, height)) {
        return false;
    }
    ctx->width = width;
    ctx->height = height;
    return true;
}

void present_dib(const IcdContext* ctx, HDC hdc, const uint32_t* pixels) noexcept {
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biWidth = static_cast<LONG>(ctx->width);
    // Positive height: bottom-up DIB, matching OpenGL's bottom-to-top rows.
    info.bmiHeader.biHeight = static_cast<LONG>(ctx->height);
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biCompression = BI_RGB;
    SetDIBitsToDevice(hdc, 0, 0, ctx->width, ctx->height, 0, 0, 0, ctx->height, pixels, &info,
                      DIB_RGB_COLORS);
}

void present(IcdContext* ctx, HDC hdc) noexcept {
    if (ctx->backend == nullptr || ctx->width == 0 || ctx->height == 0) {
        return;
    }
    if (ctx->gpu != nullptr && ctx->context->backend() == ctx->gpu) {
        const uint64_t bytes = static_cast<uint64_t>(ctx->width) * ctx->height * 4u;
        if (bytes > ctx->dib_bytes) {
            heap_free(ctx->dib);
            ctx->dib = static_cast<uint32_t*>(heap_alloc(bytes));
            ctx->dib_bytes = ctx->dib != nullptr ? bytes : 0;
        }
        rx7900_icd_bind_transport(ctx->transport, hdc);
        const PresentPath path = ctx->gpu->present(ctx->dib);
        if (path == PresentPath::gpu) {
            return;          // kernel blit clipped by the DC's CLIPOBJ
        }
        if (path == PresentPath::readback) {
            present_dib(ctx, hdc, ctx->dib);
            return;
        }
        if (path == PresentPath::failed) {
            report(ctx);
            return;
        }
        // PresentPath::software: the backend fell back; present its buffer.
    }
    const uint32_t* pixels = ctx->backend->color_buffer(gl::back);
    if (pixels == nullptr) {
        return;
    }
    present_dib(ctx, hdc, pixels);
    ctx->backend->swap();
}

} // namespace

namespace rx7900 { namespace opengl {
Context* icd_current() noexcept {
    return t_current != nullptr ? t_current->context : nullptr;
}
} }

extern "C" {

BOOL WINAPI DrvValidateVersion(DWORD version) {
    // opengl32 passes the DriverVersion the display driver reported through
    // OPENGL_GETINFO (or the custom-ICD registry value); accept only ours.
    return version == rx7900::reactos::radeon_opengl::driver_version ? TRUE : FALSE;
}

int WINAPI DrvDescribePixelFormat(HDC hdc, int index, UINT bytes, LPPIXELFORMATDESCRIPTOR pfd) {
    (void)hdc;
    const int count = 1;
    if (pfd == nullptr || bytes == 0) {
        return count;
    }
    if (index != 1) {
        return 0;
    }
    PIXELFORMATDESCRIPTOR local{};
    fill_pixel_format(&local);
    CopyMemory(pfd, &local, bytes < sizeof(local) ? bytes : sizeof(local));
    return count;
}

BOOL WINAPI DrvSetPixelFormat(HDC hdc, int index) {
    (void)hdc;
    return index == 1 ? TRUE : FALSE;
}

DHGLRC WINAPI DrvCreateContext(HDC hdc) {
    void* raw = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, sizeof(IcdContext));
    if (raw == nullptr) {
        return nullptr;
    }
    IcdContext* ctx = new (raw) IcdContext();
    ctx->hdc = hdc;
    ctx->pixel_format = 1;
    void* ctx_raw = HeapAlloc(GetProcessHeap(), 0, sizeof(Context));
    if (ctx_raw == nullptr) {
        HeapFree(GetProcessHeap(), 0, ctx);
        return nullptr;
    }
    ctx->context = new (ctx_raw) Context(nullptr, default_allocator());
    return reinterpret_cast<DHGLRC>(ctx);
}

DHGLRC WINAPI DrvCreateLayerContext(HDC hdc, int layer) {
    return layer == 0 ? DrvCreateContext(hdc) : nullptr;
}

BOOL WINAPI DrvDeleteContext(DHGLRC handle) {
    IcdContext* ctx = reinterpret_cast<IcdContext*>(handle);
    if (ctx == nullptr) {
        return FALSE;
    }
    if (t_current == ctx) {
        t_current = nullptr;
    }
    if (ctx->context != nullptr) {
        ctx->context->~Context();
        HeapFree(GetProcessHeap(), 0, ctx->context);
    }
    if (ctx->gpu != nullptr) {
        // Waits for its last fence, releases its buffers and closes the session.
        rx7900_icd_bind_transport(ctx->transport, ctx->hdc);
        ctx->gpu->~Gfx11Backend();
        heap_free(ctx->gpu);
    }
    rx7900_icd_destroy_transport(ctx->transport);
    heap_free(ctx->gpu_scratch);
    heap_free(ctx->dib);
    if (ctx->backend != nullptr) {
        ctx->backend->~SoftwareBackend();
        HeapFree(GetProcessHeap(), 0, ctx->backend);
    }
    if (ctx->storage != nullptr) {
        HeapFree(GetProcessHeap(), 0, ctx->storage);
    }
    HeapFree(GetProcessHeap(), 0, ctx);
    return TRUE;
}

const GLCLTPROCTABLE* WINAPI DrvSetContext(HDC hdc, DHGLRC handle, PFN_SETPROCTABLE callback) {
    IcdContext* ctx = reinterpret_cast<IcdContext*>(handle);
    if (ctx == nullptr) {
        return nullptr;
    }
    ctx->hdc = hdc;
    rx7900_icd_bind_transport(ctx->transport, hdc);
    if (!resize_drawable(ctx, hdc)) {
        return nullptr;
    }
    t_current = ctx;
    if (callback != nullptr) {
        callback(&rx7900_icd_proc_table);
    }
    return &rx7900_icd_proc_table;
}

void WINAPI DrvReleaseContext(DHGLRC handle) {
    if (reinterpret_cast<IcdContext*>(handle) == t_current) {
        t_current = nullptr;
    }
}

BOOL WINAPI DrvSwapBuffers(HDC hdc) {
    IcdContext* ctx = t_current;
    if (ctx == nullptr) {
        return FALSE;
    }
    // Repaint to the current window size, then present and cycle buffers.
    resize_drawable(ctx, hdc);
    present(ctx, hdc);
    return TRUE;
}

BOOL WINAPI DrvCopyContext(DHGLRC source, DHGLRC dest, UINT mask) {
    (void)source;
    (void)dest;
    (void)mask;
    return FALSE;   // GL state copy is not supported; apps rarely use it.
}

BOOL WINAPI DrvShareLists(DHGLRC source, DHGLRC dest) {
    IcdContext* a = reinterpret_cast<IcdContext*>(source);
    IcdContext* b = reinterpret_cast<IcdContext*>(dest);
    if (a == nullptr || b == nullptr || a->context == nullptr || b->context == nullptr) {
        return FALSE;
    }
    return b->context->adopt_share_group(*a->context) ? TRUE : FALSE;
}

PROC WINAPI DrvGetProcAddress(LPCSTR name) {
    (void)name;
    // Only GL 1.1 core (served through the dispatch table) is implemented; no
    // extension entry points are exported yet.
    return nullptr;
}

BOOL WINAPI DrvDescribeLayerPlane(HDC hdc, int format, int layer, UINT bytes,
                                  LPLAYERPLANEDESCRIPTOR plpd) {
    (void)hdc; (void)format; (void)layer; (void)bytes; (void)plpd;
    return FALSE;
}

int WINAPI DrvGetLayerPaletteEntries(HDC hdc, int layer, int start, int entries, COLORREF* colors) {
    (void)hdc; (void)layer; (void)start; (void)entries; (void)colors;
    return 0;
}

int WINAPI DrvSetLayerPaletteEntries(HDC hdc, int layer, int start, int entries,
                                     const COLORREF* colors) {
    (void)hdc; (void)layer; (void)start; (void)entries; (void)colors;
    return 0;
}

BOOL WINAPI DrvRealizeLayerPalette(HDC hdc, int layer, BOOL realize) {
    (void)hdc; (void)layer; (void)realize;
    return FALSE;
}

BOOL WINAPI DrvSwapLayerBuffers(HDC hdc, UINT planes) {
    (void)hdc; (void)planes;
    return FALSE;
}

void WINAPI DrvSetCallbackProcs(int count, PROC* procs) {
    // opengl32 offers wglSetCurrentValue/wglGetCurrentValue/wglGetDHGLRC; the
    // software fallback keeps its own thread-current pointer and needs none.
    (void)count;
    (void)procs;
}

// Diagnostic export (not part of the opengl32 contract): 1 when the GFX11 path
// renders, 0 for software; reason receives the fallback reason text.
int WINAPI RadeonGlRendererInfo(DHGLRC handle, char* reason, int bytes) {
    const IcdContext* ctx = reinterpret_cast<const IcdContext*>(handle);
    if (ctx == nullptr) {
        return -1;
    }
    const bool gpu = ctx->gpu != nullptr && ctx->context->backend() == ctx->gpu && ctx->gpu->gpu_active();
    const char* text = ctx->gpu != nullptr ? fallback_reason_text(ctx->gpu->reason()) :
                       "native GPU process service unavailable";
    if (reason != nullptr && bytes > 0) {
        lstrcpynA(reason, text, bytes);
    }
    return gpu ? 1 : 0;
}

} // extern "C"
