/*
Module: rx7900.tests.opengl_icd_gpu
Purpose: ICD-level test of the GFX11 path under Wine: the radeonglicd_fake DLL
(the shipped ICD sources with the transport replaced by a fake device that runs
the production kernel graphics modules and the pipeline model) is loaded the way
opengl32 loads an ICD, renders through the dispatch table on the GPU backend,
reads pixels back, presents through the GPU, and honours RADEONGL_SOFTWARE=1.
Created: 2026-10-23
Architecture: GCC13 MinGW executable run under Wine. Not hardware evidence.
*/
#include <windows.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstring>

#include "../src/opengl/icd/icd_table.hpp"

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

namespace {

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

using PFN_SetFormat = BOOL(WINAPI*)(HDC, int);
using PFN_Create = DHGLRC(WINAPI*)(HDC);
using PFN_Delete = BOOL(WINAPI*)(DHGLRC);
using PFN_SetCtx = const GLCLTPROCTABLE*(WINAPI*)(HDC, DHGLRC, PFN_SETPROCTABLE);
using PFN_Release = void(WINAPI*)(DHGLRC);
using PFN_Swap = BOOL(WINAPI*)(HDC);
using PFN_Info = int(WINAPI*)(DHGLRC, char*, int);
using PFN_Stats = int(WINAPI*)(unsigned*, int);

void APIENTRY set_table(const GLCLTPROCTABLE*) {}

void draw(const GLDISPATCHTABLE& gl, int width, int height) {
    gl.Viewport(0, 0, width, height);
    gl.ClearColor(0.0f, 0.0f, 1.0f, 1.0f);
    gl.Clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    gl.Enable(GL_DEPTH_TEST);
    gl.Begin(GL_TRIANGLES);
    gl.Color3f(1.0f, 0.0f, 0.0f);
    gl.Vertex3f(-0.9f, -0.9f, 0.0f);
    gl.Vertex3f(0.9f, -0.9f, 0.0f);
    gl.Vertex3f(0.0f, 0.9f, 0.0f);
    gl.End();
}

} // namespace

int main() {
    HMODULE module = LoadLibraryW(L"radeonglicd_fake.dll");
    if (module == nullptr) {
        std::fprintf(stderr, "FAIL: could not load radeonglicd_fake.dll (%lu)\n", GetLastError());
        return 1;
    }
    auto set_format = reinterpret_cast<PFN_SetFormat>(GetProcAddress(module, "DrvSetPixelFormat"));
    auto create = reinterpret_cast<PFN_Create>(GetProcAddress(module, "DrvCreateContext"));
    auto del = reinterpret_cast<PFN_Delete>(GetProcAddress(module, "DrvDeleteContext"));
    auto set_ctx = reinterpret_cast<PFN_SetCtx>(GetProcAddress(module, "DrvSetContext"));
    auto release = reinterpret_cast<PFN_Release>(GetProcAddress(module, "DrvReleaseContext"));
    auto swap = reinterpret_cast<PFN_Swap>(GetProcAddress(module, "DrvSwapBuffers"));
    auto info = reinterpret_cast<PFN_Info>(GetProcAddress(module, "RadeonGlRendererInfo"));
    auto stats = reinterpret_cast<PFN_Stats>(GetProcAddress(module, "RadeonGlFakeDeviceStats"));
    expect(set_format && create && del && set_ctx && release && swap && info && stats, "exports present");
    if (!(set_format && create && del && set_ctx && release && swap && info && stats)) return 1;

    WNDCLASSW wc{};
    wc.lpfnWndProc = DefWindowProcW;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.lpszClassName = L"rx7900icdgpu";
    RegisterClassW(&wc);
    HWND window = CreateWindowExW(0, L"rx7900icdgpu", L"icd", WS_POPUP, 0, 0, 96, 80, nullptr, nullptr,
                                  wc.hInstance, nullptr);
    HDC hdc = GetDC(window);
    RECT client{};
    GetClientRect(window, &client);
    const int width = client.right - client.left;
    const int height = client.bottom - client.top;
    expect(set_format(hdc, 1) == TRUE, "pixel format 1");

    // 1) RADEONGL_SOFTWARE=1 forces the software renderer.
    SetEnvironmentVariableA("RADEONGL_SOFTWARE", "1");
    DHGLRC soft = create(hdc);
    const GLCLTPROCTABLE* table = set_ctx(hdc, soft, set_table);
    expect(table != nullptr, "software context current");
    char reason[128] = {};
    expect(info(soft, reason, sizeof(reason)) == 0, "RADEONGL_SOFTWARE=1 selects the software renderer");
    draw(table->glDispatchTable, width, height);
    unsigned char pixel[4] = {};
    table->glDispatchTable.ReadPixels(width / 2, height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    expect(pixel[0] > 200 && pixel[2] < 40, "software render is correct");
    release(soft);
    del(soft);
    SetEnvironmentVariableA("RADEONGL_SOFTWARE", nullptr);

    // 2) GPU path over the fake device.
    DHGLRC rc = create(hdc);
    table = set_ctx(hdc, rc, set_table);
    expect(table != nullptr, "GPU context current");
    const int gpu = info(rc, reason, sizeof(reason));
    std::printf("renderer=%d reason=%s client=%dx%d\n", gpu, reason, width, height);
    expect(gpu == 1, "the ICD selects the GFX11 backend when the process service answers");
    const GLDISPATCHTABLE& gl = table->glDispatchTable;
    draw(gl, width, height);
    std::memset(pixel, 0, sizeof(pixel));
    gl.ReadPixels(width / 2, height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    expect(pixel[0] > 200 && pixel[1] < 40 && pixel[2] < 40, "glReadPixels returns the GPU-rendered red fragment");
    unsigned char corner[4] = {};
    gl.ReadPixels(1, height - 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, corner);
    expect(corner[2] > 200 && corner[0] < 40, "glReadPixels returns the GPU clear colour outside the triangle");
    expect(gl.GetError() == GL_NO_ERROR, "no GL error on the GPU path");
    unsigned s[8] = {};
    expect(stats(s, 8) == 1 && s[1] > 0 && s[2] > 0 && s[3] >= 2 && s[4] == 0,
           "draws went through kernel-built IBs with no model or page faults");
    expect(swap(hdc) == TRUE, "DrvSwapBuffers presents");
    stats(s, 8);
    std::printf("fake device: calls=%u submits=%u ibs=%u draws=%u errors=%u presents=%u dma_rows=%u\n", s[0], s[1],
                s[2], s[3], s[4], s[5], s[6]);
    expect(s[5] >= 1 && s[6] > 0, "the back buffer reached the scanout through the GPU present path");
    expect(info(rc, reason, sizeof(reason)) == 1, "still on the GPU after present");
    release(rc);
    expect(del(rc) == TRUE, "GPU context deletes and closes its session");

    ReleaseDC(window, hdc);
    DestroyWindow(window);
    FreeLibrary(module);
    std::printf("opengl_icd_gpu: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
