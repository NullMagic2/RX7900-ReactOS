/*
Module: rx7900.tests.opengl_icd
Purpose: Exercise the ReactOS ICD loader contract against the radeonglicd DLL:
version validation, pixel-format description, context create/make-current/delete,
the 336-entry dispatch table, rendering through it and buffer presentation.
Created: 2026-10-23
Architecture: Loads the built ICD DLL exactly as opengl32 icdload.c does
(LoadLibrary + GetProcAddress of the Drv* exports), installs the dispatch table
from DrvSetContext and renders through it, then reads back pixels with the
dispatched glReadPixels. Runs under Wine with GCC13 MinGW. It also checks the
driver rejects an unknown pixel format and a null context, as a submission-safety
guard on the loader boundary.
*/
#include <windows.h>
#include <GL/gl.h>

#ifndef GLAPIENTRY
#define GLAPIENTRY APIENTRY
#endif

#include <cstdio>

#include "../src/opengl/icd/icd_table.hpp"

// GetProcAddress returns FARPROC; resolving a typed entry point is the documented
// idiom and the only way to call the loaded ICD, so the generic-function-cast
// diagnostic does not apply here.
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

using PFN_Validate = BOOL(WINAPI*)(DWORD);
using PFN_Describe = int(WINAPI*)(HDC, int, UINT, LPPIXELFORMATDESCRIPTOR);
using PFN_SetFormat = BOOL(WINAPI*)(HDC, int);
using PFN_Create = DHGLRC(WINAPI*)(HDC);
using PFN_Delete = BOOL(WINAPI*)(DHGLRC);
using PFN_SetCtx = const GLCLTPROCTABLE*(WINAPI*)(HDC, DHGLRC, PFN_SETPROCTABLE);
using PFN_Release = void(WINAPI*)(DHGLRC);
using PFN_Swap = BOOL(WINAPI*)(HDC);

const GLCLTPROCTABLE* g_table = nullptr;
void APIENTRY set_table(const GLCLTPROCTABLE* table) { g_table = table; }

} // namespace

int main() {
    HMODULE module = LoadLibraryW(L"radeonglicd.dll");
    if (module == nullptr) {
        std::fprintf(stderr, "FAIL: could not load radeonglicd.dll (%lu)\n", GetLastError());
        return 1;
    }

    auto validate = reinterpret_cast<PFN_Validate>(GetProcAddress(module, "DrvValidateVersion"));
    auto describe = reinterpret_cast<PFN_Describe>(GetProcAddress(module, "DrvDescribePixelFormat"));
    auto set_format = reinterpret_cast<PFN_SetFormat>(GetProcAddress(module, "DrvSetPixelFormat"));
    auto create = reinterpret_cast<PFN_Create>(GetProcAddress(module, "DrvCreateContext"));
    auto del = reinterpret_cast<PFN_Delete>(GetProcAddress(module, "DrvDeleteContext"));
    auto set_ctx = reinterpret_cast<PFN_SetCtx>(GetProcAddress(module, "DrvSetContext"));
    auto release = reinterpret_cast<PFN_Release>(GetProcAddress(module, "DrvReleaseContext"));
    auto swap = reinterpret_cast<PFN_Swap>(GetProcAddress(module, "DrvSwapBuffers"));

    expect(validate && describe && set_format && create && del && set_ctx && release && swap,
           "all Drv entry points are exported");
    if (!(validate && describe && set_format && create && del && set_ctx && release && swap)) {
        return 1;
    }

    expect(validate(1) == TRUE, "DrvValidateVersion accepts the registry version");

    // Offscreen window for a real client rect under Wine.
    WNDCLASSW wc{};
    wc.lpfnWndProc = DefWindowProcW;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.lpszClassName = L"rx7900icdtest";
    RegisterClassW(&wc);
    HWND window = CreateWindowExW(0, L"rx7900icdtest", L"icd", WS_OVERLAPPEDWINDOW, 0, 0, 96, 96,
                                  nullptr, nullptr, wc.hInstance, nullptr);
    expect(window != nullptr, "test window created");
    HDC hdc = GetDC(window);

    PIXELFORMATDESCRIPTOR pfd{};
    const int count = describe(hdc, 1, sizeof(pfd), &pfd);
    expect(count >= 1, "DrvDescribePixelFormat reports a format");
    expect((pfd.dwFlags & PFD_SUPPORT_OPENGL) != 0 && (pfd.dwFlags & PFD_DOUBLEBUFFER) != 0,
           "format supports OpenGL and double buffering");
    expect(pfd.cColorBits == 32 && pfd.cDepthBits == 24 && pfd.cStencilBits == 8,
           "format advertises 32-bit color, 24-bit depth, 8-bit stencil");
    expect(describe(hdc, 2, sizeof(pfd), &pfd) == 0, "unknown pixel format index is rejected");
    expect(set_format(hdc, 2) == FALSE, "DrvSetPixelFormat rejects an unknown format");
    expect(set_format(hdc, 1) == TRUE, "DrvSetPixelFormat accepts format 1");

    expect(set_ctx(hdc, nullptr, set_table) == nullptr, "DrvSetContext rejects a null context");

    DHGLRC rc = create(hdc);
    expect(rc != nullptr, "DrvCreateContext returns a context");

    const GLCLTPROCTABLE* table = set_ctx(hdc, rc, set_table);
    expect(table != nullptr, "DrvSetContext returns the dispatch table");
    expect(g_table == table, "DrvSetContext invoked the proc-table callback");
    if (table == nullptr) {
        return 1;
    }
    expect(table->cEntries == OPENGL_VERSION_110_ENTRIES, "dispatch table has 336 entries");

    const GLDISPATCHTABLE& gl = table->glDispatchTable;
    const GLubyte* version = gl.GetString(GL_VERSION);
    expect(version != nullptr && version[0] == '1', "glGetString(GL_VERSION) through the ICD");

    // Render a flat red triangle covering the center, then read it back.
    gl.Viewport(0, 0, 96, 96);
    gl.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    gl.Clear(GL_COLOR_BUFFER_BIT);
    gl.ShadeModel(GL_FLAT);
    gl.Color3f(1.0f, 0.0f, 0.0f);
    gl.Begin(GL_TRIANGLES);
    gl.Vertex2f(-0.9f, -0.9f);
    gl.Vertex2f(0.9f, -0.9f);
    gl.Vertex2f(0.0f, 0.9f);
    gl.End();

    unsigned char center[4] = {0, 0, 0, 0};
    gl.ReadPixels(48, 40, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, center);
    expect(center[0] > 200 && center[1] < 40 && center[2] < 40,
           "glReadPixels returns the rendered red fragment through the ICD");
    expect(gl.GetError() == GL_NO_ERROR, "no GL error after the ICD render");

    expect(swap(hdc) == TRUE, "DrvSwapBuffers presents the back buffer");

    release(rc);
    expect(del(rc) == TRUE, "DrvDeleteContext frees the context");

    ReleaseDC(window, hdc);
    DestroyWindow(window);
    FreeLibrary(module);

    std::printf("opengl_icd: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
