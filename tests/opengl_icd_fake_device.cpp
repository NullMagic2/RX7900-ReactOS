/*
Module: rx7900.tests.opengl_icd_fake_device
Purpose: Transport factory for the radeonglicd_fake test DLL: instead of ExtEscape
and DrawEscape through radeondisp, the ICD talks to an in-process fake device
running the production kernel graphics modules over the synthetic MES fixture
(gfx11_graphics_device) with the pipeline model executing every IB.
Created: 2026-10-23
Architecture: Test-only replacement of src/opengl/icd/icd_transport.cpp, linked
into a separate DLL so the shipped radeonglicd keeps the real transport. The
exported RadeonGlFakeDeviceStats lets the Wine test inspect the fake device.
*/
#include <windows.h>

#include <memory>

#include "gfx11_graphics_device.hpp"

namespace {

gfx11_header_db::Database& database() {
    static gfx11_header_db::Database db;
    static bool loaded = db.load();
    (void)loaded;
    return db;
}

gfx11_graphics_device::Device* g_device = nullptr;

} // namespace

rx7900::opengl::GpuTransport* rx7900_icd_create_transport(HDC) noexcept {
    // One fake device per process; a 160x120 scanout for presents.
    if (g_device == nullptr) {
        g_device = new gfx11_graphics_device::Device(database(), 160, 120);
        g_device->window_left = 8;
        g_device->window_top = 6;
        g_device->clip = {{8, 6, 168, 126}};
        char value[8] = {};
        g_device->present_hook = !(GetEnvironmentVariableA("RADEONGL_FAKE_NO_PRESENT", value, sizeof(value)) > 0 &&
                                   value[0] == '1');
    }
    return g_device;
}

void rx7900_icd_bind_transport(rx7900::opengl::GpuTransport*, HDC) noexcept {}

void rx7900_icd_destroy_transport(rx7900::opengl::GpuTransport*) noexcept {}

extern "C" int WINAPI RadeonGlFakeDeviceStats(unsigned* stats, int count) {
    if (g_device == nullptr || stats == nullptr || count < 8) return 0;
    const auto& model = g_device->model->stats();
    stats[0] = g_device->calls;
    stats[1] = g_device->submits;
    stats[2] = g_device->ibs;
    stats[3] = model.draws;
    stats[4] = model.errors + g_device->ib_faults;
    stats[5] = g_device->presents;
    stats[6] = model.dma_rows;
    // Scanout pixel at the window centre (8 + 48, 6 + 40).
    stats[7] = g_device->scanout.empty() ? 0 : g_device->scanout[(6 + 40) * 160 + 8 + 48];
    return 1;
}
