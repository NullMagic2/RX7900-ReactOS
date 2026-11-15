/*
Module: lightning.tests.draw_escape
Purpose: Exercise production capture, window clipping and balanced native lifetimes.
Created: 2026-10-22
Architecture: Fake OS callbacks surround the unmodified C++ syscall implementation.
*/
extern "C" {
#include "draw_escape_native/win32k.h"
}
#include "../src/reactos/framebuffer_draw_escape.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr size_t maximum_events = 32;
constexpr INT private_escape = 0x4c474450;
constexpr ULONG driver_receipt = 173;
enum class Event { allocate, capture, user_enter, dc_lock, prepare, driver,
    finish, dc_unlock, user_leave, free };
struct Model final {
    std::array<Event, maximum_events> events{};
    size_t count = 0;
    bool allocations = true;
    bool capture = true;
    bool dc_valid = true;
    bool user_locked = false;
    bool dc_locked = false;
    bool prepared = false;
    bool invalid_driver_call = false;
    const void* user_input = nullptr;
    ULONG error = 0;
    NTSTATUS status = 0;
    SURFACE surface{{19}};
    PDEVOBJ device{};
    DC dc{};
    void event(Event value) {
        if (count >= events.size()) std::abort();
        events[count++] = value;
    }
} model;
unsigned checks = 0;
unsigned failures = 0;
void check(bool value, const char* name) {
    ++checks;

    if (!value) {
        ++failures;
        std::printf("FAIL: %s\n", name);
    }
}
ULONG callback(SURFOBJ* surface, ULONG escape, CLIPOBJ* clip,
    RECTL* window, ULONG bytes, void* input) {
    model.event(Event::driver);
    model.invalid_driver_call = !model.user_locked || !model.dc_locked || !model.prepared;
    check(surface == &model.surface.SurfObj, "locked surface passed");
    check(escape == static_cast<ULONG>(private_escape), "escape preserved");
    check(clip == static_cast<CLIPOBJ*>(&model.dc.co), "authoritative clip passed");
    check(window->left == 20 && window->top == 30 && window->right == 220 &&
        window->bottom == 130, "window rectangle preserved independently of clip");
    check(window != &model.dc.erclWindow, "driver cannot mutate DC window through callback");
    check(bytes == 4 && input != model.user_input && input != nullptr,
        "input privately captured");
    check(std::memcmp(input, model.user_input, bytes) == 0, "captured contents preserved");
    window->left = -999;
    return driver_receipt;
}
void reset() {
    model = {};
    model.device.DriverFunctions.DrawEscape = callback;
    model.dc.dclevel.pSurface = &model.surface;
    model.dc.ppdev = &model.device;
    model.dc.erclWindow = {20, 30, 220, 130};
    model.dc.ptlDCOrig = {20, 30};
    model.dc.co.rclBounds = {40, 35, 120, 100};
    model.dc.co.unique = 47;
}
bool balanced() {
    return !model.user_locked && !model.dc_locked && !model.prepared;
}
}

extern "C" PVOID ExAllocatePoolWithTag(ULONG, ULONG bytes, ULONG) {
    model.event(Event::allocate);
    return model.allocations ? std::malloc(bytes) : nullptr;
}
extern "C" VOID ExFreePoolWithTag(PVOID value, ULONG) {
    model.event(Event::free);
    std::free(value);
}
extern "C" NTSTATUS _MmCopyFromCaller(PVOID output, PVOID input, ULONG bytes) {
    model.event(Event::capture);

    if (!model.capture) return STATUS_ACCESS_VIOLATION;
    std::memcpy(output, input, bytes);
    return STATUS_SUCCESS;
}
extern "C" VOID EngSetLastError(ULONG value) { model.error = value; }
extern "C" VOID SetLastNtError(NTSTATUS value) { model.status = value; }
extern "C" VOID UserEnterExclusive() {
    model.event(Event::user_enter);
    check(!model.dc_locked, "USER acquired before DC");
    model.user_locked = true;
}
extern "C" VOID UserLeave() {
    model.event(Event::user_leave);
    check(!model.dc_locked && !model.prepared, "USER released last");
    model.user_locked = false;
}
extern "C" PDC DC_LockDc(HDC) {
    model.event(Event::dc_lock);

    if (!model.dc_valid) return nullptr;
    model.dc_locked = true;
    return &model.dc;
}
extern "C" VOID DC_UnlockDc(PDC) {
    model.event(Event::dc_unlock);
    check(!model.prepared, "DC released after device");
    model.dc_locked = false;
}
extern "C" VOID DC_vPrepareDCsForBlit(PDC, const RECTL*, PDC, const RECTL*) {
    model.event(Event::prepare);
    check(model.user_locked && model.dc_locked, "device acquired under USER and DC");
    model.prepared = true;
}
extern "C" VOID DC_vFinishBlit(PDC, PDC) {
    model.event(Event::finish);
    check(model.user_locked && model.dc_locked, "device released before DC and USER");
    model.prepared = false;
}

int main() {
    const uint32_t payload = 0x12345678;
    HDC handle = reinterpret_cast<HDC>(static_cast<uintptr_t>(1));
    reset();
    model.user_input = &payload;
    check(LightningDrawEscape(handle, private_escape, sizeof(payload), &payload) ==
        static_cast<INT>(driver_receipt), "actual driver receipt");
    check(!model.invalid_driver_call && balanced(), "success owns and balances every lock");
    check(model.dc.erclWindow.left == 20, "driver cannot rewrite original window");
    check(model.events[0] == Event::allocate && model.events[1] == Event::capture &&
        model.events[2] == Event::user_enter && model.events[model.count - 1] == Event::free,
        "capture before locks and free after unlock");

    reset();
    model.user_input = &payload;
    model.dc.erclWindow = {-80, -70, 120, 30};
    check(LightningDrawEscape(handle, private_escape, sizeof(payload), &payload) ==
        static_cast<INT>(driver_receipt) && balanced(), "moved retained DC uses current GDI origin");
    reset();
    model.user_input = &payload;
    model.dc.ptlDCOrig.x = INT32_MAX;
    check(LightningDrawEscape(handle, private_escape, sizeof(payload), &payload) == -1 &&
        model.error == ERROR_INVALID_PARAMETER && balanced(), "translated extent overflow rejected under balanced locks");

    for (INT bytes : {-1, 65537}) {
        reset();
        check(LightningDrawEscape(handle, private_escape, bytes, &payload) == -1 &&
            model.error == ERROR_INVALID_PARAMETER && model.count == 0,
            "invalid extent rejected before access");
    }
    reset();
    check(LightningDrawEscape(handle, private_escape, sizeof(payload), nullptr) == -1 &&
        model.count == 0, "null nonempty input rejected");
    reset();
    model.allocations = false;
    check(LightningDrawEscape(handle, private_escape, sizeof(payload), &payload) == -1 &&
        model.count == 1 && model.status == STATUS_INSUFFICIENT_RESOURCES,
        "allocation failure never acquires USER");
    reset();
    model.capture = false;
    check(LightningDrawEscape(handle, private_escape, sizeof(payload), &payload) == -1 &&
        model.count == 3 && model.events[2] == Event::free &&
        model.status == STATUS_ACCESS_VIOLATION, "failed capture frees before any lock");
    reset();
    model.dc_valid = false;
    check(LightningDrawEscape(handle, private_escape, 0, nullptr) == -1 &&
        model.error == ERROR_INVALID_HANDLE && balanced(), "invalid DC releases USER");

    for (unsigned unavailable = 0; unavailable < 5; ++unavailable) {
        reset();

        switch (unavailable) {
        case 0: model.dc.dclevel.pSurface = nullptr; break;
        case 1: model.dc.ppdev = nullptr; break;
        case 2: model.device.DriverFunctions.DrawEscape = nullptr; break;
        case 3: model.device.flFlags = PDEV_DISABLED; break;
        default: model.device.flFlags = PDEV_DRIVER_PUNTED_CALL; break;
        }
        check(LightningDrawEscape(handle, private_escape, 0, nullptr) == -1 &&
            model.error == ERROR_NOT_SUPPORTED && balanced(),
            "unavailable callback/mode releases all native owners");
    }
    std::printf("DrawEscape native: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
