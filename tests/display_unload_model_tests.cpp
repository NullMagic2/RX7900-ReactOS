/*
Module: rx7900.tests.display_unload_model
Purpose: Model the removal decisions of reactos-videoprt-remove.patch and the primary
switch of reactos-win32k-display-release.patch.
Created: 2026-10-24
Architecture: Host-only model; the patched C runs only inside ReactOS. Each model
function follows the patched function step by step: videoprt's REMOVE/close accounting
(IntVideoPortRemoveAdapter, IntVideoPortFreeRemovedAdapter, IntVideoPortDispatchClose,
child.c REMOVE), the restart rule of PiPrepareDisabledDeviceForStart
(reactos-pnp-query-remove.patch) over the resulting stack, the close routing that the
LightningNT5 filter's retained attachment relies on (IoGetRelatedDeviceObject), and
win32k's device list, fallback choice, PDEV exchange and release (device.c, pdevobj.c,
display.c). Fixed-capacity fakes; no allocation beyond automatic storage.
*/
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace {

unsigned checks = 0;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "FAIL %s\n", description);
        std::exit(1);
    }
}

// ---------------------------------------------------------------- videoprt ---

constexpr int max_children = 4;

// One adapter FDO with its miniport mappings and child PDOs (videoprt.h fields).
struct Adapter final {
    bool attached = true;          // NextDeviceObject != NULL
    bool removed = false;          // Removed (Interlocked)
    bool freed = false;            // ResourcesFreed (Interlocked)
    long opened = 0;               // DeviceOpened
    bool interrupt = true;
    bool in_reset_list = true;     // HwResetListEntry linked
    bool vga_owner = false;        // VgaDeviceExtension == this
    bool deleted = false;          // IoDeleteDevice ran
    int unmaps = 0;                // mapping teardowns
    int inbv_resets = 0;           // IntVideoPortResetDisplayParametersEx from a close
    int children = 0;              // ChildDeviceList entries
    int deleted_children = 0;

    // IntVideoPortFreeRemovedAdapter: once, only when removed and closed.
    void free_if_unused() {
        if (!removed || opened != 0 || freed) return;
        freed = true;
        ++unmaps;
    }

    // IntVideoPortStopRemovedAdapter: idempotent.
    void stop() {
        interrupt = false;
        in_reset_list = false;
        vga_owner = false;
    }

    // I/O manager: a delete-pending device refuses opens.
    bool open() {
        if (deleted) return false;
        ++opened;
        return true;
    }

    // IntVideoPortDispatchClose.
    void close() {
        if (removed) {
            if (opened >= 1) --opened;
            free_if_unused();
            return;
        }
        if (opened >= 1 && --opened == 0) ++inbv_resets;
    }

    // child.c REMOVE: the child leaves the list before its PDO is deleted.
    void child_removed() {
        if (children > 0) --children;
        ++deleted_children;
    }

    // IntVideoPortRemoveAdapter.
    void remove() {
        stop();
        deleted_children += children;
        children = 0;
        attached = false;
        removed = true;
        free_if_unused();
        deleted = true;
    }
};

void adapter_closed_before_remove() {
    Adapter a;
    a.open();
    a.remove();
    expect(!a.attached && a.deleted, "REMOVE detaches and deletes the FDO");
    expect(a.unmaps == 0, "mappings stay while win32k keeps the display open");
    expect(!a.interrupt && !a.in_reset_list, "no interrupt or INBV reset after REMOVE");
    expect(!a.open(), "a removed adapter refuses a new open");
    a.close();
    expect(a.unmaps == 1, "the last close unmaps exactly once");
    expect(a.inbv_resets == 0, "closing a removed adapter never resets the other displays");
    a.close();
    expect(a.unmaps == 1 && a.opened == 0, "an extra close neither unmaps again nor underflows");
}

void adapter_never_opened() {
    Adapter a;
    a.remove();
    expect(a.unmaps == 1, "an adapter nobody opened unmaps at REMOVE");
}

void stock_close_resets_inbv() {
    Adapter a;
    a.open();
    a.close();
    expect(a.inbv_resets == 1, "a live adapter's last close keeps the stock INBV reset");
    expect(a.unmaps == 0, "a live adapter keeps its mappings");
}

void surprise_then_remove() {
    Adapter a;
    a.vga_owner = true;
    a.open();
    a.stop();
    expect(!a.interrupt && !a.vga_owner, "SURPRISE_REMOVAL stops interrupt and VGA fallback");
    a.remove();
    expect(!a.in_reset_list && a.unmaps == 0, "REMOVE after SURPRISE repeats the stop safely");
}

void children_leave_the_list() {
    Adapter a;
    a.children = 2;
    a.child_removed();
    expect(a.children == 1, "a removed child leaves the adapter's list");
    a.remove();
    expect(a.children == 0 && a.deleted_children == 2, "REMOVE deletes the child never removed");
}

// REMOVE (set Removed, then check) and the last close (decrement, then check)
// race through Interlocked operations: every interleaving frees exactly once.
void free_once_in_every_interleaving() {
    // 0 = remove sets Removed, 1 = remove checks, 2 = close decrements, 3 = close checks
    const int orders[][4] = {
        {0, 1, 2, 3}, {0, 2, 1, 3}, {0, 2, 3, 1}, {2, 0, 1, 3}, {2, 0, 3, 1}, {2, 3, 0, 1},
    };
    for (const auto& order : orders) {
        bool removed = false;
        long opened = 1;
        bool freed = false;
        int unmaps = 0;
        auto check = [&]() {
            if (removed && opened == 0 && !freed) {
                freed = true;
                ++unmaps;
            }
        };
        for (int step : order) {
            if (step == 0) removed = true;
            if (step == 2) --opened;
            if (step == 1 || step == 3) check();
        }
        expect(unmaps == 1, "exactly one unmap in this REMOVE/close interleaving");
    }
}

// ------------------------------------------------- PnP restart and CLOSE path ---

// The PCI PDO's stack after a Device Manager disable.
struct Stack final {
    bool pdo_attached = false;     // PDO->AttachedDevice
    int child_nodes = 0;           // monitor device nodes still allocated
    bool gone = false;             // DNF_DEVICE_GONE
    bool delete_pending = false;   // the PDO itself
};

// PiPrepareDisabledDeviceForStart: restart only a torn-down stack.
bool restartable(const Stack& stack) {
    return !stack.gone && !stack.delete_pending && !stack.pdo_attached && stack.child_nodes == 0;
}

void restart_needs_videoprt_teardown() {
    Stack stock{true, 0, false, false};
    expect(!restartable(stock), "stock videoprt stays attached: Cannot restart");

    // Patched videoprt detached; the filter stays on the removed FDO, not the PDO.
    Stack patched{false, 1, false, false};
    expect(!restartable(patched), "the monitor node win32k still references blocks restart");
    patched.child_nodes = 0;
    expect(restartable(patched), "after win32k drops the monitor PDO the device restarts");

    Stack surprise{false, 0, true, false};
    expect(!restartable(surprise), "a surprise-removed device is never restarted");
}

// IoGetRelatedDeviceObject: win32k's final CLOSE goes to the top of the stack
// above the FDO the file object names, as attached at close time.
enum class CloseTarget { filter, fdo };

CloseTarget close_target(bool filter_attached_to_fdo) {
    return filter_attached_to_fdo ? CloseTarget::filter : CloseTarget::fdo;
}

void retained_filter_sees_the_final_close() {
    expect(close_target(false) == CloseTarget::fdo,
           "a filter detached at REMOVE would never see win32k's close");
    expect(close_target(true) == CloseTarget::filter,
           "the retained attachment routes win32k's close to the filter");
}

// ------------------------------------------------------------------ win32k ---

constexpr int max_devices = 6;
constexpr int max_fallbacks = 4;

enum class Removal { present, pending, handled };

struct GraphicsDevice final {
    int id = 0;
    bool linked = false;           // in gpGraphicsDeviceFirst list
    int vga = -1;                  // pVgaDevice index
    Removal removal = Removal::present;
    bool file_object = true;       // FileObject reference held
    int monitors = 1;              // pvMonDev references held
    bool primary_flag = false;     // DISPLAY_DEVICE_PRIMARY_DEVICE
    bool attached_flag = false;    // DISPLAY_DEVICE_ATTACHED_TO_DESKTOP
    bool vga_compatible = false;   // DISPLAY_DEVICE_VGA_COMPATIBLE
    bool can_switch = true;        // whether a PDEV can be created on it
};

struct Win32k final {
    GraphicsDevice devices[max_devices]{};
    int order[max_devices]{};      // list order of linked devices
    int count = 0;
    int linked = 0;
    int primary = -1;              // gpPrimaryGraphicsDevice
    int vga_device = -1;           // gpVgaGraphicsDevice
    int desktop = -1;              // device of the single desktop PDEV
    int desktop_displays = 1;      // gpmdev->cDev
    int extra_pdev = -1;           // a device another PDEV still uses
    int queued = 0;                // removal work items queued
    int moves = 0;
    int refused_primary = 0;       // "no other display took the desktop"
    int refused_in_use = 0;        // "is in use or the VGA device"

    int add(bool vga) {
        GraphicsDevice& device = devices[count];
        device.id = count;
        device.vga_compatible = vga;
        device.linked = true;
        order[linked++] = count;
        return count++;
    }

    void unlink(int index) {
        int to = 0;
        for (int i = 0; i < linked; ++i) {
            if (order[i] != index) order[to++] = order[i];
        }
        linked = to;
        devices[index].linked = false;
    }

    void link(int index) {
        for (int i = 0; i < linked; ++i) {
            if (order[i] == index) return;
        }
        order[linked++] = index;
        devices[index].linked = true;
    }

    // EngpUpdateGraphicsDeviceList's primary choice and VGA link.
    void boot(int first_primary, int vga) {
        primary = first_primary;
        devices[primary].primary_flag = true;
        devices[primary].attached_flag = true;
        vga_device = vga;
        if (vga >= 0 && vga != primary) {
            unlink(vga);
            devices[primary].vga = vga;
        }
        desktop = primary;
    }

    // EngpGraphicsDeviceRemoveCallback with a desktop thread available.
    void remove_complete(int index) {
        if (devices[index].removal == Removal::present) devices[index].removal = Removal::pending;
        ++queued;
    }

    // EngpTakeRemovedGraphicsDevice.
    int take_removed() {
        for (int i = 0; i < linked; ++i) {
            GraphicsDevice& device = devices[order[i]];
            if (device.removal == Removal::pending) {
                device.removal = Removal::handled;
                return device.id;
            }
        }
        if (primary >= 0 && devices[primary].vga >= 0 &&
            devices[devices[primary].vga].removal == Removal::pending) {
            devices[devices[primary].vga].removal = Removal::handled;
            return devices[primary].vga;
        }
        return -1;
    }

    // EngpGetDesktopFallbacks.
    bool fallbacks(int removed, int* out, int& n) {
        n = 0;
        if (removed != primary) return false;
        for (int i = 0; i < linked && n < max_fallbacks; ++i) {
            if (order[i] != removed && devices[order[i]].removal == Removal::present) out[n++] = order[i];
        }
        const int vga = devices[removed].vga;
        if (n < max_fallbacks && vga >= 0 && devices[vga].removal == Removal::present) out[n++] = vga;
        return true;
    }

    // UserMoveDesktopToDevice.
    bool move(int removed, int target) {
        if (desktop_displays != 1 || desktop != removed || !devices[target].can_switch) return false;
        desktop = target;
        devices[target].attached_flag = true;
        ++moves;
        return true;
    }

    // EngpSetPrimaryGraphicsDevice.
    void set_primary(int old_primary, int target) {
        devices[old_primary].primary_flag = false;
        devices[old_primary].attached_flag = false;
        devices[target].primary_flag = true;
        devices[target].attached_flag = true;
        primary = target;
    }

    // EngpReleaseRemovedGraphicsDeviceIfUnused.
    bool release(int index) {
        if (desktop == index || extra_pdev == index) return false;
        if (index == primary || index == vga_device) return false;
        GraphicsDevice& device = devices[index];
        unlink(index);
        if (device.vga >= 0) link(device.vga);
        device.vga = -1;
        device.monitors = 0;
        device.file_object = false;
        return true;
    }

    // UserProcessDisplayRemovals; returns devices released.
    int process() {
        int released = 0;
        int removed;
        while ((removed = take_removed()) >= 0) {
            int candidates[max_fallbacks];
            int n = 0;
            if (fallbacks(removed, candidates, n)) {
                bool moved = false;
                for (int i = 0; i < n && !moved; ++i) {
                    moved = move(removed, candidates[i]);
                    if (moved) set_primary(removed, candidates[i]);
                }
                if (!moved) {
                    ++refused_primary;
                    continue;
                }
            }
            if (release(removed)) ++released;
            else ++refused_in_use;
        }
        return released;
    }
};

void primary_moves_to_linked_vga() {
    Win32k w;
    const int lightning = w.add(false);
    const int vga = w.add(true);
    w.boot(lightning, vga);
    expect(w.linked == 1 && !w.devices[vga].linked, "boot links the VGA device to the primary");
    w.remove_complete(lightning);
    expect(w.devices[lightning].file_object, "the PnP callback itself releases nothing");
    expect(w.process() == 1, "the desktop thread releases the removed primary");
    expect(w.desktop == vga && w.primary == vga, "the desktop and primary moved to VGA");
    expect(w.devices[vga].primary_flag && w.devices[vga].attached_flag, "VGA carries the primary flags");
    expect(!w.devices[lightning].primary_flag && !w.devices[lightning].attached_flag,
           "the removed device lost the primary flags");
    expect(w.devices[vga].linked && !w.devices[lightning].linked && w.linked == 1,
           "VGA is back in the list, the removed device is out");
    expect(!w.devices[lightning].file_object && w.devices[lightning].monitors == 0,
           "the file object and monitor references are dropped");
    expect(w.devices[lightning].vga == -1, "the released device no longer links VGA");
}

void primary_without_other_display_is_kept() {
    Win32k w;
    const int lightning = w.add(false);
    w.boot(lightning, -1);
    w.remove_complete(lightning);
    expect(w.process() == 0, "a lone primary is not released");
    expect(w.devices[lightning].file_object && w.desktop == lightning, "it keeps its references and the desktop");
    expect(w.devices[lightning].removal == Removal::handled, "the request is consumed, not retried");
    expect(w.refused_primary == 1 && w.refused_in_use == 0, "it is refused as the primary, once");
    w.remove_complete(lightning);
    w.process();
    expect(w.refused_primary == 1, "a handled device is not handled again");
}

void listed_display_is_preferred_and_failure_falls_back() {
    Win32k w;
    const int lightning = w.add(false);
    const int second = w.add(false);
    const int vga = w.add(true);
    w.boot(lightning, vga);
    int candidates[max_fallbacks];
    int n = 0;
    expect(w.fallbacks(lightning, candidates, n) && n == 2 && candidates[0] == second && candidates[1] == vga,
           "registered displays come before the linked VGA device");
    w.devices[second].can_switch = false;
    w.remove_complete(lightning);
    w.process();
    expect(w.desktop == vga && w.primary == vga, "a display without a usable PDEV falls back to VGA");
    expect(w.devices[second].linked && w.devices[vga].linked, "both remaining displays stay listed");
}

void removed_fallback_is_skipped() {
    Win32k w;
    const int lightning = w.add(false);
    const int second = w.add(false);
    const int vga = w.add(true);
    w.boot(lightning, vga);
    w.remove_complete(second);
    w.remove_complete(lightning);
    w.process();
    expect(w.desktop == vga && w.moves == 1, "a display that is itself removed never takes the desktop");
    expect(!w.devices[second].file_object && !w.devices[lightning].file_object, "both removed devices released");
}

void secondary_and_vga_rules() {
    Win32k w;
    const int lightning = w.add(false);
    const int second = w.add(false);
    const int vga = w.add(true);
    w.boot(lightning, vga);
    w.extra_pdev = second;
    w.remove_complete(second);
    expect(w.process() == 0 && w.devices[second].file_object, "a secondary a PDEV uses is kept");
    w.extra_pdev = -1;
    Win32k unused;
    const int primary = unused.add(false);
    const int spare = unused.add(false);
    unused.boot(primary, -1);
    unused.remove_complete(spare);
    expect(unused.process() == 1 && !unused.devices[spare].file_object, "an unused secondary is released");
    expect(unused.desktop == primary && unused.moves == 0, "releasing a secondary never moves the desktop");
    w.remove_complete(vga);
    expect(w.process() == 0 && w.devices[vga].file_object, "the VGA device is never released");
    expect(w.devices[vga].removal == Removal::handled && w.refused_in_use == 2,
           "the linked VGA device's request is taken and refused");
}

void multi_display_desktop_is_not_moved() {
    Win32k w;
    const int lightning = w.add(false);
    const int vga = w.add(true);
    w.boot(lightning, vga);
    w.desktop_displays = 2;
    w.remove_complete(lightning);
    expect(w.process() == 0 && w.desktop == lightning, "a meta-device desktop keeps today's refusal");
}

void repeated_notification_is_handled_once() {
    Win32k w;
    const int lightning = w.add(false);
    const int vga = w.add(true);
    w.boot(lightning, vga);
    w.remove_complete(lightning);
    w.remove_complete(lightning);
    expect(w.process() == 1 && w.moves == 1, "two notifications move and release once");
    w.remove_complete(lightning);
    expect(w.process() == 0 && w.moves == 1, "a handled device stays handled");
    expect(w.queued == 3, "each notification still queues the worker");
}

// ------------------------------------------------- PDEV exchange (pdevobj.c) ---

constexpr std::uint32_t pdev_display = 0x1;
constexpr std::uint32_t pdev_hardware_pointer = 0x2;
constexpr std::uint32_t pdev_software_pointer = 0x4;
constexpr std::uint32_t pdev_disabled = 0x400;
constexpr std::uint32_t driver_state = pdev_disabled | pdev_hardware_pointer | pdev_software_pointer;

struct Device final {
    std::uint32_t state_flags = 0;
};

// The PDEVOBJ fields that belong to one driver instance or one device.
struct Pdev final {
    int pfn = 0, pldev = 0, dhpdev = 0, surface = 0, devinfo = 0, gdiinfo = 0, devmode = 0;
    int spooler = 0, palette = 0, move_pointer = 0;
    std::uint32_t flags = pdev_display;
    Device* device = nullptr;
};

template <typename T>
void swap_values(T& a, T& b) {
    const T temp = a;
    a = b;
    b = temp;
}

// PDEVOBJ_bDynamicModeChange as upstream: the instance moves, and so do the
// two devices' StateFlags.
void dynamic_mode_change(Pdev& a, Pdev& b) {
    swap_values(a.pfn, b.pfn);
    swap_values(a.pldev, b.pldev);
    swap_values(a.dhpdev, b.dhpdev);
    swap_values(a.surface, b.surface);
    swap_values(a.devinfo, b.devinfo);
    swap_values(a.gdiinfo, b.gdiinfo);
    swap_values(a.devmode, b.devmode);
    swap_values(a.device->state_flags, b.device->state_flags);
}

// PDEVOBJ_bSwitchGraphicsDevice after a successful mode change exchange.
void switch_graphics_device(Pdev& ppdev, Pdev& tmp) {
    dynamic_mode_change(ppdev, tmp);
    swap_values(ppdev.device->state_flags, tmp.device->state_flags);
    swap_values(ppdev.device, tmp.device);
    swap_values(ppdev.spooler, tmp.spooler);
    swap_values(ppdev.palette, tmp.palette);
    swap_values(ppdev.move_pointer, tmp.move_pointer);
    const std::uint32_t flags = ppdev.flags;
    ppdev.flags = (flags & ~driver_state) | (tmp.flags & driver_state);
    tmp.flags = (tmp.flags & ~driver_state) | (flags & driver_state);
}

Pdev instance(int base, Device& device, std::uint32_t flags) {
    Pdev pdev;
    pdev.pfn = base + 1;
    pdev.pldev = base + 2;
    pdev.dhpdev = base + 3;
    pdev.surface = base + 4;
    pdev.devinfo = base + 5;
    pdev.gdiinfo = base + 6;
    pdev.devmode = base + 7;
    pdev.spooler = base + 8;
    pdev.palette = base + 9;
    pdev.move_pointer = base + 10;
    pdev.flags = flags;
    pdev.device = &device;
    return pdev;
}

bool owns(const Pdev& pdev, int base, const Device& device) {
    return pdev.pfn == base + 1 && pdev.pldev == base + 2 && pdev.dhpdev == base + 3 &&
        pdev.surface == base + 4 && pdev.devinfo == base + 5 && pdev.gdiinfo == base + 6 &&
        pdev.devmode == base + 7 && pdev.spooler == base + 8 && pdev.palette == base + 9 &&
        pdev.move_pointer == base + 10 && pdev.device == &device;
}

void pdev_moves_to_the_new_device() {
    Device removed{0x5};   // primary | attached
    Device vga{0x10};      // VGA compatible
    Pdev desktop = instance(100, removed, pdev_display | pdev_hardware_pointer);
    Pdev tmp = instance(200, vga, pdev_display);
    switch_graphics_device(desktop, tmp);
    expect(owns(desktop, 200, vga), "the desktop PDEV now holds the VGA instance and device");
    expect(owns(tmp, 100, removed), "the temporary PDEV holds the removed instance for its release");
    expect(removed.state_flags == 0x5 && vga.state_flags == 0x10,
           "neither device's StateFlags moved; the caller sets the primary flags");
    expect((desktop.flags & pdev_hardware_pointer) == 0 && (desktop.flags & pdev_display) != 0,
           "the old driver's pointer mode stays with it; PDEV_DISPLAY stays");
    expect((tmp.flags & pdev_hardware_pointer) != 0, "the released instance keeps its own pointer mode");
}

void same_device_mode_change_is_unchanged() {
    Device device{0x5};
    Pdev desktop = instance(100, device, pdev_display);
    Pdev tmp = instance(200, device, pdev_display);
    dynamic_mode_change(desktop, tmp);
    expect(desktop.pfn == 201 && desktop.device == &device && desktop.spooler == 108,
           "a mode change keeps device-named fields");
    expect(device.state_flags == 0x5, "swapping one device's flags with itself is a no-op");
}

} // namespace

int main() {
    adapter_closed_before_remove();
    adapter_never_opened();
    stock_close_resets_inbv();
    surprise_then_remove();
    children_leave_the_list();
    free_once_in_every_interleaving();
    restart_needs_videoprt_teardown();
    retained_filter_sees_the_final_close();
    primary_moves_to_linked_vga();
    primary_without_other_display_is_kept();
    listed_display_is_preferred_and_failure_falls_back();
    removed_fallback_is_skipped();
    secondary_and_vga_rules();
    multi_display_desktop_is_not_moved();
    repeated_notification_is_handled_once();
    pdev_moves_to_the_new_device();
    same_device_mode_change_is_unchanged();
    std::printf("display_unload_model_tests: %u checks passed\n", checks);
    return 0;
}
