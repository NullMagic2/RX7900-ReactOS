/*
Module: lightning.reactos.scanout
Purpose: Checked CPU scanout commits against the actual generated Lightning ABI.
Created: 2026-10-21
Architecture: Allocation-free protocol core; native VideoPort owns BAR mappings.
License: GPL-2.0-or-later; generated ABI retained in third_party/qemu_lightning.
*/
#pragma once

#include <stdint.h>
#include "lightning-memory.h"
#include "lightning-display.h"

namespace lightning::reactos {

struct VirtualMode final {
    uint32_t width;
    uint32_t height;
};

// End is exclusive. Validate before adding a PCI BAR base and mapping span;
// 32-bit VideoPort mappings also require the end to stay below 4 GiB.
inline bool valid_bus_span(uint64_t base,uint64_t bytes,uint64_t end_limit) noexcept {
    return bytes!=0 && base<=end_limit && bytes<=end_limit-base;
}

enum class ScanoutStatus {
    success, invalid_access, unavailable, invalid_mode, invalid_backing,
    busy, hardware_failure, state_unknown
};

// The reserved CPU scanout is BAR4 offset zero, never a GPU virtual address or
// renderer blob aperture. Ownership and cache attributes remain native duties.
struct ScanoutAccess final {
    void* context = nullptr;
    bool (*read)(void*,uint32_t offset,uint32_t& value) noexcept = nullptr;
    bool (*write)(void*,uint32_t offset,uint32_t value) noexcept = nullptr;
    bool (*clear)(void*,uint32_t bytes) noexcept = nullptr;
    uint64_t reserved_bytes = 0;
};

struct DisplayCompletion final {
    uint32_t accepted_epoch = 0;
    uint32_t accepted_sequence = 0;
    uint32_t completed_epoch = 0;
    uint32_t completed_sequence = 0;
    uint32_t refresh_ns = 0;
    bool published = false;
};

ScanoutStatus virtual_mode(uint32_t index, VirtualMode& output) noexcept;
uint32_t maximum_scanout_bytes() noexcept;
bool valid_damage(VirtualMode mode, const LightningDisplayRect* rectangles,
                  uint32_t count) noexcept;

// Caller serializes all methods. Acceptance retires synchronous source reads;
// publication is reported separately by completion, never physical vblank.
class CpuScanout final {
public:
    ScanoutStatus initialize(ScanoutAccess access) noexcept;
    ScanoutStatus set_mode(uint32_t index, bool preserve) noexcept;
    ScanoutStatus present() noexcept;
    ScanoutStatus damage(const LightningDisplayRect* rectangles, uint32_t count) noexcept;
    ScanoutStatus completion(DisplayCompletion& output) noexcept;
    ScanoutStatus disable() noexcept;
    ScanoutStatus current_mode(uint32_t& index) const noexcept;
    // Bounded re-query after an uncertain command. The host's accepted prefix
    // must be exactly our acknowledged prefix (the command was not applied) or
    // exactly the command we attempted (applied, acknowledgement lost), and be
    // published or dropped by a failed publication that is no longer pending.
    // The core then adopts it, assumes no mode, and only a FULL in the next
    // epoch or a blank may follow. busy: still publishing or another scanout
    // owner; quarantine stays and a later call may succeed. state_unknown:
    // identity, cadence or any other prefix (a host reset or another writer),
    // or the consecutive limit is spent; quarantine is then permanent.
    ScanoutStatus recover() noexcept;
    bool ready() const noexcept { return ready_; }
    bool quarantined() const noexcept { return quarantined_; }
    static constexpr uint32_t recovery_limit = 3;
private:
    bool geometry_matches(const VirtualMode& mode) noexcept;
    ScanoutStatus submit(uint32_t command, const LightningDisplayRect* rectangles,
                         uint32_t count, bool new_epoch) noexcept;
    ScanoutStatus unknown() noexcept;
    ScanoutAccess access_{};
    uint32_t current_ = 0;
    bool ready_ = false;
    bool current_known_ = false;
    bool quarantined_ = false;
    bool owned_ = false;
    uint32_t epoch_ = 0;
    uint32_t sequence_ = 0;
    // The command written but not yet acknowledged, for recover() only.
    bool attempted_ = false;
    uint32_t attempted_epoch_ = 0;
    uint32_t attempted_sequence_ = 0;
    // Recoveries since a command was accepted outside a recovery replay.
    uint32_t recoveries_ = 0;
    bool replay_required_ = false;
    // A recovered failed publication: the host's completed counters stay below
    // the adopted prefix until the next acceptance and are never published.
    bool dropped_prefix_ = false;
    uint32_t dropped_epoch_ = 0;
    uint32_t dropped_sequence_ = 0;
};

} // namespace lightning::reactos
