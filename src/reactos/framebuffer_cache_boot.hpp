/*
Module: rx7900.reactos.framebuffer_cache_boot
Purpose: Validate the exact-release UP boot prerequisites for genuine PAT WC.
Created: 2026-10-21
Architecture: Shared allocation-free policy; private kernel entry has no IOCTL.
*/
#pragma once

#include <stdint.h>

namespace rx7900::reactos::cache_boot {

constexpr uint64_t reset_pat = UINT64_C(0x0007040600070406);
constexpr uint64_t wc_pat = UINT64_C(0x0007010600070106);
constexpr uint64_t changed_pat_bytes = UINT64_C(0x0000ff000000ff00);
constexpr uint32_t cpuid_msr_pat = (1u << 5) | (1u << 16);
constexpr uint32_t cr0_cache_disabled = (1u << 30) | (1u << 29);
constexpr uint32_t cr0_paging_protection = (1u << 31) | 1u;
constexpr uint32_t cr4_unsupported = (1u << 5) | (1u << 7) | (1u << 17);
constexpr uint32_t no_fill_cr0(uint32_t saved) noexcept {
    return (saved | (1u << 30)) & ~(1u << 29);
}

struct Snapshot final {
    uint64_t pat;
    uint32_t cpuid_edx;
    uint32_t cpu_signature;
    uint32_t cr0;
    uint32_t cr4;
    uint32_t eflags;
    uint32_t processor_count;
    uint32_t active_affinity;
    uint32_t initialization_phase;
    bool smp_kernel;
    bool pentium_lock_erratum;
};

enum class BootStatus : uint32_t {
    ready, unsupported_cpu, unsupported_processors, wrong_phase,
    unexpected_control_state, unexpected_pat, existing_mapping
};

BootStatus validate_boot(const Snapshot& snapshot) noexcept;

// A fixed present entry is read by the native owner only after its containing
// table is known present. directory=true distinguishes a table reference from a
// 4KiB leaf; large directory leaves use PAT bit12 rather than leaf bit7.
bool uses_changed_pat(uint32_t entry, bool directory) noexcept;

struct WriteCombineBits final { uint32_t cache_disable; uint32_t write_through; };
constexpr WriteCombineBits write_combine_bits(bool verified_boot_ready) noexcept {
    return {verified_boot_ready ? 0u : 1u, 1u};
}

} // namespace rx7900::reactos::cache_boot
