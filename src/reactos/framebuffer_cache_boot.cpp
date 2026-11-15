/*
Module: rx7900.reactos.framebuffer_cache_boot
Purpose: Pair checked boot-only PAT initialization with exact-release WC PTEs.
Created: 2026-10-21
Architecture: Effective UP/non-PAE only; native code is opt-in at kernel build.
References: Intel SDM253668-085US13.12.4/13.11.8; AMD APM24593rev3.44,7.8.7.
ReactOS integration:822e8647792a2c43f033e3e00ed273d59f163c7b.
License: GPL-2.0-or-later, matching the ReactOS kernel integration.
*/
#include "framebuffer_cache_boot.hpp"

namespace rx7900::reactos::cache_boot {

BootStatus validate_boot(const Snapshot& s) noexcept {
    uint32_t family = (s.cpu_signature >> 8) & 15u;

    if (family == 15) {
        family += (s.cpu_signature >> 20) & 255u;
    }

    if ((s.cpuid_edx & cpuid_msr_pat) != cpuid_msr_pat || family < 6 || s.pentium_lock_erratum) {
        return BootStatus::unsupported_cpu;
    }

    if (s.smp_kernel || s.processor_count != 1 || s.active_affinity != 1) {
        return BootStatus::unsupported_processors;
    }

    if (s.initialization_phase != 1) {
        return BootStatus::wrong_phase;
    }

    if ((s.cr0 & cr0_paging_protection) != cr0_paging_protection ||
        (s.cr0 & cr0_cache_disabled) != 0 || (s.cr4 & cr4_unsupported) != 0 ||
        (s.eflags & (1u << 9)) == 0) {
        return BootStatus::unexpected_control_state;
    }

    // Accept a reset layout or the exact same safe WC layout left by firmware.
    // Every WB/UC byte is preserved; mixed or undocumented layouts are rejected.
    if (s.pat != reset_pat && s.pat != wc_pat) {
        return BootStatus::unexpected_pat;
    }

    return BootStatus::ready;
}

bool uses_changed_pat(uint32_t entry, bool directory) noexcept {
    if ((entry & 1u) == 0) {
        return false;
    }

    // PWT-only directory accesses are rejected too: no page-walk or leaf type
    // may change while the boot code makes PAT1/5 write-combined.
    if (directory && (entry & (1u << 7)) == 0) {
        return (entry & ((1u << 4) | (1u << 3))) == (1u << 3);
    }

    const uint32_t pat_bit = directory ? 12u : 7u;
    const uint32_t index = ((entry >> pat_bit) & 1u) * 4u + ((entry >> 3) & 3u);
    return index == 1 || index == 5;
}

} // namespace rx7900::reactos::cache_boot

#if defined(RX7900_FRAMEBUFFER_CACHE_BOOT_NATIVE)
#include <ntddk.h>
#include <intrin.h>

extern "C" {
// Private kernel data, never exported. Old early mappings remain strict UC
// until this single boot initializer has verified its complete transition.
volatile LONG RxFramebufferCacheBootReady = 0;
}

namespace {
using namespace rx7900::reactos::cache_boot;
constexpr ULONG pat_msr = 0x277;
constexpr ULONG present = 1;
constexpr ULONG large_page = 1u << 7;

// Native non-PAE self-map addresses are supplied by the exact-release header.
// The directory is one resident4KiB page. A present directory entry proves its
// associated self-mapped table page is readable; interrupts-disabled UP excludes
// concurrent page-table mutation. No pointer is derived from user input.
bool no_affected_mappings(ULONG_PTR directory_base, ULONG_PTR table_base) noexcept {
    if ((directory_base & (PAGE_SIZE - 1)) != 0 || (table_base & (PAGE_SIZE - 1)) != 0 ||
        directory_base < 0x80000000u || table_base < 0x80000000u ||
        directory_base > MAXULONG - (PAGE_SIZE - 1) || table_base > MAXULONG - (0x400000u - 1)) {
        return false;
    }

    const auto* directory = reinterpret_cast<const volatile ULONG*>(directory_base);
    const auto* tables = reinterpret_cast<const volatile ULONG*>(table_base);

    for (size_t index = 0; index < 1024; ++index) {
        const ULONG entry = directory[index];

        if ((entry & present) == 0) {
            continue;
        }

        if (uses_changed_pat(entry, true)) {
            return false;
        }

        if ((entry & large_page) != 0) {
            continue;
        }

        for (size_t leaf = 0; leaf < 1024; ++leaf) {
            if (uses_changed_pat(tables[index * 1024 + leaf], false)) {
                return false;
            }
        }
    }

    return true;
}
} // namespace

extern "C" VOID NTAPI RxInitializeFramebufferCacheBoot(
    PULONG64 features, ULONG64 pat_feature, ULONG processor_count, ULONG initialization_phase,
    BOOLEAN smp_kernel, BOOLEAN pentium_lock_erratum, ULONG_PTR directory_base, ULONG_PTR table_base) {
    using namespace rx7900::reactos::cache_boot;
    RxFramebufferCacheBootReady = 0;

    if (features == nullptr || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return;
    }

    int cpu[4]{};
    __cpuid(cpu, 1);
    Snapshot snapshot{};
    snapshot.cpuid_edx = static_cast<uint32_t>(cpu[3]);
    snapshot.cpu_signature = static_cast<uint32_t>(cpu[0]);
    snapshot.processor_count = processor_count;
    snapshot.active_affinity = static_cast<uint32_t>(KeQueryActiveProcessors());
    snapshot.initialization_phase = initialization_phase;
    snapshot.smp_kernel = smp_kernel != FALSE;
    snapshot.pentium_lock_erratum = pentium_lock_erratum != FALSE;

    if ((snapshot.cpuid_edx & cpuid_msr_pat) != cpuid_msr_pat || (*features & pat_feature) == 0) {
        *features &= ~pat_feature;
        DbgPrint("FramebufferCacheBoot: CPU lacks enabled PAT/MSR support, WC disabled\n");
        return;
    }

    snapshot.cr0 = static_cast<uint32_t>(__readcr0());
    snapshot.cr4 = static_cast<uint32_t>(__readcr4());
    snapshot.eflags = static_cast<uint32_t>(__readeflags());

    // Only an architectural PAT/MSR CPU may execute RDMSR. Other prerequisites
    // are checked before it as well, using the expected reset layout temporarily.
    snapshot.pat = reset_pat;
    BootStatus status = validate_boot(snapshot);
    ULONG64 observed_before = 0;
    bool pat_read = false;

    if (status == BootStatus::ready) {
        observed_before = __readmsr(pat_msr);
        pat_read = true;
        snapshot.pat = observed_before;
        status = validate_boot(snapshot);
    }

    if (status != BootStatus::ready || (*features & pat_feature) == 0) {
        *features &= ~pat_feature;
        DbgPrint("FramebufferCacheBoot: disabled status %lu cpus %lu active %lx CR0 %lx CR4 %lx PAT read %lu value %08lx%08lx\n",
            static_cast<ULONG>(status), processor_count, static_cast<ULONG>(snapshot.active_affinity),
            static_cast<ULONG>(snapshot.cr0), static_cast<ULONG>(snapshot.cr4), pat_read ? 1ul : 0ul,
            static_cast<ULONG>(observed_before >> 32), static_cast<ULONG>(observed_before));
        return;
    }

    _disable();

    if (!no_affected_mappings(directory_base, table_base)) {
        *features &= ~pat_feature;
        _enable();
        DbgPrint("FramebufferCacheBoot: existing PAT1/5 mapping, WC disabled\n");
        return;
    }

    // No existing translation selects a changing byte. Use the conservative UP
    // no-fill transition anyway: CD=1/NW=0, WBINVD and full non-global TLB flush,
    // change PAT, then flush again before restoring the exact original CR0.
    // PGE/PCID were rejected, so CR3 reload invalidates every active translation.
    // No MTRR MSR is touched: qemu32 exposes PAT without MTRR capability.
    __writecr0(no_fill_cr0(snapshot.cr0));
    __wbinvd();
    __writecr3(__readcr3());
    __writemsr(pat_msr, wc_pat);
    const ULONG64 observed = __readmsr(pat_msr);
    __writecr3(__readcr3());
    __wbinvd();

    if (observed != wc_pat) {
        // Unknown PAT contents can change WB/UC semantics globally. Continuing
        // with a capability bit cleared would not prove memory consistency.
        KeBugCheckEx(0x80, pat_msr, static_cast<ULONG_PTR>(observed),
            static_cast<ULONG_PTR>(observed >> 32), 0);
    }

    __writecr0(snapshot.cr0);
    __writecr3(__readcr3());
    KeMemoryBarrier();
    RxFramebufferCacheBootReady = 1;
    _enable();
    DbgPrint("FramebufferCacheBoot: UP PAT %08lx%08lx verified, WC PTE index 1 enabled\n",
        static_cast<ULONG>(observed >> 32), static_cast<ULONG>(observed));
}
#endif
