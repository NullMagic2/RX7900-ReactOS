/*
Module: rx7900.tests.framebuffer_cache_boot
Purpose: Check boot prerequisites, unchanged PAT bytes and bounded entry selection.
Created: 2026-10-21
Architecture: CPU policy only; never executes privileged instructions.
*/
#include <cstdio>
#include <initializer_list>
#include "framebuffer_cache_boot.hpp"

namespace {
using namespace rx7900::reactos::cache_boot;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

Snapshot supported_boot() {
    return {reset_pat, cpuid_msr_pat, 0x663, cr0_paging_protection | (1u << 16),
        0x600, 0x202, 1, 1, 1, false, false};
}

void check_prerequisites() {
    const auto initial = supported_boot();
    expect(validate_boot(initial) == BootStatus::ready, "qemu32 effective UP reset state is supported");
    auto changed = initial;
    changed.pat = wc_pat;
    expect(validate_boot(changed) == BootStatus::ready, "exact same WC firmware layout is idempotent");
    changed = initial;
    changed.cpuid_edx &= ~(1u << 16);
    expect(validate_boot(changed) == BootStatus::unsupported_cpu, "unsupported PAT blocks privileged MSR operations");
    changed = initial;
    changed.cpuid_edx &= ~(1u << 5);
    expect(validate_boot(changed) == BootStatus::unsupported_cpu, "MSR instruction capability is mandatory");
    changed = initial;
    changed.cpu_signature = 0x543;
    expect(validate_boot(changed) == BootStatus::unsupported_cpu, "Pentium family with F00F WT mapping is rejected");
    changed = initial;
    changed.pentium_lock_erratum = true;
    expect(validate_boot(changed) == BootStatus::unsupported_cpu, "actual kernel F00F workaround flag independently rejects transition");
    changed = initial;
    changed.cpu_signature = 0x00800f11;
    expect(validate_boot(changed) == BootStatus::ready, "architectural extended-family encoding remains checked");

    for (const auto processors : {0u, 2u, 64u, UINT32_MAX}) {
        changed = initial;
        changed.processor_count = processors;
        expect(validate_boot(changed) == BootStatus::unsupported_processors, "only exactly one effective CPU may initialize this bounded backport");
    }

    for (const auto affinity : {0u, 2u, 3u, UINT32_MAX}) {
        changed = initial;
        changed.active_affinity = affinity;
        expect(validate_boot(changed) == BootStatus::unsupported_processors, "active mask must agree with single boot processor ownership");
    }

    changed = initial;
    changed.smp_kernel = true;
    expect(validate_boot(changed) == BootStatus::unsupported_processors, "SMP build cannot invoke unimplemented x86 IPI rendezvous");

    for (const auto phase : {0u, 2u, UINT32_MAX}) {
        changed = initial;
        changed.initialization_phase = phase;
        expect(validate_boot(changed) == BootStatus::wrong_phase, "runtime/phase-zero callers cannot publish boot capability");
    }

    for (const auto bit : {0u, 31u}) {
        changed = initial;
        changed.cr0 &= ~(1u << bit);
        expect(validate_boot(changed) == BootStatus::unexpected_control_state, "paging and protected mode are required before self-map reads");
    }

    for (const auto bit : {29u, 30u}) {
        changed = initial;
        changed.cr0 |= 1u << bit;
        expect(validate_boot(changed) == BootStatus::unexpected_control_state, "unexpected cache-disabled state is rejected rather than repaired");
    }

    for (const auto bit : {5u, 7u, 17u}) {
        changed = initial;
        changed.cr4 |= 1u << bit;
        expect(validate_boot(changed) == BootStatus::unexpected_control_state, "PAE/global/PCID mappings require a different verified transition");
    }

    changed = initial;
    changed.eflags &= ~(1u << 9);
    expect(validate_boot(changed) == BootStatus::unexpected_control_state, "unexpected prior interrupt-disabled state cannot be accidentally enabled");
}

void check_pat_preservation() {
    const auto initial = supported_boot();
    expect((no_fill_cr0(initial.cr0) & cr0_cache_disabled) == (1u << 30) &&
           (no_fill_cr0(initial.cr0) & ~cr0_cache_disabled) == (initial.cr0 & ~cr0_cache_disabled),
           "no-fill transition sets CD, clears NW and preserves every other CR0 bit");
    expect((reset_pat & ~changed_pat_bytes) == (wc_pat & ~changed_pat_bytes),
           "every WB/UC/UC-minus byte is preserved exactly");

    for (uint32_t slot = 0; slot < 8; ++slot) {
        for (uint32_t byte = 0; byte < 256; ++byte) {
            auto snapshot = supported_boot();
            const uint64_t mask = UINT64_C(255) << (slot * 8);
            snapshot.pat = (reset_pat & ~mask) | (static_cast<uint64_t>(byte) << (slot * 8));
            expect((validate_boot(snapshot) == BootStatus::ready) == (snapshot.pat == reset_pat),
                   "any unexpected PAT byte or mixed firmware layout fails closed");
        }
    }

    const auto early = write_combine_bits(false);
    const auto verified = write_combine_bits(true);
    expect(early.cache_disable == 1 && early.write_through == 1,
           "early boot and unsupported CPUs select strict UC index3");
    expect(verified.cache_disable == 0 && verified.write_through == 1,
           "verified boot selects actual programmed WC index1");
}

void check_present_translations() {
    for (uint32_t index = 0; index < 8; ++index) {
        const uint32_t low = ((index & 3u) << 3);
        const uint32_t leaf = 1u | low | ((index >> 2) << 7);
        const uint32_t large = 1u | (1u << 7) | low | ((index >> 2) << 12);
        const bool affected = index == 1 || index == 5;
        expect(uses_changed_pat(leaf, false) == affected, "4KiB translation decodes PAT bit7 plus PCD/PWT");
        expect(uses_changed_pat(large, true) == affected, "large translation decodes PAT bit12 independently of page-size bit7");
        expect(!uses_changed_pat(leaf & ~1u, false) && !uses_changed_pat(large & ~1u, true),
               "nonpresent/software entries never authorize a table dereference");
    }

    expect(uses_changed_pat(1u | (1u << 3), true), "PWT-only page-table access also blocks changed PAT bytes");
    expect(!uses_changed_pat(1u | (1u << 4) | (1u << 3), true), "loader APIC UC mapping is unchanged");
    expect(!uses_changed_pat(1u | (1u << 4), true), "early HAL UC-minus mapping is unchanged");
    expect(uses_changed_pat(0x12345009, false), "nonzero physical frame cannot conceal an affected leaf mapping");
}
} // namespace

int main() {
    check_prerequisites();
    check_pat_preservation();
    check_present_translations();
    std::printf("%u framebuffer cache boot checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
