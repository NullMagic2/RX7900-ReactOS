/*
Module: rx7900.tools.report
Purpose: Present the shared adapter ABI consistently in every inspection tool.
Created: 2026-10-21
Architecture: Presentation shared by the native client and file inspector.
*/
#pragma once

#include <stdio.h>

#include "protocol.hpp"

namespace rx7900 {

inline const char* bar_name(uint32_t kind) noexcept {
    switch (static_cast<BarKind>(kind)) {
    case BarKind::unused: return "unused";
    case BarKind::io: return "io";
    case BarKind::memory32: return "memory32";
    case BarKind::memory64: return "memory64";
    case BarKind::continuation: return "continuation";
    }

    return "unknown";
}

inline void print_adapter(const Adapter& adapter) {
    printf("%04x:%02x:%02x.%u %04x:%04x revision=%02x subsystem=%04x:%04x\n",
        static_cast<unsigned>(adapter.pci_domain), static_cast<unsigned>(adapter.bus),
        static_cast<unsigned>(adapter.slot), static_cast<unsigned>(adapter.function),
        static_cast<unsigned>(adapter.vendor_id), static_cast<unsigned>(adapter.device_id),
        static_cast<unsigned>(adapter.revision),
        static_cast<unsigned>(adapter.subsystem_vendor_id),
        static_cast<unsigned>(adapter.subsystem_device_id));

    for (uint32_t index = 0; index < pci_bar_count; ++index) {
        const auto& bar = adapter.bars[index];

        printf("  BAR%u %-12s bus_address=0x%016llx prefetch=%u\n",
            static_cast<unsigned>(index), bar_name(bar.kind),
            static_cast<unsigned long long>(bar.bus_address),
            static_cast<unsigned>(bar.flags & bar_prefetchable));
    }
}

} // namespace rx7900
