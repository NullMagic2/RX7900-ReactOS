/*
Module: rx7900.reactos.framebuffer_shadow_abi
Purpose: Read-only proof of the active cached GDI surface and bounded counters.
Created: 2026-10-21
Architecture: Driver escape returns no pointers, addresses or writable controls.
*/
#pragma once
#include <stdint.h>

namespace rx7900::reactos::framebuffer {
constexpr uint32_t query_shadow_escape = 0x52585348U;
constexpr uint32_t shadow_statistics_magic = 0x53484457U;
constexpr uint32_t shadow_statistics_version = 2;

struct ShadowStatistics final {
    uint32_t size;
    uint32_t version;
    uint32_t magic;
    uint32_t active;
    uint32_t width;
    uint32_t height;
    uint32_t cached_stride;
    uint32_t physical_stride;
    uint64_t bitblt_calls;
    uint64_t copy_calls;
    uint64_t source_only_calls;
    uint64_t flush_calls;
    uint64_t flush_bytes;
    uint64_t failed_calls;
    uint64_t render_ticks;
    uint64_t flush_ticks;
    uint64_t max_flush_ticks;
    uint64_t qpc_frequency;
    uint64_t qpc_failures;
};
static_assert(sizeof(ShadowStatistics)==120,"Fixed read-only framebuf ABI");
} // namespace rx7900::reactos::framebuffer
