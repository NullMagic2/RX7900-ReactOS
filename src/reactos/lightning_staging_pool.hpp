/*
Module: lightning.reactos.staging_pool
Purpose: Choose the kernel pool for full-frame display RAM (native staging, cached GDI bitmap).
Created: 2026-10-25
Architecture: Shared by lightningvideo (staging) and lightningdisp (cache); no allocation here.
*/
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace lightning::reactos {

// ReactOS MiBuildPagedPool (ntoskrnl/mm/ARM3/mminit.c, pinned a97c8a2) sizes
// paged pool as twice the nonpaged maximum only under _M_IX86. AMD64 keeps
// MI_MIN_INIT_PAGED_POOLSIZE (include/internal/amd64/mm.h) for the whole
// system, and no hive sets Memory Management\PagedPoolSize. One 1920x1080
// staging frame (8.3 MB) plus the desktop's cached bitmap already take half of
// it; the second frame a mode change maps cannot be allocated reliably, so the
// switch fails at its MAP and the old PDEV cannot remap (run 3: last-step 3).
// The AMD64 nonpaged maximum scales with RAM (mm/amd64/init.c, ~400 KB per MB).
constexpr uint64_t reactos_amd64_paged_pool_bytes = 32ull * 1024 * 1024;

enum class FramePool { paged, nonpaged };

// i386 keeps its tested PagedPool behaviour; 64-bit builds use nonpaged pool.
constexpr FramePool frame_pool(size_t pointer_bytes) noexcept {
    return pointer_bytes == 8 ? FramePool::nonpaged : FramePool::paged;
}

// A mode change transiently holds the old and the new frame, beside the rest
// of the system's users of that pool: keep that pair within a quarter of it.
constexpr bool mode_change_fits_pool(uint64_t frame_bytes, uint64_t pool_bytes) noexcept {
    return frame_bytes != 0 && frame_bytes <= pool_bytes / 8;
}

} // namespace lightning::reactos
