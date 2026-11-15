/*
Module: rx7900.reactos.radeon_2d_ioctl
Purpose: Validate private Radeon 2D IOCTLs against miniport-owned sessions,
surfaces and staging, then drive the bound SDMA 2D backend.
Created: 2026-10-22
Architecture: Portable core plus optional VideoPort glue (RX7900_RADEON2D_VIDEOPORT).
Security argument: every request is decoded through ByteView at fixed offsets
with exact length; every object reference is a (slot, generation) handle plus a
per-session random key; surfaces are usable only by their owning session (the
primary only by the session currently holding it); every rectangle is checked
against the referenced surface in 64-bit arithmetic; every staging access is
proven to lie inside the caller's own miniport-allocated staging buffer. No
request can name a GPU, physical or virtual address, so the interface cannot be
turned into an arbitrary memory read or write primitive.
*/
#if defined(RX7900_RADEON2D_VIDEOPORT)
#include "video_sdk.hpp"
#endif
#include "radeon_2d_ioctl.hpp"

namespace rx7900::reactos::radeon2d {
namespace {

constexpr uint32_t handle_index_mask = 0xffU;
constexpr uint32_t handle_generation_shift = 8;
constexpr uint32_t generation_limit = 0x00ffffffU;

constexpr uint32_t make_handle(uint32_t index, uint32_t generation) noexcept {
    return (index + 1U) | (generation << handle_generation_shift);
}

// Returns false for the reserved zero index; output is the slot index.
constexpr bool handle_slot(uint32_t handle, uint32_t limit, uint32_t& index) noexcept {
    const uint32_t encoded = handle & handle_index_mask;
    if (encoded == 0 || encoded > limit) {
        return false;
    }

    index = encoded - 1U;
    return true;
}

constexpr uint32_t handle_generation(uint32_t handle) noexcept {
    return handle >> handle_generation_shift;
}

constexpr uint32_t next_generation(uint32_t generation) noexcept {
    return generation >= generation_limit ? 1U : generation + 1U;
}

void saturating_increment(uint32_t& value) noexcept {
    if (value != UINT32_MAX) {
        ++value;
    }
}

// Each advertised raster operation has the callbacks it needs.
bool complete_raster(const Blit2dBackend& backend) noexcept {
    const uint64_t ops = backend.raster_operations;
    const uint64_t rop_ops = abi::operation_bit(abi::ioctl_rop) | abi::operation_bit(abi::ioctl_pattern) |
                             abi::operation_bit(abi::ioctl_stretch) | abi::operation_bit(abi::ioctl_text) |
                             abi::operation_bit(abi::ioctl_stretch_rop);
    return (ops & ~abi::raster_operations) == 0 &&
           ((ops & rop_ops) == 0 || backend.raster_rop != nullptr) &&
           ((ops & (abi::operation_bit(abi::ioctl_alpha) | abi::operation_bit(abi::ioctl_text))) == 0 ||
            backend.raster_alpha != nullptr) &&
           ((ops & abi::operation_bit(abi::ioctl_line)) == 0 || backend.raster_line != nullptr) &&
           ((ops & abi::operation_bit(abi::ioctl_gradient)) == 0 || backend.raster_gradient != nullptr);
}

// ROP3 operand use (DIB_DoRop minterm index P << 2 | S << 1 | D).
constexpr bool rop3_uses_source(uint32_t rop3) noexcept {
    return ((rop3 ^ (rop3 >> 2)) & 0x33U) != 0;
}

bool rop4_uses_source(uint32_t rop4) noexcept {
    return rop3_uses_source(rop4 & 0xffU) || rop3_uses_source((rop4 >> 8) & 0xffU);
}

bool rect_overlap(const abi::Rect& a, int64_t left, int64_t top, int64_t right, int64_t bottom) noexcept {
    return a.left < right && left < a.right && a.top < bottom && top < a.bottom;
}

bool complete(const Blit2dBackend& backend) noexcept {
    return backend.fill != nullptr && backend.copy != nullptr && backend.upload != nullptr &&
           backend.readback != nullptr && backend.surface_create != nullptr &&
           backend.surface_destroy != nullptr && backend.staging_create != nullptr &&
           backend.staging_destroy != nullptr && backend.sync != nullptr &&
           backend.completed != nullptr && backend.surface_evict != nullptr &&
           backend.surface_restore != nullptr && complete_raster(backend);
}

constexpr uint32_t session_capabilities = abi::capability_fill | abi::capability_copy |
    abi::capability_upload | abi::capability_readback | abi::capability_offscreen |
    abi::capability_primary | abi::capability_eviction;

// Bytes an offscreen surface accounts against the budgets (valid_surface
// bounds both factors, so the product cannot overflow).
uint64_t surface_bytes(const EngineSurface& surface) noexcept {
    return static_cast<uint64_t>(surface.pitch_bytes) * surface.height;
}

bool same_geometry(const EngineSurface& a, const EngineSurface& b) noexcept {
    return a.width == b.width && a.height == b.height && a.pitch_bytes == b.pitch_bytes &&
           a.format == b.format;
}

// Surface layout usable by the 4-byte-aligned SDMA fill/copy encoders.
bool valid_surface(const EngineSurface& surface) noexcept {
    return surface.format == abi::format_xrgb8888 && surface.width != 0 && surface.height != 0 &&
           surface.width <= abi::max_surface_dimension &&
           surface.height <= abi::max_surface_dimension && (surface.pitch_bytes & 3U) == 0 &&
           static_cast<uint64_t>(surface.width) * abi::bytes_per_pixel <= surface.pitch_bytes &&
           surface.gpu_address != 0 && (surface.gpu_address & 3U) == 0 &&
           surface.gpu_address <= sdma::gpu_virtual_address_max &&
           static_cast<uint64_t>(surface.pitch_bytes) * surface.height <=
               sdma::gpu_virtual_address_max - surface.gpu_address;
}

EngineRect engine_rect(const abi::Rect& rect) noexcept {
    // rect_inside proved 0 <= left < right <= width (and likewise vertically).
    return {static_cast<uint32_t>(rect.left), static_cast<uint32_t>(rect.top),
            static_cast<uint32_t>(rect.right - rect.left),
            static_cast<uint32_t>(rect.bottom - rect.top)};
}

bool engine_intersect(const EngineRect& a, uint32_t bx, uint32_t by, uint32_t bw,
                      uint32_t bh) noexcept {
    const uint64_t a_right = static_cast<uint64_t>(a.x) + a.width;
    const uint64_t a_bottom = static_cast<uint64_t>(a.y) + a.height;
    const uint64_t b_right = static_cast<uint64_t>(bx) + bw;
    const uint64_t b_bottom = static_cast<uint64_t>(by) + bh;
    return a.x < b_right && bx < a_right && a.y < b_bottom && by < a_bottom;
}

} // namespace

bool rect_inside(const abi::Rect& rect, uint32_t width, uint32_t height) noexcept {
    return rect.left >= 0 && rect.top >= 0 && rect.left < rect.right && rect.top < rect.bottom &&
           static_cast<int64_t>(rect.right) <= static_cast<int64_t>(width) &&
           static_cast<int64_t>(rect.bottom) <= static_cast<int64_t>(height);
}

bool rects_intersect(const abi::Rect& a, const abi::Rect& b) noexcept {
    return a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom;
}

bool Dispatcher::configure(const Blit2dBackend& backend, const EngineSurface& primary,
                           uint32_t staging_bytes, uint32_t seed) noexcept {
    Limits limits{};
    limits.staging_bytes = staging_bytes;
    return configure(backend, primary, limits, seed);
}

bool Dispatcher::configure(const Blit2dBackend& backend, const EngineSurface& primary,
                           const Limits& limits, uint32_t seed) noexcept {
    const uint32_t staging_bytes = limits.staging_bytes;
    if (configured_ || disabled_ || !complete(backend) || !valid_surface(primary) || staging_bytes == 0 ||
        (staging_bytes & 3U) != 0 || surfaces_[0].quarantined ||
        limits.session_surface_bytes > limits.device_surface_bytes) {
        return false;
    }

    backend_ = backend;
    staging_bytes_ = staging_bytes;
    session_surface_bytes_ = limits.session_surface_bytes;
    device_surface_bytes_ = limits.device_surface_bytes;
    used_surface_bytes_ = 0;
    suspended_ = false;
    key_state_ = seed != 0 ? seed : 0x9e3779b9U;
    primary_owner_ = abi::max_sessions;
    Surface& slot = surfaces_[0];
    slot.used = true;
    slot.primary = true;
    slot.generation = next_generation(slot.generation);
    slot.owner = abi::max_sessions;
    slot.allocation = 0;
    slot.last_use = 0;
    slot.engine = primary;
    configured_ = true;
    return true;
}

bool Dispatcher::shutdown() noexcept {
    if (!configured_) {
        return true;
    }

    // A disabled dispatcher never calls its backend again: everything stays
    // retained (the display DLL may still hold staging views).
    if (disabled_) {
        return false;
    }

    bool retired = true;
    for (Session& session : sessions_) {
        if (session.open && !retire_session(session)) {
            retired = false;
        }
    }

    // The primary is miniport-owned; only its handle is invalidated here. A
    // later configure issues a new generation for the re-adopted scanout.
    surfaces_[0].used = false;
    surfaces_[0].primary = false;
    primary_owner_ = abi::max_sessions;
    configured_ = false;
    suspended_ = false;
    used_surface_bytes_ = 0;
    return retired;
}

bool Dispatcher::suspend() noexcept {
    if (!configured_ || suspended_ || recovering_ || disabled_) {
        return false;
    }

    bool saved = true;
    for (uint32_t owner = 0; owner < abi::max_sessions; ++owner) {
        Session& session = sessions_[owner];
        if (!session.open) {
            continue;
        }

        const bool idle = !session.poisoned &&
            (session.last_fence <= completed() ||
             backend_.sync(backend_.context, session.last_fence) == BackendStatus::ok);
        if (!idle) {
            // Work may still read or write its surfaces: nothing of it is moved.
            session.poisoned = true;
            saved = false;
            continue;
        }

        for (uint32_t index = 1; index < abi::max_surfaces; ++index) {
            Surface& surface = surfaces_[index];
            if (!surface.used || surface.primary || surface.owner != owner || surface.quarantined ||
                surface.lost || surface.evicted) {
                continue;
            }

            EngineSurface moved{};
            const BackendStatus status = backend_.surface_evict(backend_.context, surface.allocation, moved);
            if (status == BackendStatus::ok && valid_surface(moved) &&
                same_geometry(moved, surface.engine)) {
                surface.engine = moved;
                surface.evicted = true;
                saturating_increment(counters_.evicted);
                continue;
            }

            if (status == BackendStatus::uncertain) {
                // Both placements stay retained until a reset; a recovery then
                // reports the surface lost (its move never proved complete).
                surface.quarantined = true;
                surface.uncertain = true;
                session.poisoned = true;
                saved = false;
                continue;
            }

            // No system memory for it (or a malformed placement): the surface was
            // proven idle, so it is freed now and its handle reports the loss.
            backend_.surface_destroy(backend_.context, surface.allocation);
            surface.allocation = 0;
            surface.lost = true;
            used_surface_bytes_ -= surface.bytes;
            session.surface_bytes -= surface.bytes;
            surface.bytes = 0;
            saturating_increment(counters_.lost);
        }
    }

    suspended_ = true;
    return saved;
}

bool Dispatcher::resume(const Blit2dBackend& backend, const EngineSurface& primary) noexcept {
    if (!configured_ || !suspended_ || recovering_ || disabled_ || !complete(backend) ||
        !valid_surface(primary)) {
        return false;
    }

    backend_ = backend;
    Surface& slot = surfaces_[0];
    if (!same_geometry(slot.engine, primary)) {
        // A different scanout is a different surface: old handles stop working.
        slot.generation = next_generation(slot.generation);
        primary_owner_ = abi::max_sessions;
    }

    slot.engine = primary;
    slot.last_use = 0;
    for (uint32_t index = 1; index < abi::max_surfaces; ++index) {
        Surface& surface = surfaces_[index];
        if (!surface.used || !surface.evicted || surface.quarantined) {
            continue;
        }

        EngineSurface moved{};
        const BackendStatus status = backend_.surface_restore(backend_.context, surface.allocation, moved);
        if (status == BackendStatus::ok && valid_surface(moved) && same_geometry(moved, surface.engine)) {
            surface.engine = moved;
            surface.evicted = false;
            saturating_increment(counters_.restored);
        } else if (status == BackendStatus::uncertain && surface.owner < abi::max_sessions) {
            sessions_[surface.owner].poisoned = true;
        }
        // rejected: it stays usable in system memory.
    }

    suspended_ = false;
    return true;
}

bool Dispatcher::begin_recovery() noexcept {
    if (!configured_ || recovering_ || disabled_) {
        return false;
    }

    // Non-blocking: a hung engine must not be waited for here. Everything at or
    // below this fence provably retired before the reset; everything above may
    // or may not have executed when the reset stopped the engines.
    recovery_completed_ = completed();
    for (uint32_t index = 1; index < abi::max_surfaces; ++index) {
        Surface& surface = surfaces_[index];
        if (surface.used && !surface.primary && surface.last_write > recovery_completed_) {
            surface.uncertain = true;
        }
    }

    recovering_ = true;
    return true;
}

// One offscreen surface after the reset proof and the backend's restart.
bool Dispatcher::recover_surface(Surface& surface) noexcept {
    if (surface.quarantined && surface.uncertain && !surface.destroyed && sessions_[surface.owner].open) {
        // Quarantined by an uncertain move (eviction) while its session lives:
        // the session still counts it and the DLL still names it, so it is
        // reported lost like a VRAM surface (park_after_reset freed both copies).
        surface.quarantined = false;
        surface.evicted = false;
    } else if (surface.quarantined) {
        // Its retirement was unproven; the reset proof released the backend's
        // allocations (park_after_reset). The handle was already dead and the
        // session's share already returned: only the device accounting remains.
        used_surface_bytes_ -= surface.bytes;
        const uint32_t generation = surface.generation;
        surface = Surface{};
        surface.generation = generation;
        saturating_increment(counters_.released_quarantine);
        return true;
    }

    if (surface.lost) {
        return true; // Lost earlier (eviction); still answered once.
    }

    if (surface.evicted && !surface.uncertain) {
        // System memory survives a mode-1 reset, and no write to it can still
        // be pending: the contents are exact. Back to VRAM where possible.
        EngineSurface moved{};
        const BackendStatus status = backend_.surface_restore(backend_.context, surface.allocation, moved);
        if (status == BackendStatus::uncertain) {
            return false;
        }

        if (status == BackendStatus::ok && valid_surface(moved) && same_geometry(moved, surface.engine)) {
            surface.engine = moved;
            surface.evicted = false;
        }

        surface.last_use = 0;
        surface.last_write = 0;
        saturating_increment(counters_.reset_restored);
        return true;
    }

    // VRAM contents are lost on a mode-1 reset (the backend already returned
    // the VRAM); a system-memory surface with an uncertain write is not exact.
    if (surface.evicted) {
        backend_.surface_destroy(backend_.context, surface.allocation);
    }

    Session& owner = sessions_[surface.owner];
    used_surface_bytes_ -= surface.bytes;
    owner.surface_bytes -= surface.bytes;
    surface.bytes = 0;
    surface.allocation = 0;
    surface.lost = true;
    surface.evicted = false;
    surface.uncertain = false;
    saturating_increment(counters_.reset_lost);
    return true;
}

bool Dispatcher::complete_recovery(const Blit2dBackend& backend, const EngineSurface& primary) noexcept {
    if (!configured_ || !recovering_ || disabled_ || !complete(backend) || !valid_surface(primary)) {
        return false;
    }

    backend_ = backend;
    Surface& slot = surfaces_[0];
    if (!same_geometry(slot.engine, primary)) {
        slot.generation = next_generation(slot.generation);
        primary_owner_ = abi::max_sessions;
    }

    // The primary is miniport-owned scanout memory: its handle survives, its
    // VRAM contents do not (the owner of the display repaints it).
    slot.engine = primary;
    slot.last_use = 0;
    slot.last_write = 0;
    for (uint32_t index = 1; index < abi::max_surfaces; ++index) {
        Surface& surface = surfaces_[index];
        if (surface.used && !surface.primary && surface.owner < abi::max_sessions &&
            !recover_surface(surface)) {
            return false;
        }
    }

    for (Session& session : sessions_) {
        if (session.quarantined && !session.open) {
            // park_after_reset released its staging; the slot may be reused.
            session.quarantined = false;
            session.staging = {};
            saturating_increment(counters_.released_quarantine);
            continue;
        }

        if (session.open) {
            // Nothing issued before the reset can still execute: every fence
            // the session holds is retired, and its uncertainty became the
            // per-surface losses above.
            session.revived = true;
            session.poisoned = false;
            saturating_increment(counters_.revived_sessions);
        }
    }

    recovering_ = false;
    suspended_ = false;
    ++recovery_generation_;
    saturating_increment(counters_.recoveries);
    return true;
}

void Dispatcher::disable(bool quiescent) noexcept {
    recovering_ = false;
    disabled_ = true;
    // A later quiescent report never upgrades an unproven one.
    if (recovery_state_ != abi::recovery_disabled_unproven) {
        recovery_state_ = quiescent ? abi::recovery_disabled_quiescent : abi::recovery_disabled_unproven;
    }
}

uint32_t Dispatcher::next_key() noexcept {
    // xorshift32: unpredictable enough to stop accidental cross-PDEV reuse of
    // stale handles; the generation check is the actual lifetime guarantee.
    uint32_t value = key_state_;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    key_state_ = value != 0 ? value : 0x9e3779b9U;
    return key_state_;
}

uint64_t Dispatcher::completed() const noexcept {
    return backend_.completed(backend_.context);
}

uint32_t Dispatcher::session_index(const Session& session) const noexcept {
    for (uint32_t index = 0; index < abi::max_sessions; ++index) {
        if (&sessions_[index] == &session) {
            return index;
        }
    }

    return abi::max_sessions;
}

Dispatcher::Session* Dispatcher::authorize(const abi::Header& header) noexcept {
    uint32_t index = 0;
    if (!handle_slot(header.session, abi::max_sessions, index)) {
        return nullptr;
    }

    Session& session = sessions_[index];
    if (!session.open || session.quarantined ||
        session.generation != handle_generation(header.session) ||
        session.key != header.session_key) {
        return nullptr;
    }

    return &session;
}

Dispatcher::Surface* Dispatcher::resolve(uint32_t handle, const Session& session) noexcept {
    uint32_t index = 0;
    if (!handle_slot(handle, abi::max_surfaces, index)) {
        return nullptr;
    }

    Surface& surface = surfaces_[index];
    const uint32_t owner = session_index(session);
    if (!surface.used || surface.quarantined || surface.generation != handle_generation(handle)) {
        return nullptr;
    }

    if (surface.primary ? primary_owner_ != owner : surface.owner != owner) {
        return nullptr;
    }

    return &surface;
}

// Every rectangle inside the surface and pairwise disjoint (clip lists are).
bool Dispatcher::convert_rects(const abi::RectList& list, const EngineSurface& surface) noexcept {
    if (list.count == 0 || list.count > abi::max_rects) {
        return false;
    }

    for (uint32_t index = 0; index < list.count; ++index) {
        const abi::Rect& rect = list.rects[index];
        if (!rect_inside(rect, surface.width, surface.height)) {
            return false;
        }

        for (uint32_t previous = 0; previous < index; ++previous) {
            if (rects_intersect(rect, list.rects[previous])) {
                return false;
            }
        }

        engine_rects_[index] = engine_rect(rect);
    }

    return true;
}

// Order copy_input_ into copy_rects_ so that no rectangle's destination
// overwrites a source still to be read by a later rectangle. Self-overlap of a
// single rectangle is the backend's memmove contract. Returns false when no
// such order exists (a cyclic dependency); the request is then declined.
bool Dispatcher::order_copies(bool aliased) noexcept {
    const uint32_t count = copy_command_.rects.count;
    if (!aliased) {
        for (uint32_t index = 0; index < count; ++index) {
            copy_rects_[index] = copy_input_[index];
        }

        return true;
    }

    uint64_t remaining = count == 64 ? UINT64_MAX : ((uint64_t{1} << count) - 1U);
    uint32_t emitted = 0;
    while (remaining != 0) {
        bool progress = false;
        for (uint32_t candidate = 0; candidate < count; ++candidate) {
            if ((remaining & (uint64_t{1} << candidate)) == 0) {
                continue;
            }

            const EngineRect& destination = copy_input_[candidate].destination;
            bool blocked = false;
            for (uint32_t other = 0; other < count && !blocked; ++other) {
                if (other == candidate || (remaining & (uint64_t{1} << other)) == 0) {
                    continue;
                }

                const CopyRect& later = copy_input_[other];
                blocked = engine_intersect(destination, later.source_x, later.source_y,
                                           later.destination.width, later.destination.height);
            }

            if (!blocked) {
                copy_rects_[emitted++] = copy_input_[candidate];
                remaining &= ~(uint64_t{1} << candidate);
                progress = true;
            }
        }

        if (!progress) {
            return false;
        }
    }

    return emitted == count;
}

// ok with a valid monotonic fence records it; rejected declines without
// effect; uncertain (or a malformed fence from the backend) poisons the session.
bool Dispatcher::accept_fence(Session& session, BackendStatus status, uint64_t fence) noexcept {
    if (status == BackendStatus::ok && fence != 0 && fence >= last_fence_) {
        last_fence_ = fence;
        session.last_fence = fence;
        return true;
    }

    if (status != BackendStatus::rejected) {
        session.poisoned = true;
    }

    return false;
}

bool Dispatcher::retire_session(Session& session) noexcept {
    const uint32_t owner = session_index(session);
    if (primary_owner_ == owner) {
        primary_owner_ = abi::max_sessions;
    }

    const bool idle = session.last_fence == 0 || session.last_fence <= completed() ||
        backend_.sync(backend_.context, session.last_fence) == BackendStatus::ok;
    for (uint32_t index = 1; index < abi::max_surfaces; ++index) {
        Surface& surface = surfaces_[index];
        if (!surface.used || surface.primary || surface.owner != owner) {
            continue;
        }

        if (idle && !surface.quarantined) {
            release_surface(surface, session);
        } else {
            surface.quarantined = true; // Retained: the GPU may still access it.
        }
    }

    if (idle) {
        backend_.staging_destroy(backend_.context, session.staging);
        session.staging = {};
    } else {
        session.quarantined = true;
    }

    session.open = false;
    session.poisoned = false;
    session.surfaces = 0;
    session.surface_bytes = 0;
    session.key = 0;
    return idle;
}

// The caller proved every fence that referenced the surface retired.
void Dispatcher::release_surface(Surface& surface, Session& owner) noexcept {
    if (surface.allocation != 0 && !surface.lost) {
        backend_.surface_destroy(backend_.context, surface.allocation);
    }

    used_surface_bytes_ -= surface.bytes;
    owner.surface_bytes -= surface.bytes;
    --owner.surfaces;
    const uint32_t generation = surface.generation;
    surface = Surface{};
    surface.generation = generation;
}

uint32_t Dispatcher::surface_index(const Surface& surface) const noexcept {
    for (uint32_t index = 0; index < abi::max_surfaces; ++index) {
        if (&surfaces_[index] == &surface) {
            return index;
        }
    }

    return abi::max_surfaces;
}

Dispatcher::Surface* Dispatcher::lost_operand(Surface* first, Surface* second, Surface* third) noexcept {
    if (first != nullptr && first->lost) {
        return first;
    }

    if (second != nullptr && second->lost) {
        return second;
    }

    return third != nullptr && third->lost ? third : nullptr;
}

// Exactly once: the receipt names the handle, which is retired with it (its
// generation advances on reuse, so it can never alias a later allocation).
DispatchResult Dispatcher::lost_receipt(uint32_t capacity, Surface& surface) noexcept {
    const uint32_t index = surface_index(surface);
    if (index == 0 || index >= abi::max_surfaces || surface.owner >= abi::max_sessions) {
        return reject(DispatchStatus::invalid_parameter);
    }

    const uint32_t handle = make_handle(index, surface.generation);
    const DispatchResult result = receipt(capacity, abi::outcome_surface_lost, 0, handle);
    if (result.status == DispatchStatus::success) {
        release_surface(surface, sessions_[surface.owner]);
        saturating_increment(counters_.lost_reported);
    }

    return result;
}

DispatchResult Dispatcher::reject(DispatchStatus status) noexcept {
    saturating_increment(counters_.rejected);
    reply_bytes_ = 0;
    return {status, 0};
}

DispatchResult Dispatcher::receipt(uint32_t capacity, uint32_t outcome, uint64_t fence,
                                   uint32_t lost_surface) noexcept {
    abi::Receipt value{};
    value.outcome = outcome;
    value.lost_surface = lost_surface;
    value.fence = fence;
    value.completed_fence = configured_ ? completed() : 0;
    if (capacity < abi::receipt_bytes ||
        !abi::encode_receipt(sdma::CommandBuffer(reply_, abi::receipt_bytes), value)) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (outcome == abi::outcome_completed) {
        saturating_increment(counters_.accepted);
    } else {
        saturating_increment(counters_.rejected);
    }

    reply_bytes_ = abi::receipt_bytes;
    return {DispatchStatus::success, abi::receipt_bytes};
}

DispatchResult Dispatcher::dispatch(uint32_t code, ByteView input, uint32_t capacity) noexcept {
    reply_bytes_ = 0;
    if (!abi::is_radeon_2d_ioctl(code)) {
        return {DispatchStatus::unknown_code, 0};
    }

    if (code == abi::ioctl_query) {
        return query(input, capacity);
    }

    if (!configured_ || suspended_ || recovering_ || disabled_) {
        return reject(DispatchStatus::not_ready);
    }

    if (abi::is_raster_ioctl(code)) {
        return raster(code, input, capacity);
    }

    switch (code) {
    case abi::ioctl_open_session: return open_session(input, capacity);
    case abi::ioctl_close_session: return close_session(input, capacity);
    case abi::ioctl_assert_primary: return assert_primary(input, capacity);
    case abi::ioctl_fill: return fill(input, capacity);
    case abi::ioctl_copy: return copy(input, capacity);
    case abi::ioctl_upload: return transfer(input, capacity, true);
    case abi::ioctl_readback: return transfer(input, capacity, false);
    case abi::ioctl_surface_create: return surface_create(input, capacity);
    case abi::ioctl_surface_destroy: return surface_destroy(input, capacity);
    case abi::ioctl_sync: return sync(input, capacity);
    default: return {DispatchStatus::unknown_code, 0};
    }
}

DispatchResult Dispatcher::query(ByteView input, uint32_t capacity) noexcept {
    if (input.size() != 0) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::capabilities_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    abi::Capabilities value{};
    value.max_rects = abi::max_rects;
    value.max_sessions = abi::max_sessions;
    value.max_session_surfaces = abi::max_session_surfaces;
    value.max_surface_dimension = abi::max_surface_dimension;
    value.operations = abi::implemented_operations;
    if (configured_) {
        const EngineSurface& primary = surfaces_[0].engine;
        value.operations |= backend_.raster_operations;
        value.flags = suspended_ || recovering_ || disabled_ ? 0U : session_capabilities |
            (backend_.raster_operations != 0 ? abi::capability_raster : 0U);
        value.session_surface_bytes = session_surface_bytes_;
        value.device_surface_bytes = device_surface_bytes_;
        value.used_surface_bytes = used_surface_bytes_;
        value.primary_width = primary.width;
        value.primary_height = primary.height;
        value.primary_pitch = primary.pitch_bytes;
        value.primary_format = primary.format;
        value.staging_bytes = staging_bytes_;
        for (const Session& session : sessions_) {
            value.open_sessions += session.open ? 1U : 0U;
        }
    }

    value.accepted_requests = counters_.accepted;
    value.rejected_requests = counters_.rejected;
    value.recovery_generation = recovery_generation_;
    value.recovery_state = recovering_ && !disabled_ ? abi::recovery_pending : recovery_state_;
    value.reset_lost_surfaces = counters_.reset_lost;
    value.reset_restored_surfaces = counters_.reset_restored;
    if (!abi::encode_capabilities(sdma::CommandBuffer(reply_, abi::capabilities_bytes), value)) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    reply_bytes_ = abi::capabilities_bytes;
    return {DispatchStatus::success, abi::capabilities_bytes};
}

DispatchResult Dispatcher::open_session(ByteView input, uint32_t capacity) noexcept {
    if (!abi::decode_open_session(input)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::session_grant_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    Session* free_slot = nullptr;
    for (Session& session : sessions_) {
        if (!session.open && !session.quarantined) {
            free_slot = &session;
            break;
        }
    }

    if (free_slot == nullptr) {
        return reject(DispatchStatus::busy);
    }

    StagingBuffer staging{};
    if (backend_.staging_create(backend_.context, staging_bytes_, staging) != BackendStatus::ok) {
        return reject(DispatchStatus::busy);
    }

    if (staging.bytes != staging_bytes_ || staging.cpu_address == 0 || staging.gpu_address == 0 ||
        (staging.gpu_address & 3U) != 0 || (staging.cpu_address & 3U) != 0) {
        // Never handed out, never referenced by GPU work: safe to release.
        backend_.staging_destroy(backend_.context, staging);
        return reject(DispatchStatus::busy);
    }

    Session& session = *free_slot;
    session.open = true;
    session.poisoned = false;
    session.generation = next_generation(session.generation);
    session.key = next_key();
    session.surfaces = 0;
    session.surface_bytes = 0;
    session.last_fence = 0;
    session.staging = staging;

    const uint32_t index = session_index(session);
    const EngineSurface& primary = surfaces_[0].engine;
    abi::SessionGrant grant{};
    grant.session = {make_handle(index, session.generation), session.key};
    grant.primary_surface = make_handle(0, surfaces_[0].generation);
    grant.primary_width = primary.width;
    grant.primary_height = primary.height;
    grant.primary_pitch = primary.pitch_bytes;
    grant.primary_format = primary.format;
    grant.staging_bytes = staging.bytes;
    grant.flags = session_capabilities | (backend_.raster_operations != 0 ? abi::capability_raster : 0U);
    grant.staging_address = staging.cpu_address;
    if (!abi::encode_session_grant(sdma::CommandBuffer(reply_, abi::session_grant_bytes), grant)) {
        retire_session(session);
        return reject(DispatchStatus::insufficient_buffer);
    }

    saturating_increment(counters_.accepted);
    reply_bytes_ = abi::session_grant_bytes;
    return {DispatchStatus::success, abi::session_grant_bytes};
}

DispatchResult Dispatcher::close_session(ByteView input, uint32_t) noexcept {
    abi::Header header{};
    if (!abi::decode_close_session(input, header)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(header);
    if (session == nullptr) {
        return reject(DispatchStatus::invalid_parameter);
    }

    // The handle is invalid afterwards either way; unproven retirement keeps the
    // session's staging and surfaces quarantined instead of freeing them.
    if (retire_session(*session)) {
        saturating_increment(counters_.accepted);
    } else {
        saturating_increment(counters_.rejected);
    }

    return {DispatchStatus::success, 0};
}

DispatchResult Dispatcher::assert_primary(ByteView input, uint32_t capacity) noexcept {
    abi::AssertPrimaryCommand command{};
    if (!abi::decode_assert_primary(input, command)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(command.session);
    if (session == nullptr) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    const uint32_t owner = session_index(*session);
    if (session->poisoned) {
        if (primary_owner_ == owner) {
            primary_owner_ = abi::max_sessions;
        }

        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (command.enable != 0) {
        if (primary_owner_ != abi::max_sessions && primary_owner_ != owner) {
            return receipt(capacity, abi::outcome_declined, 0);
        }

        primary_owner_ = owner;
        return receipt(capacity, abi::outcome_completed, 0);
    }

    if (primary_owner_ != owner) {
        return receipt(capacity, abi::outcome_completed, 0);
    }

    // Releasing scanout: prove this session's queued writes have retired before
    // another owner (or the console) relies on the primary contents.
    primary_owner_ = abi::max_sessions;
    if (session->last_fence > completed() &&
        backend_.sync(backend_.context, session->last_fence) != BackendStatus::ok) {
        session->poisoned = true;
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    return receipt(capacity, abi::outcome_completed, session->last_fence);
}

DispatchResult Dispatcher::fill(ByteView input, uint32_t capacity) noexcept {
    abi::FillCommand& command = fill_command_;
    command = {};
    if (!abi::decode_fill(input, command)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(command.session);
    if (session == nullptr) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Surface* target = resolve(command.surface, *session);
    if (target == nullptr || !convert_rects(command.rects, target->engine)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (target->lost) {
        return lost_receipt(capacity, *target);
    }

    uint64_t fence = 0;
    const BackendStatus status = backend_.fill(backend_.context, target->engine, command.color,
                                               engine_rects_, command.rects.count, fence);
    if (!accept_fence(*session, status, fence)) {
        return receipt(capacity, session->poisoned ? abi::outcome_lost : abi::outcome_declined, 0);
    }

    target->last_use = fence;
    target->last_write = fence;
    return receipt(capacity, abi::outcome_completed, fence);
}

DispatchResult Dispatcher::copy(ByteView input, uint32_t capacity) noexcept {
    abi::CopyCommand& command = copy_command_;
    command = {};
    if (!abi::decode_copy(input, command)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(command.session);
    if (session == nullptr) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Surface* target = resolve(command.target, *session);
    Surface* source = resolve(command.source, *session);
    if (target == nullptr || source == nullptr || !convert_rects(command.rects, target->engine)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    for (uint32_t index = 0; index < command.rects.count; ++index) {
        const abi::Rect& rect = command.rects.rects[index];
        const int64_t left = static_cast<int64_t>(rect.left) + command.source_dx;
        const int64_t top = static_cast<int64_t>(rect.top) + command.source_dy;
        const int64_t right = static_cast<int64_t>(rect.right) + command.source_dx;
        const int64_t bottom = static_cast<int64_t>(rect.bottom) + command.source_dy;
        if (left < 0 || top < 0 || right > source->engine.width || bottom > source->engine.height) {
            return reject(DispatchStatus::invalid_parameter);
        }

        copy_input_[index] = {engine_rects_[index], static_cast<uint32_t>(left),
                              static_cast<uint32_t>(top)};
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (Surface* lost = lost_operand(target, source, nullptr)) {
        return lost_receipt(capacity, *lost);
    }

    if (!order_copies(target == source)) {
        return receipt(capacity, abi::outcome_declined, 0);
    }

    uint64_t fence = 0;
    const BackendStatus status = backend_.copy(backend_.context, target->engine, source->engine,
                                               copy_rects_, command.rects.count, fence);
    if (!accept_fence(*session, status, fence)) {
        return receipt(capacity, session->poisoned ? abi::outcome_lost : abi::outcome_declined, 0);
    }

    target->last_use = fence;
    target->last_write = fence;
    source->last_use = fence;
    return receipt(capacity, abi::outcome_completed, fence);
}

DispatchResult Dispatcher::transfer(ByteView input, uint32_t capacity, bool upload) noexcept {
    abi::TransferCommand& command = transfer_command_;
    command = {};
    if (!abi::decode_transfer(input, command)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(command.session);
    if (session == nullptr) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Surface* surface = resolve(command.surface, *session);
    const uint32_t pitch = command.staging_pitch;
    if (surface == nullptr || pitch == 0 || (pitch & 3U) != 0 || pitch > abi::max_staging_pitch ||
        (command.staging_offset & 3U) != 0 || !convert_rects(command.rects, surface->engine)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    const uint64_t staging_bytes = session->staging.bytes;
    for (uint32_t index = 0; index < command.rects.count; ++index) {
        const abi::Rect& rect = command.rects.rects[index];
        const int64_t column = static_cast<int64_t>(rect.left) - command.origin_x;
        const int64_t row = static_cast<int64_t>(rect.top) - command.origin_y;
        const EngineRect& area = engine_rects_[index];
        if (column < 0 || row < 0) {
            return reject(DispatchStatus::invalid_parameter);
        }

        // All terms are bounded (column, row < 2^33; pitch <= 2^16), so the
        // 64-bit products and sums cannot overflow.
        const uint64_t row_bytes = static_cast<uint64_t>(area.width) * abi::bytes_per_pixel;
        const uint64_t first = command.staging_offset + static_cast<uint64_t>(row) * pitch +
                               static_cast<uint64_t>(column) * abi::bytes_per_pixel;
        const uint64_t last_row = first + static_cast<uint64_t>(area.height - 1U) * pitch;
        if (static_cast<uint64_t>(column) * abi::bytes_per_pixel + row_bytes > pitch ||
            first > staging_bytes || last_row > staging_bytes ||
            row_bytes > staging_bytes - last_row) {
            return reject(DispatchStatus::invalid_parameter);
        }

        transfer_rects_[index] = {area, static_cast<uint32_t>(first)};
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (surface->lost) {
        return lost_receipt(capacity, *surface);
    }

    uint64_t fence = 0;
    const BackendStatus status = upload ?
        backend_.upload(backend_.context, surface->engine, session->staging, pitch,
                        transfer_rects_, command.rects.count, fence) :
        backend_.readback(backend_.context, surface->engine, session->staging, pitch,
                          transfer_rects_, command.rects.count, fence);
    if (!accept_fence(*session, status, fence)) {
        return receipt(capacity, session->poisoned ? abi::outcome_lost : abi::outcome_declined, 0);
    }

    surface->last_use = fence;
    if (upload) {
        surface->last_write = fence;
    }

    if (!upload && backend_.sync(backend_.context, fence) != BackendStatus::ok) {
        // The CPU must never consume staging bytes the GPU may still write.
        session->poisoned = true;
        return receipt(capacity, abi::outcome_lost, fence);
    }

    return receipt(capacity, abi::outcome_completed, fence);
}

DispatchResult Dispatcher::surface_create(ByteView input, uint32_t capacity) noexcept {
    abi::SurfaceCreateCommand command{};
    if (!abi::decode_surface_create(input, command)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(command.session);
    if (session == nullptr || session->poisoned || command.format != abi::format_xrgb8888 ||
        command.width == 0 || command.height == 0 || command.width > abi::max_surface_dimension ||
        command.height > abi::max_surface_dimension) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::surface_grant_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    Surface* slot = nullptr;
    uint32_t slot_index = 0;
    for (uint32_t index = 1; index < abi::max_surfaces && session->surfaces < abi::max_session_surfaces;
         ++index) {
        if (!surfaces_[index].used && !surfaces_[index].quarantined) {
            slot = &surfaces_[index];
            slot_index = index;
            break;
        }
    }

    // Budget pre-check on the smallest possible layout (pitch = width * 4); the
    // exact pitch the backend chose is checked again below.
    const uint64_t minimum = static_cast<uint64_t>(command.width) * abi::bytes_per_pixel * command.height;
    if (slot == nullptr || minimum > session_surface_bytes_ - session->surface_bytes ||
        minimum > device_surface_bytes_ - used_surface_bytes_) {
        return reject(DispatchStatus::busy);
    }

    EngineSurface engine{};
    uint32_t allocation = 0;
    const BackendStatus status = backend_.surface_create(backend_.context, command.width,
        command.height, command.format, engine, allocation);
    if (status == BackendStatus::uncertain) {
        session->poisoned = true;
    }

    if (status != BackendStatus::ok) {
        return reject(DispatchStatus::busy);
    }

    const uint64_t bytes = valid_surface(engine) ? surface_bytes(engine) : 0;
    if (!valid_surface(engine) || engine.width != command.width || engine.height != command.height ||
        bytes > session_surface_bytes_ - session->surface_bytes ||
        bytes > device_surface_bytes_ - used_surface_bytes_) {
        backend_.surface_destroy(backend_.context, allocation); // Never referenced by GPU work.
        return reject(DispatchStatus::busy);
    }

    const uint32_t generation = slot->generation;
    *slot = Surface{};
    slot->used = true;
    slot->generation = next_generation(generation);
    slot->owner = session_index(*session);
    slot->allocation = allocation;
    slot->bytes = bytes;
    slot->engine = engine;
    ++session->surfaces;
    session->surface_bytes += bytes;
    used_surface_bytes_ += bytes;

    abi::SurfaceGrant grant{};
    grant.surface = make_handle(slot_index, slot->generation);
    grant.width = engine.width;
    grant.height = engine.height;
    grant.pitch = engine.pitch_bytes;
    grant.format = engine.format;
    abi::encode_surface_grant(sdma::CommandBuffer(reply_, abi::surface_grant_bytes), grant);
    saturating_increment(counters_.accepted);
    reply_bytes_ = abi::surface_grant_bytes;
    return {DispatchStatus::success, abi::surface_grant_bytes};
}

DispatchResult Dispatcher::surface_destroy(ByteView input, uint32_t capacity) noexcept {
    abi::SurfaceDestroyCommand command{};
    if (!abi::decode_surface_destroy(input, command)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(command.session);
    Surface* surface = session == nullptr ? nullptr : resolve(command.surface, *session);
    if (surface == nullptr || surface->primary) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (surface->lost) {
        return lost_receipt(capacity, *surface);
    }

    const uint64_t last_use = surface->last_use;
    if (last_use > completed() &&
        backend_.sync(backend_.context, last_use) != BackendStatus::ok) {
        // Handle is dead; storage stays accounted to the device until a reset.
        surface->quarantined = true;
        surface->destroyed = true;
        session->surface_bytes -= surface->bytes;
        --session->surfaces;
        session->poisoned = true;
        return receipt(capacity, abi::outcome_lost, last_use);
    }

    release_surface(*surface, *session);
    return receipt(capacity, abi::outcome_completed, last_use);
}

DispatchResult Dispatcher::sync(ByteView input, uint32_t capacity) noexcept {
    abi::SyncCommand command{};
    if (!abi::decode_sync(input, command)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    Session* session = authorize(command.session);
    if (session == nullptr || command.fence > session->last_fence) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    const uint64_t target = command.fence == 0 ? session->last_fence : command.fence;
    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, target);
    }

    if (target != 0 && target > completed() &&
        backend_.sync(backend_.context, target) != BackendStatus::ok) {
        session->poisoned = true;
        return receipt(capacity, abi::outcome_lost, target);
    }

    return receipt(capacity, abi::outcome_completed, target);
}

// ---------------------------------------------------------------------------
// Compute raster requests (abi version 3).
// ---------------------------------------------------------------------------
DispatchResult Dispatcher::raster(uint32_t code, ByteView input, uint32_t capacity) noexcept {
    if ((backend_.raster_operations & abi::operation_bit(code)) == 0) {
        return {DispatchStatus::unknown_code, 0};
    }

    switch (code) {
    case abi::ioctl_rop:
    case abi::ioctl_pattern:
    case abi::ioctl_stretch:
        return raster_rop(code, input, capacity);
    case abi::ioctl_text: return raster_text(input, capacity);
    case abi::ioctl_alpha: return raster_alpha(input, capacity);
    case abi::ioctl_line: return raster_line(input, capacity);
    case abi::ioctl_gradient: return raster_gradient(input, capacity);
    case abi::ioctl_stretch_rop: return raster_stretch_rop(input, capacity);
    default: return {DispatchStatus::unknown_code, 0};
    }
}

bool Dispatcher::raster_target(const abi::Header& header, uint32_t handle, const abi::RectList& rects,
                               Session*& session, Surface*& target) noexcept {
    session = authorize(header);
    target = session == nullptr ? nullptr : resolve(handle, *session);
    return target != nullptr && convert_rects(rects, target->engine);
}

bool Dispatcher::resolve_operand(const abi::Operand& wire, const Session& session, RasterOperand& output,
                                 Surface*& surface) noexcept {
    output = {};
    surface = nullptr;
    if (wire.kind == abi::operand_none) {
        return true;
    }

    if (wire.kind == abi::operand_surface) {
        surface = resolve(wire.handle, session);
        if (surface == nullptr) {
            return false;
        }

        output.present = true;
        output.surface = surface->engine;
        return true;
    }

    // Staging: the whole descriptor span (a translated format's table, then
    // pitch * height) inside the session's own allocation; the decoder bounded
    // every factor.
    const uint64_t table = abi::staging_table_bytes(wire.format);
    const uint64_t span = table + static_cast<uint64_t>(wire.pitch) * wire.height;
    const StagingBuffer& staging = session.staging;
    if (wire.kind != abi::operand_staging || wire.handle > staging.bytes ||
        span > static_cast<uint64_t>(staging.bytes) - wire.handle) {
        return false;
    }

    output.present = true;
    output.surface = {staging.gpu_address + wire.handle + table, wire.width, wire.height, wire.pitch, wire.format};
    output.table_address = table != 0 ? staging.gpu_address + wire.handle : 0;
    return true;
}

DispatchResult Dispatcher::raster_receipt(uint32_t capacity, Session& session, Surface& target, Surface* other,
                                          BackendStatus status, uint64_t fence, Surface* mask) noexcept {
    if (!accept_fence(session, status, fence)) {
        return receipt(capacity, session.poisoned ? abi::outcome_lost : abi::outcome_declined, 0);
    }

    target.last_use = fence;
    target.last_write = fence;
    if (other != nullptr) {
        other->last_use = fence;
    }

    // A surface mask is read by the same work: destroy must wait for it too.
    if (mask != nullptr) {
        mask->last_use = fence;
    }

    return receipt(capacity, abi::outcome_completed, fence);
}

DispatchResult Dispatcher::raster_rop(uint32_t code, ByteView input, uint32_t capacity) noexcept {
    abi::RopCommand& command = rop_command_;
    command = {};
    Session* session = nullptr;
    Surface* target = nullptr;
    Surface* source = nullptr;
    Surface* mask = nullptr;
    RasterRop operation{};
    if (!abi::decode_rop(input, code == abi::ioctl_pattern, command) ||
        !raster_target(command.session, command.target, command.rects, session, target) ||
        !resolve_operand(command.source, *session, operation.source, source) ||
        !resolve_operand(command.mask, *session, operation.mask, mask)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    const bool has_source = operation.source.present;
    const int32_t bounds_width = command.bounds.right - command.bounds.left;
    const int32_t bounds_height = command.bounds.bottom - command.bounds.top;
    const bool identity = command.map.width == bounds_width && command.map.height == bounds_height;
    // 0xa10/0xa11 are 1:1 (or sourceless); 0xa14 always maps a source.
    const bool masked = operation.mask.present;
    if (rop4_uses_source(command.rop4) != has_source ||
        (!masked && (command.rop4 & 0xffU) != ((command.rop4 >> 8) & 0xffU)) ||
        (code == abi::ioctl_stretch ? !has_source : has_source && !identity) ||
        (!has_source && (command.map.width != 1 || command.map.height != 1 || command.map.left != 0 ||
                         command.map.top != 0))) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (Surface* lost = lost_operand(target, source, mask)) {
        return lost_receipt(capacity, *lost);
    }

    // The compute engine reads the source while it writes the target: a
    // same-surface operation whose mapped source span meets the bounds has no
    // safe execution order and is declined (the client renders it otherwise).
    if (source == target && rect_overlap(command.bounds, command.map.left, command.map.top,
                                         static_cast<int64_t>(command.map.left) + command.map.width,
                                         static_cast<int64_t>(command.map.top) + command.map.height)) {
        return receipt(capacity, abi::outcome_declined, 0);
    }

    operation.target = target->engine;
    operation.bounds = command.bounds;
    operation.rop4 = command.rop4;
    operation.flags = command.flags;
    operation.brush_colour = command.brush_colour;
    operation.brush_x = command.brush_x;
    operation.brush_y = command.brush_y;
    operation.mono_foreground = command.mono_foreground;
    operation.mono_background = command.mono_background;
    operation.colour_key = command.colour_key;
    operation.map = command.map;
    operation.mask_x = command.mask_x;
    operation.mask_y = command.mask_y;
    operation.pattern = command.has_pattern ? command.pattern : nullptr;
    uint64_t fence = 0;
    const BackendStatus status = backend_.raster_rop(backend_.context, operation, engine_rects_,
                                                     command.rects.count, fence);
    return raster_receipt(capacity, *session, *target, source, status, fence, mask);
}

DispatchResult Dispatcher::raster_text(ByteView input, uint32_t capacity) noexcept {
    abi::TextCommand& command = text_command_;
    command = {};
    Session* session = nullptr;
    Surface* target = nullptr;
    Surface* unused = nullptr;
    RasterOperand mask{};
    if (!abi::decode_text(input, command) ||
        !raster_target(command.session, command.target, command.rects, session, target) ||
        !resolve_operand(command.mask, *session, mask, unused)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (Surface* lost = lost_operand(target, unused, nullptr)) {
        return lost_receipt(capacity, *lost);
    }

    // The mask pixel of bounds.left/top is (mask_x, mask_y); one mask pixel per
    // destination pixel.
    const abi::SourceMap map{command.mask_x, command.mask_y, command.bounds.right - command.bounds.left,
                             command.bounds.bottom - command.bounds.top};
    uint64_t fence = 0;
    BackendStatus status = BackendStatus::rejected;
    if (command.mode == abi::text_mode_mono) {
        // eng/bitblt.c ROP4_MASKPAINT: mask bit 1 paints the solid brush
        // (PATCOPY), bit 0 keeps the destination.
        RasterRop operation{};
        operation.target = target->engine;
        operation.bounds = command.bounds;
        operation.rop4 = 0xaaf0U;
        operation.brush_colour = command.colour;
        operation.mask = mask;
        operation.mask_x = command.mask_x;
        operation.mask_y = command.mask_y;
        operation.map = {0, 0, 1, 1};
        status = backend_.raster_rop(backend_.context, operation, engine_rects_, command.rects.count, fence);
    } else {
        RasterAlpha operation{};
        operation.target = target->engine;
        operation.bounds = command.bounds;
        operation.source = mask;
        operation.map = map;
        operation.coverage8 = true;
        operation.colour = command.colour;
        status = backend_.raster_alpha(backend_.context, operation, engine_rects_, command.rects.count, fence);
    }

    return raster_receipt(capacity, *session, *target, unused, status, fence);
}

DispatchResult Dispatcher::raster_alpha(ByteView input, uint32_t capacity) noexcept {
    abi::AlphaCommand& command = alpha_command_;
    command = {};
    Session* session = nullptr;
    Surface* target = nullptr;
    Surface* source = nullptr;
    RasterAlpha operation{};
    if (!abi::decode_alpha(input, command) ||
        !raster_target(command.session, command.target, command.rects, session, target) ||
        !resolve_operand(command.source, *session, operation.source, source)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (Surface* lost = lost_operand(target, source, nullptr)) {
        return lost_receipt(capacity, *lost);
    }

    if (source == target && rect_overlap(command.bounds, command.map.left, command.map.top,
                                         static_cast<int64_t>(command.map.left) + command.map.width,
                                         static_cast<int64_t>(command.map.top) + command.map.height)) {
        return receipt(capacity, abi::outcome_declined, 0);
    }

    operation.target = target->engine;
    operation.bounds = command.bounds;
    operation.map = command.map;
    operation.constant_alpha = command.constant_alpha;
    operation.source_alpha = (command.flags & abi::alpha_flag_source_alpha) != 0;
    uint64_t fence = 0;
    const BackendStatus status = backend_.raster_alpha(backend_.context, operation, engine_rects_,
                                                       command.rects.count, fence);
    return raster_receipt(capacity, *session, *target, source, status, fence);
}

DispatchResult Dispatcher::raster_line(ByteView input, uint32_t capacity) noexcept {
    abi::LineCommand& command = line_command_;
    command = {};
    Session* session = nullptr;
    Surface* target = nullptr;
    if (!abi::decode_line(input, command) ||
        !raster_target(command.session, command.target, command.rects, session, target)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (target->lost) {
        return lost_receipt(capacity, *target);
    }

    RasterLine operation{};
    operation.target = target->engine;
    operation.x0 = command.x0;
    operation.y0 = command.y0;
    operation.x1 = command.x1;
    operation.y1 = command.y1;
    operation.colour = command.colour;
    operation.rop3 = command.rop3;
    operation.styled = (command.flags & abi::line_flag_styled) != 0;
    operation.style_bit_count = command.style_bit_count;
    operation.style = command.style;
    if ((command.flags & abi::line_flag_pattern) != 0) {
        operation.pattern = command.pattern;
        operation.brush_x = command.brush_x;
        operation.brush_y = command.brush_y;
    }

    uint64_t fence = 0;
    const BackendStatus status = backend_.raster_line(backend_.context, operation, engine_rects_,
                                                      command.rects.count, fence);
    return raster_receipt(capacity, *session, *target, nullptr, status, fence);
}

DispatchResult Dispatcher::raster_gradient(ByteView input, uint32_t capacity) noexcept {
    abi::GradientCommand& command = gradient_command_;
    command = {};
    Session* session = nullptr;
    Surface* target = nullptr;
    if (!abi::decode_gradient(input, command) ||
        !raster_target(command.session, command.target, command.rects, session, target)) {
        return reject(DispatchStatus::invalid_parameter);
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (target->lost) {
        return lost_receipt(capacity, *target);
    }

    RasterGradient operation{};
    operation.target = target->engine;
    operation.mode = command.mode;
    operation.extents = command.extents;
    for (uint32_t index = 0; index < 3; ++index) {
        operation.vertices[index] = command.vertices[index];
    }

    uint64_t fence = 0;
    const BackendStatus status = backend_.raster_gradient(backend_.context, operation, engine_rects_,
                                                          command.rects.count, fence);
    return raster_receipt(capacity, *session, *target, nullptr, status, fence);
}

} // namespace rx7900::reactos::radeon2d

#if defined(RX7900_RADEON2D_VIDEOPORT)
namespace rx7900::reactos::radeon2d {
namespace {

VP_STATUS video_status(DispatchStatus status) noexcept {
    switch (status) {
    case DispatchStatus::success: return NO_ERROR;
    case DispatchStatus::insufficient_buffer: return ERROR_INSUFFICIENT_BUFFER;
    case DispatchStatus::not_ready: return ERROR_DEV_NOT_EXIST;
    case DispatchStatus::busy: return ERROR_BUSY;
    case DispatchStatus::unknown_code: return ERROR_INVALID_FUNCTION;
    case DispatchStatus::invalid_parameter:
    default: return ERROR_INVALID_PARAMETER;
    }
}

class Guard final {
public:
    explicit Guard(KMUTEX& lock) noexcept : lock_(lock) {
        KeWaitForSingleObject(&lock_, Executive, KernelMode, FALSE, nullptr);
    }
    ~Guard() { KeReleaseMutex(&lock_, FALSE); }
    Guard(const Guard&) = delete;
    Guard& operator=(const Guard&) = delete;
private:
    KMUTEX& lock_;
};

uint32_t key_seed() noexcept {
    const LARGE_INTEGER counter = KeQueryPerformanceCounter(nullptr);
    return static_cast<uint32_t>(counter.QuadPart) ^ static_cast<uint32_t>(counter.QuadPart >> 32);
}

} // namespace

void native_initialize(NativeDispatcher& native) noexcept {
    if (native.initialized) {
        return;
    }

    KeInitializeMutex(&native.lock, 0);
    native.initialized = true;
}

bool native_configure(NativeDispatcher& native, const Blit2dBackend& backend,
                      const EngineSurface& primary, uint32_t staging_bytes) noexcept {
    if (!native.initialized || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    Guard guard(native.lock);
    return native.dispatcher.configure(backend, primary, staging_bytes, key_seed());
}

bool native_shutdown(NativeDispatcher& native) noexcept {
    if (!native.initialized || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    Guard guard(native.lock);
    return native.dispatcher.shutdown();
}

bool native_suspend(NativeDispatcher& native) noexcept {
    if (!native.initialized || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    Guard guard(native.lock);
    return native.dispatcher.suspend();
}

bool native_resume(NativeDispatcher& native, const Blit2dBackend& backend,
                   const EngineSurface& primary) noexcept {
    if (!native.initialized || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    Guard guard(native.lock);
    return native.dispatcher.resume(backend, primary);
}

VP_STATUS native_start_io(NativeDispatcher& native, VIDEO_REQUEST_PACKET& packet) noexcept {
    // Backend waits and the KMUTEX both require PASSIVE_LEVEL.
    if (!native.initialized || KeGetCurrentIrql() != PASSIVE_LEVEL || packet.StatusBlock == nullptr) {
        return ERROR_INVALID_FUNCTION;
    }

    packet.StatusBlock->Information = 0;
    Guard guard(native.lock);
    // METHOD_BUFFERED: InputBuffer and OutputBuffer alias one system buffer of
    // max(in, out) bytes owned by the I/O manager for this synchronous call.
    // The dispatcher decodes the complete input into device storage before the
    // reply below overwrites it; no pointer is retained after return.
    const ByteView input(static_cast<const uint8_t*>(packet.InputBuffer), packet.InputBufferLength);
    const uint32_t capacity = packet.OutputBuffer != nullptr ? packet.OutputBufferLength : 0;
    const DispatchResult result = native.dispatcher.dispatch(packet.IoControlCode, input, capacity);
    if (result.status != DispatchStatus::success) {
        return video_status(result.status);
    }

    if (result.reply_bytes != 0) {
        if (packet.OutputBuffer == nullptr || packet.OutputBufferLength < result.reply_bytes) {
            return ERROR_INSUFFICIENT_BUFFER; // Unreachable: capacity was checked first.
        }

        const ByteView reply = native.dispatcher.reply();
        auto* output = static_cast<uint8_t*>(packet.OutputBuffer);
        for (uint32_t index = 0; index < result.reply_bytes; ++index) {
            uint8_t value = 0;
            reply.read_u8(index, value);
            output[index] = value;
        }
    }

    packet.StatusBlock->Information = result.reply_bytes;
    return NO_ERROR;
}

} // namespace rx7900::reactos::radeon2d
#endif

namespace rx7900::reactos::radeon2d {

// One EngStretchBltROP DIB call (abi 0xa17). The window bounds every source
// sample the kernel may read, so it is also the same-surface hazard span.
DispatchResult Dispatcher::raster_stretch_rop(ByteView input, uint32_t capacity) noexcept {
    abi::StretchRopCommand& command = stretch_rop_command_;
    command = {};
    Session* session = nullptr;
    Surface* target = nullptr;
    Surface* source = nullptr;
    Surface* mask = nullptr;
    RasterRop operation{};
    if (!abi::decode_stretch_rop(input, command) ||
        !raster_target(command.session, command.target, command.rects, session, target) ||
        !resolve_operand(command.source, *session, operation.source, source) ||
        !resolve_operand(command.mask, *session, operation.mask, mask) ||
        rop4_uses_source(command.rop4) != operation.source.present) {
        return reject(DispatchStatus::invalid_parameter);
    }

    const abi::Rect& window = command.window;
    if (operation.source.present) {
        const EngineSurface& whole = operation.source.surface;
        const bool staged = command.source.kind == abi::operand_staging;
        const bool inside = window.left >= 0 && window.top >= 0 &&
            static_cast<int64_t>(window.right) <= whole.width && static_cast<int64_t>(window.bottom) <= whole.height;
        const bool entire = window.left == 0 && window.top == 0 &&
            static_cast<uint32_t>(window.right) == whole.width && static_cast<uint32_t>(window.bottom) == whole.height;
        if (!inside || (staged && !entire)) {
            return reject(DispatchStatus::invalid_parameter);
        }
    }

    if (capacity < abi::receipt_bytes) {
        return reject(DispatchStatus::insufficient_buffer);
    }

    if (session->poisoned) {
        return receipt(capacity, abi::outcome_lost, session->last_fence);
    }

    if (Surface* lost = lost_operand(target, source, mask)) {
        return lost_receipt(capacity, *lost);
    }

    if (source == target && rect_overlap(command.bounds, window.left, window.top, window.right, window.bottom)) {
        return receipt(capacity, abi::outcome_declined, 0);
    }

    if (operation.source.present) {
        // Crop the source to the window (validated inside it above): byte
        // offset top * pitch + left * 4 stays inside the operand's span.
        EngineSurface& cropped = operation.source.surface;
        cropped.gpu_address += static_cast<uint64_t>(window.top) * cropped.pitch_bytes +
                               static_cast<uint64_t>(window.left) * abi::bytes_per_pixel;
        cropped.width = static_cast<uint32_t>(window.right - window.left);
        cropped.height = static_cast<uint32_t>(window.bottom - window.top);
    }

    operation.target = target->engine;
    operation.bounds = command.bounds;
    operation.rop4 = command.rop4;
    // dib/stretchblt.c: a sample outside the source draws nothing under SRCCOPY.
    operation.flags = (command.rop4 & 0xffU) == 0xccU ? abi::rop_flag_skip_outside_source : 0U;
    operation.brush_colour = command.brush_colour;
    operation.brush_x = command.brush_x;
    operation.brush_y = command.brush_y;
    operation.mono_foreground = command.mono_foreground;
    operation.mono_background = command.mono_background;
    operation.map = {command.map.left - window.left, command.map.top - window.top, command.map.width,
                     command.map.height};
    operation.mask_x = command.mask_x + window.left;
    operation.mask_y = command.mask_y + window.top;
    operation.pattern = (command.flags & abi::stretch_flag_tile) != 0 ? command.pattern : nullptr;
    operation.mirror_x = (command.flags & abi::stretch_flag_mirror_x) != 0;
    operation.mirror_y = (command.flags & abi::stretch_flag_mirror_y) != 0;
    operation.stretch_mask = operation.mask.present;
    operation.running_phase = (command.flags & abi::stretch_flag_running_phase) != 0;
    uint64_t fence = 0;
    const BackendStatus status = backend_.raster_rop(backend_.context, operation, engine_rects_,
                                                     command.rects.count, fence);
    return raster_receipt(capacity, *session, *target, source, status, fence, mask);
}

} // namespace rx7900::reactos::radeon2d
