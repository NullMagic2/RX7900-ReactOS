/*
Module: lightning.reactos.gdi_client
Purpose: Submit real GPU operations from the existing cached GDI surface owner.
Created: 2026-10-21
Architecture: Included after framebuf.h; registry guard owns all surface access.
*/
#pragma once
#include "lightning_gdi_abi.hpp"
#include "lightning_gdi_geometry.hpp"
#include "lightning_gdi_lifecycle.hpp"
#include "gdi_protocol.hpp"

namespace lightning::reactos::gdi {
enum class Attempt { unsupported, performed, failed };

struct ClipBatch final {
    Rectangle bounds{};
    LightningDisplayRect rectangles[LIGHTNING_GDI_MAX_RECTS]{};
    uint32_t count=0;
    struct Enumeration final {
        ULONG count=0;
        RECTL rectangles[LIGHTNING_GDI_MAX_RECTS]{};
    } enumeration{};
};

bool capture_clip(RECTL requested,CLIPOBJ* clip,uint32_t width,uint32_t height,
                  ClipBatch& output) noexcept;

// Borrowed native operands remain owned by GDI for this synchronous operation.
// Normalized bytes alone enter the native staging/transfer ABI.
struct RasterInput final {
    SURFOBJ* source=nullptr;
    XLATEOBJ* translation=nullptr;
    RECTL original{};
    POINTL* source_point=nullptr;
    bool same_canvas=false;
    bool stretch=false;
    RECTL source_rectangle{};
    // With stretch, mask is the DrvStretchBltROP 1bpp mask (engine source-texel
    // semantics, mask_point unused) and requires LIGHTNING_GDI_CAP_STRETCH_MASK.
    SURFOBJ* mask=nullptr;
    POINTL* mask_point=nullptr;
    bool coverage=false;
    BRUSHOBJ* brush=nullptr;
    POINTL* brush_origin=nullptr;
    uint32_t rop4=0;
    uint32_t raster_flags=0;
    uint32_t color_key=0;
    uint32_t constant_alpha=0;
};

class ClientCanvas final {
public:
    Attempt initialize(PPDEV device) noexcept;
    // Registry caller retained an authoritative cached baseline before the
    // owned mode transition. A newer, actually retired native canvas is the
    // only proof permitting a failed local bind to be discarded and retried.
    Attempt reinitialize_after_mode(PPDEV device) noexcept;
    // DrvAssertMode(TRUE) only, after the native owner accepted this PDEV's
    // own mode commit (which reset any native canvas) while the registry guard
    // holds the complete cached baseline retained at suspend. Stops GPU use
    // and promotion until the next mode reinitialization; nothing accepted is
    // replayed and the CPU publishes the authoritative cache.
    void demote_to_cpu() noexcept;
    bool enabled() const noexcept { return active_; }
    bool poisoned() const noexcept { return visibility_.poisoned(); }
    bool synchronize(SURFOBJ* cached,SURFOBJ* staging) noexcept;
    // GPU execution barrier only; cache visibility and logical groups persist.
    bool complete_gpu(SURFOBJ* cached,SURFOBJ* staging) noexcept;
    bool prepare(SURFOBJ* cached,SURFOBJ* staging) noexcept;
    bool upload_cached(Rectangle rectangle) noexcept;
    Attempt fill(const ClipBatch& clip,uint32_t color,SURFOBJ* cached,CLIPOBJ* native_clip) noexcept;
    Attempt copy(const ClipBatch& clip,uint32_t source_x,uint32_t source_y) noexcept;
    Attempt external_image(const ClipBatch& clip,uint32_t source_x,uint32_t source_y,
        const LightningGdiExternalImage& image) noexcept;
    // async-present: revision-2 present. The producer point limits the
    // native producer wait; async lets an accepted copy return before its
    // source read retires (pending reports that), otherwise this call waits
    // for the read itself. Requires async_external(); unsupported otherwise.
    struct ExternalOptions final {
        bool async=false;
        uint64_t producer_sync=0;
        uint64_t producer_value=0;
    };
    Attempt external_image(const ClipBatch& clip,uint32_t source_x,uint32_t source_y,
        const LightningGdiExternalImage& image,const ExternalOptions& options,bool& pending) noexcept;
    // The native stack (upper filter and miniport) pins asynchronous reads.
    // An older stack fails the private request and stays synchronous.
    bool async_external() noexcept;
    // Never waits for the GPU. False: no native answer available now.
    bool external_receipt(uint64_t lease,uint32_t& status) noexcept;
    Attempt raster(const ClipBatch& clip,uint32_t rop,uint32_t pattern,SURFOBJ* source,
                   XLATEOBJ* translation,RECTL original,POINTL* source_point,
                   bool same_canvas,SURFOBJ* staging) noexcept;
    Attempt raster_operands(const ClipBatch& clip,const RasterInput& input,SURFOBJ* staging) noexcept;
    Attempt glyph(const ClipBatch& clip,SURFOBJ* mask,RECTL original,POINTL point,
                  BRUSHOBJ* brush,SURFOBJ* staging) noexcept;
    // A successful queue owns a copied coverage snapshot until its text barrier.
    // Only disjoint same-color glyphs share a draw; counters require acceptance.
    Attempt queue_glyph(const ClipBatch& clip,SURFOBJ* mask,RECTL original,POINTL point,
                        BRUSHOBJ* brush,SURFOBJ* staging,SURFOBJ* cached) noexcept;
    bool flush_glyphs() noexcept;
    // Surface teardown only, after callbacks are excluded. These CPU bytes are
    // never leased to a backend beyond a synchronous accepted submit receipt.
    void release_cpu_storage() noexcept;
    Attempt bitmap(const ClipBatch& clip,SURFOBJ* source,XLATEOBJ* translation,
                   RECTL original,POINTL point,SURFOBJ* staging,SURFOBJ* cached,CLIPOBJ* native_clip) noexcept;
    Attempt blend(const ClipBatch& clip,SURFOBJ* source,XLATEOBJ* translation,
                  RECTL original,RECTL source_rectangle,BLENDOBJ* blend,SURFOBJ* staging,
                  bool same_canvas=false) noexcept;
    Attempt transparent(const ClipBatch& clip,SURFOBJ* source,XLATEOBJ* translation,
                        RECTL original,RECTL source_rectangle,uint32_t color_key,
                        bool same_canvas,SURFOBJ* staging) noexcept;
    // GDI owns captured stable vertex/mesh arrays for this synchronous call.
    // The complete mesh is checked before any gradient command is accepted.
    Attempt gradient(ClipBatch& workspace,CLIPOBJ* clip,XLATEOBJ* translation,
                     const TRIVERTEX* vertices,uint32_t vertex_count,const void* mesh,uint32_t mesh_count,
                     RECTL extents,POINTL origin,uint32_t mode) noexcept;
    bool statistics(Statistics& output) noexcept;
    bool begin_text() noexcept;
    bool end_text() noexcept;
    void abort_text() noexcept;
    void fallback() noexcept;
    bool retire() noexcept;
private:
    bool valid_information(const Information& information) const noexcept;
    bool owner_offline() const noexcept;
    Attempt bind(const Information& information) noexcept;
    Attempt submit(LightningGdiCommand command,const LightningDisplayRect* clips,
                   uint32_t count,uint32_t source_x=0,uint32_t source_y=0,
                   const LightningGdiRasterOperands* operands=nullptr,
                   const LightningGdiGradientOperands* gradient=nullptr,
                   const LightningGdiExternalImage* external=nullptr,
                   uint32_t abi_version=native_abi_version) noexcept;
    bool upload(Rectangle rectangle,const LightningDisplayRect* clips,uint32_t count,
                bool normalized) noexcept;
    bool wait() noexcept;
    bool ensure_group() noexcept;
    Attempt submit_external(const ClipBatch& clip,uint32_t source_x,uint32_t source_y,
        const LightningGdiExternalImage& image,uint32_t abi_version) noexcept;
    HANDLE device_=nullptr;
    // async-present: 1 once the native stack reported asynchronous pins.
    uint8_t async_probe_=0;
    // The last accepted external copy's read had not retired at its receipt.
    bool external_pending_=false;
    uint32_t generation_=0;
    uint32_t width_=0;
    uint32_t height_=0;
    uint32_t last_sequence_=0;
    uint32_t last_epoch_=0;
    uint32_t last_revision_=0;
    uint32_t reset_generation_=0;
    uint32_t capabilities_=0;
    void* staging_base_=nullptr;
    uint32_t staging_bytes_=0;
    bool active_=false;
    bool promotable_=false;
    bool text_scope_=false;
    uint32_t group_sequence_=0;
    PromotionSchedule promotion_{};
    CacheVisibility visibility_{};
    Statistics statistics_{};
    struct GlyphBatch final {
        uint8_t* pixels=nullptr;
        uint32_t bytes=0;
        uint32_t stride=0;
        Rectangle bounds{};
        Rectangle rectangles[LIGHTNING_GDI_MAX_RECTS]{};
        uint32_t rectangle_count=0;
        uint32_t glyph_count=0;
        uint32_t color=0;
    } glyphs_{};
    // The surface registry serializes submissions; no transfer snapshot lives
    // on the shallow kernel stack or survives a synchronous native receipt.
    struct Submission final {
        SubmitRequest request{};
        uint8_t tail[LIGHTNING_GDI_RASTER_BYTES+LIGHTNING_GDI_MAX_RECTS*sizeof(LightningDisplayRect)]{};
    } submission_{};
};
} // namespace lightning::reactos::gdi
