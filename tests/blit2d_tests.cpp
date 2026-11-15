/*
Module: rx7900.tests.blit2d
Purpose: Verify SDMA 6.0 2D packet bytes, field limits, overlap ordering, IB
recycling and failure handling against a pixel-exact CPU reference.
Created: 2026-10-22
Architecture: Host-only. A test SDMA execution model decodes the IBs the engine
submits (COPY_LINEAR_SUBWIN, CONSTANT_FILL, NOP), checks every field, flags any
memory conflict between packets that no NOP barrier separates (so results cannot
depend on unspecified intra- or inter-packet ordering) and applies them to
synthetic GPU memory. Never touches a GPU.
*/
#include <algorithm>
#include <array>
#include <cstdio>
#include <deque>
#include <initializer_list>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "blit2d.hpp"
#include "byte_view.hpp"

namespace {

using namespace rx7900::hardware;
namespace memory = rx7900::memory;
namespace sdma = rx7900::sdma;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

// ---------------------------------------------------------------- GPU memory

struct Shadow final {
    uint32_t epoch = 0;
    uint32_t writer = 0;
    uint32_t reader = 0;
};

constexpr uint32_t many_readers = UINT32_MAX;

struct Region final {
    uint64_t base = 0;
    std::vector<uint8_t> bytes;
    std::vector<Shadow> shadow;
    bool live = true;
    bool compare = false;
};

struct TestGpu final {
    std::vector<std::unique_ptr<Region>> regions;
    unsigned errors = 0;
    std::string first_error;
    uint32_t epoch = 1;
    uint32_t packet = 0;

    Region* add(uint64_t base, uint64_t bytes, bool compare) {
        auto region = std::make_unique<Region>();
        region->base = base;
        region->bytes.assign(static_cast<size_t>(bytes), 0);
        region->shadow.assign(static_cast<size_t>((bytes + 3) / 4), Shadow{});
        region->compare = compare;
        regions.push_back(std::move(region));
        return regions.back().get();
    }

    void error(const std::string& message) {
        if (errors++ == 0) {
            first_error = message;
        }
    }

    Region* find(uint64_t address, uint64_t bytes) {
        for (auto& region : regions) {
            if (region->live && address >= region->base &&
                address - region->base <= region->bytes.size() &&
                bytes <= region->bytes.size() - (address - region->base)) {
                return region.get();
            }
        }

        return nullptr;
    }

    bool read32(uint64_t address, uint32_t& value) {
        Region* region = find(address, 4);

        if (region == nullptr || (address & 3) != 0) {
            error("GPU read outside mapped memory");
            return false;
        }

        const rx7900::ByteView view(region->bytes.data(), region->bytes.size());
        return view.read_u32(static_cast<size_t>(address - region->base), value);
    }

    bool write32(uint64_t address, uint32_t value) {
        Region* region = find(address, 4);

        if (region == nullptr || (address & 3) != 0) {
            error("GPU write outside mapped memory");
            return false;
        }

        const size_t offset = static_cast<size_t>(address - region->base);

        for (size_t index = 0; index < 4; ++index) {
            region->bytes[offset + index] = static_cast<uint8_t>(value >> (index * 8));
        }

        return true;
    }

    Shadow* shadow(uint64_t address) {
        Region* region = find(address, 4);
        return region != nullptr ? &region->shadow[(address - region->base) / 4] : nullptr;
    }

    // Conflict rules between packets in one barrier group (see module header).
    void mark_read(uint64_t address) {
        Shadow* entry = shadow(address);

        if (entry == nullptr) {
            error("read of unmapped dword");
            return;
        }

        if (entry->epoch != epoch) {
            *entry = {epoch, 0, 0};
        }

        if (entry->writer != 0 && entry->writer != packet) {
            error("read-after-write without barrier");
        }

        entry->reader = entry->reader == 0 || entry->reader == packet ? packet : many_readers;
    }

    void mark_write(uint64_t address) {
        Shadow* entry = shadow(address);

        if (entry == nullptr) {
            error("write of unmapped dword");
            return;
        }

        if (entry->epoch != epoch) {
            *entry = {epoch, 0, 0};
        }

        if (entry->reader == packet) {
            error("packet overlaps its own source");
        } else if (entry->reader != 0) {
            error("write-after-read without barrier");
        }

        if (entry->writer != 0 && entry->writer != packet) {
            error("write-after-write without barrier");
        }

        entry->writer = packet;
    }
};

// ------------------------------------------------------- SDMA execution model

struct ModelStats final {
    unsigned ibs = 0;
    unsigned nops = 0;
    unsigned fills = 0;
    unsigned copies = 0;
    unsigned dwords = 0;
};

// Decodes exactly what blit2d may emit and rejects every other bit pattern.
void execute_ib(TestGpu& gpu, rx7900::ByteView ib, uint32_t dwords, ModelStats& stats) {
    ++stats.ibs;
    stats.dwords += dwords;

    if (dwords == 0 || dwords % 8 != 0 || dwords > blit2d_max_ib_dwords) {
        gpu.error("IB size violates SDMA IB rules");
        return;
    }

    uint32_t index = 0;
    const auto word = [&](uint32_t at) {
        uint32_t value = 0;

        if (at >= dwords || !ib.read_u32(size_t{at} * 4, value)) {
            gpu.error("packet runs past the IB end");
        }

        return value;
    };

    while (index < dwords && gpu.errors == 0) {
        const uint32_t header = word(index);
        const uint32_t op = header & 0xff;

        if (op == 0) {
            if (header != 0) {
                gpu.error("NOP with nonzero fields");
            }

            ++stats.nops;
            ++gpu.epoch;
            ++index;
            continue;
        }

        ++gpu.packet;

        if (header == 0x40000401u) {
            uint32_t dw[13]{};

            for (uint32_t at = 0; at < 13; ++at) {
                dw[at] = word(index + at);
            }

            const uint64_t src = dw[1] | (uint64_t{dw[2]} << 32);
            const uint64_t dst = dw[6] | (uint64_t{dw[7]} << 32);

            if ((dw[3] & 0xC000C000u) != 0 || (dw[8] & 0xC000C000u) != 0 ||
                (dw[4] & 0x1fffu) != 0 || (dw[9] & 0x1fffu) != 0 ||
                (dw[5] & 0xF0000000u) != 0 || (dw[10] & 0xF0000000u) != 0 ||
                (dw[11] & 0xC000C000u) != 0 || dw[12] != 0) {
                gpu.error("sub-window reserved, z, depth or policy bits set");
                return;
            }

            const uint32_t sx = dw[3] & 0x3fff;
            const uint32_t sy = dw[3] >> 16;
            const uint32_t sp = (dw[4] >> 13) + 1;
            const uint32_t ss = dw[5] + 1;
            const uint32_t dxp = dw[8] & 0x3fff;
            const uint32_t dyp = dw[8] >> 16;
            const uint32_t dp = (dw[9] >> 13) + 1;
            const uint32_t ds = dw[10] + 1;
            const uint32_t w = (dw[11] & 0x3fff) + 1;
            const uint32_t h = (dw[11] >> 16) + 1;

            if (sx + w > sp || dxp + w > dp || uint64_t{sp} * (sy + h) > ss ||
                uint64_t{dp} * (dyp + h) > ds || sp > blit2d_max_pitch_elements ||
                dp > blit2d_max_pitch_elements) {
                gpu.error("sub-window rectangle exceeds its pitch or slice");
                return;
            }

            std::vector<uint32_t> pixels(size_t{w} * h);

            for (uint32_t row = 0; row < h; ++row) {
                for (uint32_t column = 0; column < w; ++column) {
                    const uint64_t at = src + (uint64_t{sy + row} * sp + sx + column) * 4;
                    gpu.mark_read(at);
                    gpu.read32(at, pixels[size_t{row} * w + column]);
                }
            }

            for (uint32_t row = 0; row < h; ++row) {
                for (uint32_t column = 0; column < w; ++column) {
                    const uint64_t at = dst + (uint64_t{dyp + row} * dp + dxp + column) * 4;
                    gpu.mark_write(at);
                    gpu.write32(at, pixels[size_t{row} * w + column]);
                }
            }

            ++stats.copies;
            index += 13;
        } else if (header == 0x8000000Bu) {
            const uint64_t dst = word(index + 1) | (uint64_t{word(index + 2)} << 32);
            const uint32_t value = word(index + 3);
            const uint32_t count = word(index + 4);

            if ((count & 0xC0000000u) != 0 || ((count + 1) & 3) != 0 || (dst & 3) != 0 ||
                count + 1 > sdma::max_transfer_bytes) {
                gpu.error("constant fill count or address invalid");
                return;
            }

            for (uint64_t at = dst; at < dst + count + 1; at += 4) {
                gpu.mark_write(at);
                gpu.write32(at, value);
            }

            ++stats.fills;
            index += 5;
        } else {
            gpu.error("unexpected packet header");
            return;
        }
    }
}

// --------------------------------------------------------- Manager backend

struct TestBackend final {
    TestGpu* gpu = nullptr;
    bool non_cpu_visible_has_bytes = true;

    static memory::CreateResult create(void* context, sdma::GpuAllocation allocation,
                                       memory::Access, bool cpu_visible,
                                       memory::Backing& backing) noexcept {
        auto& self = *static_cast<TestBackend*>(context);
        Region* region = self.gpu->add(allocation.virtual_address, allocation.byte_length, false);
        backing = {region, cpu_visible ? region->bytes.data() : nullptr,
                   cpu_visible ? region->bytes.size() : 0};
        return memory::CreateResult::created;
    }

    static bool destroy(void*, const memory::Backing& backing, sdma::GpuAllocation) noexcept {
        static_cast<Region*>(backing.token)->live = false;
        return true;
    }

    memory::Backend backend() noexcept { return {this, create, destroy}; }
};

// ---------------------------------------------------------- Fake kernel ring

struct Pending final {
    memory::Handle ib;
    uint64_t offset;
    uint32_t dwords;
    uint32_t sequence;
    std::vector<uint8_t> snapshot;
};

struct FakeRing final {
    TestGpu* gpu = nullptr;
    memory::Manager* manager = nullptr;
    ModelStats stats{};
    std::deque<Pending> pending;
    uint32_t next_sequence = 100;
    uint32_t completed = 99;
    uint32_t outstanding_limit = fence_slots;
    bool complete_on_query = false;
    bool hang = false;
    bool fail_query = false;
    unsigned fail_submit_after = UINT32_MAX;
    unsigned submits = 0;
    unsigned busy_returns = 0;
    unsigned max_outstanding = 0;

    void run_until(uint32_t sequence) {
        while (!pending.empty() && pending.front().sequence <= sequence) {
            Pending& job = pending.front();
            memory::AllocationInfo info{};
            manager->describe(job.ib, info);

            const uint8_t* bytes = info.backing.cpu_bytes + job.offset;

            if (!std::equal(job.snapshot.begin(), job.snapshot.end(), bytes)) {
                gpu->error("IB modified while in flight");
            }

            execute_ib(*gpu, rx7900::ByteView(bytes, job.snapshot.size()),
                       job.dwords, stats);
            completed = job.sequence;
            pending.pop_front();
        }
    }

    static Blit2dSubmitResult submit(void* context, memory::Handle ib, uint64_t offset,
                                     uint32_t dwords, uint32_t& sequence) noexcept {
        auto& self = *static_cast<FakeRing*>(context);
        memory::AllocationInfo info{};

        if (self.submits >= self.fail_submit_after) {
            return Blit2dSubmitResult::failed;
        }

        if (self.pending.size() >= self.outstanding_limit) {
            ++self.busy_returns;
            return Blit2dSubmitResult::busy;
        }

        if (self.manager->describe(ib, info) != memory::Status::success ||
            (info.gpu.virtual_address + offset) % sdma6_ib_alignment != 0 ||
            dwords % sdma6_ib_dword_multiple != 0 || dwords == 0 ||
            offset > info.backing.cpu_byte_count ||
            uint64_t{dwords} * 4 > info.backing.cpu_byte_count - offset) {
            self.gpu->error("submitted IB violates Sdma6Submission rules");
            return Blit2dSubmitResult::failed;
        }

        ++self.submits;
        sequence = self.next_sequence++;
        const uint8_t* bytes = info.backing.cpu_bytes + offset;
        self.pending.push_back({ib, offset, dwords, sequence,
                                std::vector<uint8_t>(bytes, bytes + size_t{dwords} * 4)});
        self.max_outstanding = std::max<unsigned>(self.max_outstanding,
                                                  static_cast<unsigned>(self.pending.size()));
        return Blit2dSubmitResult::submitted;
    }

    static Blit2dFenceState query(void* context, uint32_t sequence) noexcept {
        auto& self = *static_cast<FakeRing*>(context);

        if (self.fail_query) {
            return Blit2dFenceState::failed;
        }

        if (self.complete_on_query && !self.hang) {
            self.run_until(sequence);
        }

        return sequence <= self.completed ? Blit2dFenceState::signalled : Blit2dFenceState::pending;
    }

    static Blit2dFenceState wait(void* context, uint32_t sequence, uint64_t) noexcept {
        auto& self = *static_cast<FakeRing*>(context);

        if (self.hang) {
            return Blit2dFenceState::pending;
        }

        self.run_until(sequence);
        return sequence <= self.completed ? Blit2dFenceState::signalled : Blit2dFenceState::failed;
    }

    Blit2dSubmitter submitter() noexcept { return {this, submit, query, wait}; }
};

// ------------------------------------------------------------- CPU reference

struct RefMemory final {
    TestGpu* gpu;
    std::vector<std::vector<uint8_t>> bytes;

    explicit RefMemory(TestGpu& source) : gpu(&source) {
        for (auto& region : source.regions) {
            bytes.push_back(region->bytes);
        }
    }

    std::vector<uint8_t>* locate(uint64_t address, size_t& offset) {
        for (size_t index = 0; index < gpu->regions.size(); ++index) {
            const auto& region = *gpu->regions[index];

            if (address >= region.base && address + 4 <= region.base + region.bytes.size()) {
                offset = static_cast<size_t>(address - region.base);
                return &bytes[index];
            }
        }

        return nullptr;
    }

    uint32_t read(uint64_t address) {
        size_t offset = 0;
        auto* data = locate(address, offset);
        return data == nullptr ? 0 : uint32_t{(*data)[offset]} | (uint32_t{(*data)[offset + 1]} << 8) |
               (uint32_t{(*data)[offset + 2]} << 16) | (uint32_t{(*data)[offset + 3]} << 24);
    }

    void write(uint64_t address, uint32_t value) {
        size_t offset = 0;
        auto* data = locate(address, offset);

        if (data != nullptr) {
            for (size_t index = 0; index < 4; ++index) {
                (*data)[offset + index] = static_cast<uint8_t>(value >> (index * 8));
            }
        }
    }

    bool matches() const {
        for (size_t index = 0; index < gpu->regions.size(); ++index) {
            if (gpu->regions[index]->compare && gpu->regions[index]->bytes != bytes[index]) {
                return false;
            }
        }

        return true;
    }
};

uint64_t pixel_address(const Blit2dSurface& surface, int64_t x, int64_t y) {
    return surface.gpu_address + static_cast<uint64_t>(y) * surface.pitch_bytes +
           static_cast<uint64_t>(x) * 4;
}

std::vector<Blit2dRect> effective_rects(const Blit2dRect& bounds, const Blit2dClipList& clip) {
    std::vector<Blit2dRect> rects;
    const auto add = [&](Blit2dRect rect) {
        if (rect.left < rect.right && rect.top < rect.bottom) {
            rects.push_back(rect);
        }
    };

    if (clip.count == 0) {
        add(bounds);
    }

    for (uint32_t index = 0; index < clip.count; ++index) {
        const auto& c = clip.rects[index];
        add({std::max(c.left, bounds.left), std::max(c.top, bounds.top),
             std::min(c.right, bounds.right), std::min(c.bottom, bounds.bottom)});
    }

    return rects;
}

void reference_fill(RefMemory& memory, const Blit2dFill& fill, const Blit2dClipList& clip) {
    for (const auto& rect : effective_rects(fill.bounds, clip)) {
        for (int32_t y = rect.top; y < rect.bottom; ++y) {
            for (int32_t x = rect.left; x < rect.right; ++x) {
                memory.write(pixel_address(fill.destination, x, y), fill.color);
            }
        }
    }
}

// GDI SRCCOPY: every destination pixel takes the source pixel as it was before
// the operation, whatever the overlap.
void reference_copy(RefMemory& memory, const Blit2dCopy& copy, const Blit2dClipList& clip) {
    const auto& source = copy.source;
    std::vector<uint32_t> snapshot(size_t{source.width} * source.height);

    for (uint32_t y = 0; y < source.height; ++y) {
        for (uint32_t x = 0; x < source.width; ++x) {
            snapshot[size_t{y} * source.width + x] = memory.read(pixel_address(source, x, y));
        }
    }

    const int64_t dx = int64_t{copy.destination_rect.left} - copy.source_point.x;
    const int64_t dy = int64_t{copy.destination_rect.top} - copy.source_point.y;

    for (const auto& rect : effective_rects(copy.destination_rect, clip)) {
        for (int32_t y = rect.top; y < rect.bottom; ++y) {
            for (int32_t x = rect.left; x < rect.right; ++x) {
                memory.write(pixel_address(copy.destination, x, y),
                             snapshot[size_t(y - dy) * source.width + size_t(x - dx)]);
            }
        }
    }
}

// ------------------------------------------------------------------ Fixture

constexpr uint64_t arena_base = UINT64_C(0x100000000);
constexpr uint64_t arena_bytes = UINT64_C(0x10000000);
constexpr uint64_t vram_base = UINT64_C(0x800000000);

struct Fixture final {
    TestGpu gpu;
    TestBackend backend;
    memory::Manager manager;
    FakeRing ring;
    Blit2dEngine engine;
    memory::Handle ib{};
    memory::Handle scratch{};
    uint64_t next_vram = vram_base;
    std::mt19937 random{1234};

    Fixture(uint32_t ib_count, uint64_t ib_bytes, uint64_t scratch_bytes) {
        backend.gpu = &gpu;
        manager.initialize({arena_base, arena_bytes}, backend.backend());
        ring.gpu = &gpu;
        ring.manager = &manager;

        const uint64_t ring_bytes = (ib_count * ib_bytes + 4095) & ~uint64_t{4095};
        manager.allocate(ring_bytes, 4096, memory::Access::read_write, true, ib);

        if (scratch_bytes != 0) {
            manager.allocate((scratch_bytes + 4095) & ~uint64_t{4095}, 4096,
                             memory::Access::read_write, false, scratch);
        }

        Blit2dEngineConfig config{};
        config.manager = &manager;
        config.ib_buffer = ib;
        config.ib_count = ib_count;
        config.ib_bytes = static_cast<uint32_t>(ib_bytes);

        config.scratch = scratch;
        config.submitter = ring.submitter();
        expect(engine.initialize(config) == Blit2dStatus::success, "engine initializes");
    }

    Blit2dSurface surface(uint32_t width, uint32_t height, uint32_t pitch_elements) {
        const Blit2dSurface result{next_vram, pitch_elements * 4, width, height};
        Region* region = gpu.add(next_vram, uint64_t{pitch_elements} * 4 * height, true);

        for (auto& byte : region->bytes) {
            byte = static_cast<uint8_t>(random());
        }

        next_vram += (uint64_t{pitch_elements} * 4 * height + 0xffff) & ~UINT64_C(0xffff);
        next_vram += 0x10000;
        return result;
    }

    bool clean() {
        if (gpu.errors != 0) {
            std::fprintf(stderr, "model: %s\n", gpu.first_error.c_str());
        }

        return gpu.errors == 0;
    }
};

Blit2dClipList no_clip() {
    Blit2dClipList clip{};
    clip.count = 0;
    return clip;
}

Blit2dClipList clip_of(std::initializer_list<Blit2dRect> rects) {
    Blit2dClipList clip{};

    for (const auto& rect : rects) {
        clip.rects[clip.count++] = rect;
    }

    return clip;
}

std::vector<uint32_t> ib_words(const uint8_t* bytes, size_t count) {
    std::vector<uint32_t> words;
    const rx7900::ByteView view(bytes, count * 4);

    for (size_t index = 0; index < count; ++index) {
        uint32_t value = 0;
        view.read_u32(index * 4, value);
        words.push_back(value);
    }

    return words;
}

// ------------------------------------------------------------------- Tests

void check_packet_encodings() {
    std::array<uint8_t, 64> bytes{};
    const sdma::CommandBuffer output(bytes.data(), bytes.size());
    const Blit2dSubWindow window{UINT64_C(0x123456780), 3, 5, 1920, 1920 * 1080,
                                 UINT64_C(0xABCDEF000), 16383, 16383, 16384, 16384 * 16384,
                                 16384, 7};
    expect(encode_sub_window_copy(window, output, 4) == Blit2dStatus::success,
           "sub-window copy at the field limits encodes");
    const auto words = ib_words(bytes.data() + 4, 13);
    const std::vector<uint32_t> expected{
        0x40000401u, 0x23456780u, 0x1u, 3u | (5u << 16), 1919u << 13, 1920u * 1080u - 1u,
        0xBCDEF000u, 0xAu, 16383u | (16383u << 16), 16383u << 13, 0x0FFFFFFFu,
        16383u | (6u << 16), 0u};
    expect(words == expected, "COPY_LINEAR_SUBWIN dwords follow sdma_v6_0_0_pkt_open.h");
    expect(bytes[0] == 0 && bytes[56] == 0, "sub-window copy leaves surrounding bytes");

    auto bad = window;
    bad.source_x = 0x4000;
    std::array<uint8_t, 64> before = bytes;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument &&
           bytes == before, "14-bit x refused without writing");
    bad = window;
    bad.destination_pitch_elements = blit2d_max_pitch_elements + 1;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument,
           "pitch above the Mesa 2^14 limit refused");
    bad = window;
    bad.source_pitch_elements = 0;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument,
           "zero pitch refused");
    bad = window;
    bad.source_slice_elements = blit2d_max_slice_elements + 1;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument,
           "28-bit slice pitch overflow refused");
    bad = window;
    bad.width = blit2d_max_extent + 1;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument,
           "14-bit rect width overflow refused");
    bad = window;
    bad.height = 0;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument,
           "empty rect refused");
    bad = window;
    bad.source_address += 2;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument,
           "unaligned element address refused");
    bad = window;
    bad.destination_address = sdma::gpu_virtual_address_max + 1;
    expect(encode_sub_window_copy(bad, output, 0) == Blit2dStatus::invalid_argument,
           "address above 48 bits refused");
    expect(encode_sub_window_copy(window, output, 64 - 51) == Blit2dStatus::invalid_argument,
           "sub-window needs 52 bytes of output");

    bytes.fill(0);
    expect(encode_constant_fill(UINT64_C(0x200000010), 0x00FF8040u, 7680, output, 0) ==
           Blit2dStatus::success, "constant fill encodes");
    const std::vector<uint32_t> fill{0x8000000Bu, 0x00000010u, 0x2u, 0x00FF8040u, 7679u};
    expect(ib_words(bytes.data(), 5) == fill, "CONSTANT_FILL uses dword fillsize and bytes - 1");
    expect(encode_constant_fill(0x1000, 0, 0, output, 0) == Blit2dStatus::invalid_argument,
           "zero fill refused");
    expect(encode_constant_fill(0x1000, 0, 6, output, 0) == Blit2dStatus::invalid_argument,
           "partial-dword fill refused");
    expect(encode_constant_fill(0x1002, 0, 8, output, 0) == Blit2dStatus::invalid_argument,
           "unaligned fill refused");
    expect(encode_constant_fill(0x1000, 0, sdma::max_transfer_bytes + 4, output, 0) ==
           Blit2dStatus::invalid_argument, "fill above the 4 MiB policy refused");
    expect(encode_constant_fill(0x1000, 0, sdma::max_transfer_bytes, output, 0) ==
           Blit2dStatus::success, "4 MiB fill accepted");
    expect(encode_constant_fill(0x1000, 0, 4, output, 60) == Blit2dStatus::invalid_argument,
           "fill needs 20 bytes of output");

    bytes.fill(0xff);
    expect(encode_nop(output, 60) == Blit2dStatus::success &&
           ib_words(bytes.data() + 60, 1)[0] == 0, "NOP is a zero dword");
    expect(encode_nop(output, 61) == Blit2dStatus::invalid_argument, "NOP bounds checked");
}

void check_hazard_tracker() {
    Blit2dHazardTracker tracker;
    const Blit2dRegion a{0x1000, 64, 16, {0, 0, 4, 4}};
    const Blit2dRegion b{0x1000, 64, 16, {4, 0, 8, 4}};
    const Blit2dRegion c{0x1000, 64, 16, {2, 2, 6, 6}};
    tracker.add(a, false);
    expect(!tracker.conflicts(&a, b), "read-read and disjoint write do not conflict");
    expect(tracker.conflicts(nullptr, c), "write over an earlier read conflicts (WAR)");
    tracker.add(b, true);
    expect(tracker.conflicts(&c, Blit2dRegion{0x9000, 64, 16, {0, 0, 1, 1}}),
           "read of an earlier write conflicts (RAW)");
    expect(tracker.conflicts(nullptr, Blit2dRegion{0x1000, 64, 16, {5, 1, 6, 2}}),
           "write over an earlier write conflicts (WAW)");
    const Blit2dRegion other_pitch{0x1000 + 64, 32, 16, {0, 0, 1, 1}};
    expect(tracker.conflicts(nullptr, other_pitch),
           "different layouts compare conservatively by byte span");
    const Blit2dRegion after{0x1000 + 4 * 64, 64, 16, {0, 0, 16, 1}};
    expect(!blit2d_regions_overlap(a, after) && blit2d_regions_overlap(c, c),
           "same-layout regions compare exactly");
    // A window cropped out of a 16-pixel-wide surface at (5, 3): base +3 rows
    // +5 pixels, same pitch. It meets surface pixel (5 + x, 3 + y) only.
    const Blit2dRegion surface{0x1000, 64, 16, {0, 0, 16, 16}};
    const Blit2dRegion window{0x1000 + 3 * 64 + 5 * 4, 64, 10, {0, 0, 4, 2}};
    expect(blit2d_regions_overlap(window, Blit2dRegion{0x1000, 64, 16, {8, 4, 9, 5}}) &&
               blit2d_regions_overlap(Blit2dRegion{0x1000, 64, 16, {5, 3, 6, 4}}, window) &&
               !blit2d_regions_overlap(window, Blit2dRegion{0x1000, 64, 16, {9, 3, 16, 5}}) &&
               !blit2d_regions_overlap(window, Blit2dRegion{0x1000, 64, 16, {0, 3, 5, 5}}) &&
               !blit2d_regions_overlap(window, Blit2dRegion{0x1000, 64, 16, {5, 5, 9, 9}}) &&
               blit2d_regions_overlap(window, surface),
           "a cropped window of one layout compares exactly by translated pixels");
    // A translated row that would wrap past the pitch stays conservative.
    const Blit2dRegion wrapping{0x1000 + 3 * 64 + 14 * 4, 64, 10, {0, 0, 4, 1}};
    expect(blit2d_regions_overlap(wrapping, Blit2dRegion{0x1000, 64, 16, {0, 4, 1, 5}}),
           "a translated region wider than the remaining row keeps the byte-span rule");
    tracker.clear();
    expect(tracker.size() == 0 && !tracker.conflicts(&c, c), "barrier clears hazards");
}

void check_initialize_and_shutdown() {
    TestGpu gpu;
    TestBackend backend{&gpu};
    memory::Manager manager;
    manager.initialize({arena_base, arena_bytes}, backend.backend());
    FakeRing ring;
    ring.gpu = &gpu;
    ring.manager = &manager;
    memory::Handle ib0{};
    memory::Handle hidden{};
    memory::Handle readonly{};
    manager.allocate(4096, 4096, memory::Access::read_write, true, ib0);
    manager.allocate(4096, 4096, memory::Access::read_write, false, hidden);
    manager.allocate(4096, 4096, memory::Access::read, true, readonly);

    Blit2dEngine engine;
    Blit2dEngineConfig config{};
    config.manager = &manager;
    config.submitter = ring.submitter();
    config.ib_buffer = ib0;
    config.ib_bytes = 1024;
    config.ib_count = 1;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument, "one IB slot refused");
    config.ib_count = 5;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument,
           "slots beyond the allocation refused");
    config.ib_count = 4;
    config.ib_bytes = 128;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument, "128-byte IB refused");
    config.ib_bytes = 1000;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument,
           "IB slot not a multiple of 32 bytes refused");
    config.ib_bytes = 1024;
    config.ib_buffer = hidden;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument,
           "IB without CPU backing refused");
    config.ib_buffer = ib0;
    config.scratch = readonly;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument,
           "scratch without GPU write refused");
    config.scratch = ib0;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument,
           "scratch aliasing an IB refused");
    config.scratch = {};
    config.submitter.wait = nullptr;
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument, "wait callback required");
    config.submitter = ring.submitter();

    memory::AllocationInfo info{};
    manager.describe(ib0, info);
    expect(info.leases == 0, "refused initialization retains nothing");
    expect(engine.fill({}, no_clip()) == Blit2dStatus::not_initialized, "fill before initialize");
    config.scratch = hidden;
    expect(engine.initialize(config) == Blit2dStatus::success, "valid configuration accepted");
    expect(engine.initialize(config) == Blit2dStatus::invalid_argument, "second initialize refused");
    manager.describe(ib0, info);
    expect(info.leases == 1, "IB lease retained");
    manager.describe(hidden, info);
    expect(info.leases == 1, "scratch lease retained");
    expect(manager.shutdown() == memory::Status::busy, "manager cannot shut down under leases");
    expect(engine.shutdown() == Blit2dStatus::success, "idle engine shuts down");
    manager.describe(ib0, info);
    expect(info.leases == 0, "shutdown returns IB leases");
    manager.describe(hidden, info);
    expect(info.leases == 0, "shutdown returns scratch lease");
}

void check_fill() {
    Fixture f(2, 4096, 0);
    const auto screen = f.surface(48, 40, 56);
    RefMemory reference(f.gpu);

    Blit2dFill fill{screen, {3, 4, 20, 9}, 0xAABBCCDDu};
    expect(f.engine.fill(fill, no_clip()) == Blit2dStatus::success, "unclipped fill accepted");
    reference_fill(reference, fill, no_clip());

    fill = {screen, {0, 0, 48, 40}, 0x01020304u};
    const auto clip = clip_of({{0, 0, 10, 2}, {12, 0, 30, 2}, {5, 20, 60, 25}, {7, 30, 7, 35}});
    expect(f.engine.fill(fill, clip) == Blit2dStatus::success, "clipped fill accepted");
    reference_fill(reference, fill, clip);
    expect(f.engine.synchronize() == Blit2dStatus::success, "fill synchronizes");
    expect(f.clean() && reference.matches(), "fill matches the CPU reference");
    expect(f.engine.report().fill_packets == 5 + 2 + 2 + 5, "one CONSTANT_FILL per row");

    const auto before = f.engine.report();
    expect(f.engine.fill({screen, {0, 0, 49, 1}, 0}, no_clip()) == Blit2dStatus::out_of_bounds,
           "fill outside the surface refused");
    expect(f.engine.fill({screen, {5, 5, 4, 6}, 0}, no_clip()) == Blit2dStatus::invalid_rect,
           "inverted bounds refused");
    expect(f.engine.fill({screen, {0, 0, 4, 4}, 0}, clip_of({{3, 3, 2, 4}})) ==
           Blit2dStatus::invalid_rect, "inverted clip rectangle refused");
    Blit2dClipList many{};
    many.count = blit2d_max_rects + 1;
    expect(f.engine.fill({screen, {0, 0, 4, 4}, 0}, many) == Blit2dStatus::too_many_rects,
           "more than 64 clip rectangles refused");
    many.count = blit2d_max_rects;

    for (uint32_t index = 0; index < blit2d_max_rects; ++index) {
        many.rects[index] = {int32_t(index % 8) * 6, int32_t(index / 8) * 5,
                             int32_t(index % 8) * 6 + 5, int32_t(index / 8) * 5 + 4};
    }

    Blit2dSurface bad = screen;
    bad.pitch_bytes = 47 * 4;
    expect(f.engine.fill({bad, {0, 0, 1, 1}, 0}, no_clip()) == Blit2dStatus::invalid_surface,
           "pitch below width refused");
    bad = screen;
    bad.gpu_address += 2;
    expect(f.engine.fill({bad, {0, 0, 1, 1}, 0}, no_clip()) == Blit2dStatus::invalid_surface,
           "unaligned surface refused");
    bad = screen;
    bad.width = blit2d_max_surface_dimension + 1;
    bad.pitch_bytes = bad.width * 4;
    expect(f.engine.fill({bad, {0, 0, 1, 1}, 0}, no_clip()) == Blit2dStatus::invalid_surface,
           "surface wider than 16384 refused");
    memory::AllocationInfo info{};
    f.manager.describe(f.ib, info);
    expect(f.engine.fill({{info.gpu.virtual_address, 64, 16, 4}, {0, 0, 1, 1}, 0}, no_clip()) ==
           Blit2dStatus::engine_memory_overlap, "fill into an IB buffer refused");
    expect(f.engine.report().fill_packets == before.fill_packets &&
           f.engine.report().open_dwords == before.open_dwords, "refusals encode nothing");

    expect(f.engine.fill({screen, {0, 0, 48, 40}, 0x55u}, many) == Blit2dStatus::success,
           "exactly 64 clip rectangles accepted");
    reference_fill(reference, {screen, {0, 0, 48, 40}, 0x55u}, many);
    expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
           "64-rectangle fill matches the reference");
}

void check_fill_strategy_numbers() {
    // Full-pitch rectangles are contiguous: one CONSTANT_FILL per 4 MiB of rows.
    Fixture f(4, 65536, 0);
    const auto screen = f.surface(1920, 1080, 1920);
    RefMemory reference(f.gpu);
    auto before = f.engine.report();
    expect(f.engine.fill({screen, {0, 0, 1920, 1080}, 0xFF336699u}, no_clip()) ==
           Blit2dStatus::success, "full-screen fill accepted");
    reference_fill(reference, {screen, {0, 0, 1920, 1080}, 0xFF336699u}, no_clip());
    auto after = f.engine.report();
    const uint64_t full_packets = after.fill_packets - before.fill_packets;
    expect(full_packets == 2, "1920x1080 full-pitch fill is two 4 MiB-bounded packets");

    before = after;
    expect(f.engine.fill({screen, {100, 100, 200, 200}, 0x12345678u}, no_clip()) ==
           Blit2dStatus::success, "100x100 fill accepted");
    reference_fill(reference, {screen, {100, 100, 200, 200}, 0x12345678u}, no_clip());
    after = f.engine.report();
    expect(after.fill_packets - before.fill_packets == 100 &&
           after.open_dwords - before.open_dwords ==
               500 + (after.barrier_nops - before.barrier_nops),
           "100x100 fill is 100 packets, 500 dwords plus its barrier");

    before = after;
    expect(f.engine.fill({screen, {10, 0, 11, 1080}, 0x1u}, no_clip()) == Blit2dStatus::success,
           "1-pixel vertical line accepted");
    reference_fill(reference, {screen, {10, 0, 11, 1080}, 0x1u}, no_clip());
    after = f.engine.report();
    expect(after.fill_packets - before.fill_packets == 1080, "vertical line is one fill per row");

    before = after;
    expect(f.engine.fill({screen, {0, 500, 1919, 1080}, 0x2u}, no_clip()) == Blit2dStatus::success,
           "almost-full-width fill accepted");
    reference_fill(reference, {screen, {0, 500, 1919, 1080}, 0x2u}, no_clip());
    after = f.engine.report();
    expect(after.fill_packets - before.fill_packets == 580, "narrower than pitch is per row");
    expect(after.barrier_nops == 2, "overlapping fills are separated by barriers");
    expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
           "large fills match the reference");
    std::printf("fill packets: 1920x1080 full pitch %llu; 100x100 100 (500 dwords); "
                "1x1080 1080 (5400 dwords); IBs %llu, dwords %llu\n",
                static_cast<unsigned long long>(full_packets),
                static_cast<unsigned long long>(f.engine.report().ib_submissions),
                static_cast<unsigned long long>(f.engine.report().submitted_dwords));
}

void check_copy_between_surfaces() {
    Fixture f(2, 4096, 0);
    const auto offscreen = f.surface(32, 24, 40);
    const auto screen = f.surface(48, 40, 48);
    RefMemory reference(f.gpu);
    const Blit2dCopy copy{offscreen, screen, {10, 12, 40, 30}, {1, 2}};
    const auto clip = clip_of({{0, 0, 48, 15}, {10, 20, 25, 40}, {30, 20, 48, 26}});
    expect(f.engine.copy(copy, clip) == Blit2dStatus::success, "offscreen-to-screen copy");
    reference_copy(reference, copy, clip);
    expect(f.engine.report().copy_packets == 3 && f.engine.report().barrier_nops == 0,
           "one sub-window packet per independent clip rectangle");
    expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
           "offscreen copy matches the reference");

    expect(f.engine.copy({offscreen, screen, {0, 0, 10, 10}, {25, 0}}, no_clip()) ==
           Blit2dStatus::out_of_bounds, "source outside its surface refused");
    expect(f.engine.copy({offscreen, screen, {40, 0, 50, 10}, {0, 0}}, no_clip()) ==
           Blit2dStatus::out_of_bounds, "destination outside its surface refused");
    Blit2dSurface alias = screen;
    alias.gpu_address += 64;
    alias.width = 16;
    alias.height = 8;
    expect(f.engine.copy({alias, screen, {0, 0, 4, 4}, {0, 0}}, no_clip()) ==
           Blit2dStatus::overlapping_surfaces, "partially aliasing descriptors refused");
}

void check_upload_readback() {
    Fixture f(2, 4096, 0);
    const auto screen = f.surface(64, 32, 64);
    memory::Handle staging{};
    memory::Handle readonly{};
    f.manager.allocate(65536, 4096, memory::Access::read_write, true, staging);
    f.manager.allocate(65536, 4096, memory::Access::read, true, readonly);
    memory::AllocationInfo info{};
    f.manager.describe(staging, info);

    for (size_t index = 0; index < 16 * 4 * 10 + 64; ++index) {
        info.backing.cpu_bytes[index] = static_cast<uint8_t>(index * 7 + 3);
    }

    // Find the staging Region so the reference sees the CPU-written pixels.
    for (auto& region : f.gpu.regions) {
        if (region->base == info.gpu.virtual_address) {
            region->compare = true;
        }
    }

    RefMemory reference(f.gpu);
    const Blit2dStaging upload{staging, 64, 16 * 4, 16, 10};
    const Blit2dSurface upload_surface{info.gpu.virtual_address + 64, 64, 16, 10};
    expect(f.engine.upload(upload, {0, 0}, screen, {5, 5, 21, 15}, no_clip()) ==
           Blit2dStatus::success, "upload from GTT staging");
    reference_copy(reference, {upload_surface, screen, {5, 5, 21, 15}, {0, 0}}, no_clip());

    const Blit2dStaging readback{staging, 32768, 20 * 4, 20, 12};
    const Blit2dSurface readback_surface{info.gpu.virtual_address + 32768, 80, 20, 12};
    expect(f.engine.readback(screen, {2, 3, 22, 15}, readback, {0, 0}) == Blit2dStatus::success,
           "readback into GTT staging");
    reference_copy(reference, {screen, readback_surface, {0, 0, 20, 12}, {2, 3}}, no_clip());
    expect(f.engine.report().barrier_nops == 1, "readback after an upload is barrier-separated");
    expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
           "upload and readback match the reference");

    uint32_t pixel = 0;
    const rx7900::ByteView cpu(info.backing.cpu_bytes, info.backing.cpu_byte_count);
    expect(cpu.read_u32(32768 + 3 * 80 + 3 * 4, pixel) &&
           pixel == reference.read(pixel_address(screen, 5, 6)), "CPU sees readback pixels");

    expect(f.engine.readback(screen, {0, 0, 4, 4}, {readonly, 0, 16, 4, 4}, {0, 0}) ==
           Blit2dStatus::invalid_staging, "readback into GPU-read-only staging refused");
    expect(f.engine.upload({staging, 65536 - 32, 64, 16, 4}, {0, 0}, screen, {0, 0, 4, 4},
                           no_clip()) == Blit2dStatus::invalid_staging,
           "staging rectangle beyond its allocation refused");
    expect(f.engine.upload({memory::Handle{}, 0, 64, 16, 4}, {0, 0}, screen, {0, 0, 4, 4},
                           no_clip()) == Blit2dStatus::invalid_staging, "stale staging refused");
    expect(f.engine.readback(screen, {0, 0, 4, 4}, {f.ib, 0, 16, 4, 4}, {0, 0}) ==
           Blit2dStatus::engine_memory_overlap, "readback into an IB refused");
}

// Every overlap direction, with and without scratch, plus a 1-row scroll.
void check_same_surface_overlap() {
    const int32_t moves[][2] = {{3, 0}, {-3, 0}, {0, 2}, {0, -2}, {2, 3}, {-2, 3},
                                {2, -3}, {-2, -3}, {1, 1}, {0, 1}, {1, 0}, {0, 0},
                                {-1, -1}, {7, 0}, {0, -9}, {9, 7}};

    for (const bool with_scratch : {false, true}) {
        for (const auto& move : moves) {
            Fixture f(2, 512, with_scratch ? 65536 : 0);
            const auto screen = f.surface(40, 36, 44);
            RefMemory reference(f.gpu);
            const int32_t dx = move[0];
            const int32_t dy = move[1];
            const Blit2dRect dst{10 + dx, 10 + dy, 30 + dx, 28 + dy};
            const Blit2dCopy copy{screen, screen, dst, {10, 10}};
            expect(f.engine.copy(copy, no_clip()) == Blit2dStatus::success,
                   "same-surface overlapping copy accepted");
            reference_copy(reference, copy, no_clip());
            expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() &&
                   reference.matches(), "overlapping copy matches the GDI snapshot result");
        }
    }

    for (const bool with_scratch : {false, true}) {
        Fixture f(3, 1024, with_scratch ? 1 << 20 : 0);
        const auto screen = f.surface(256, 192, 256);
        RefMemory reference(f.gpu);
        const Blit2dCopy scroll{screen, screen, {0, 0, 256, 191}, {0, 1}};
        expect(f.engine.copy(scroll, no_clip()) == Blit2dStatus::success, "1-row scroll accepted");
        reference_copy(reference, scroll, no_clip());
        const auto report = f.engine.report();
        expect(with_scratch ? report.copy_packets == 2 && report.bounced_rects == 1
                            : report.copy_packets == 191 && report.strip_rects == 1,
               "scroll uses two bounced packets or one strip per row");
        expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
               "scroll matches the reference");
        std::printf("1-row scroll 256x191: %s, %llu copy packets, %llu barriers, %llu IBs\n",
                    with_scratch ? "bounce" : "strips",
                    static_cast<unsigned long long>(report.copy_packets),
                    static_cast<unsigned long long>(f.engine.report().barrier_nops),
                    static_cast<unsigned long long>(f.engine.report().ib_submissions));
    }
}

Blit2dClipList banded_clip(std::mt19937& random, int32_t width, int32_t height) {
    Blit2dClipList clip{};
    int32_t y = static_cast<int32_t>(random() % 4) - 2;

    while (y < height && clip.count < 20) {
        const int32_t band = 1 + static_cast<int32_t>(random() % 9);
        int32_t x = static_cast<int32_t>(random() % 6) - 2;

        while (x < width && clip.count < 20) {
            const int32_t span = 1 + static_cast<int32_t>(random() % 14);
            clip.rects[clip.count++] = {x, y, x + span, y + band};
            x += span + 1 + static_cast<int32_t>(random() % 6);
        }

        y += band + static_cast<int32_t>(random() % 3);
    }

    return clip;
}

void check_random_banded_copies() {
    unsigned accepted = 0;

    for (unsigned iteration = 0; iteration < 300; ++iteration) {
        Fixture f(2 + iteration % 3, iteration % 2 ? 256 : 2048, iteration % 4 == 0 ? 0 : 1 << 16);
        f.random.seed(iteration);
        const bool same = iteration % 5 != 0;
        const auto screen = f.surface(37, 29, 41);
        const auto other = f.surface(33, 31, 33);
        RefMemory reference(f.gpu);
        const auto source = same ? screen : other;
        const int32_t width = 4 + static_cast<int32_t>(f.random() % 20);
        const int32_t height = 4 + static_cast<int32_t>(f.random() % 18);
        const int32_t sx = static_cast<int32_t>(f.random() % (source.width - width));
        const int32_t sy = static_cast<int32_t>(f.random() % (source.height - height));
        const int32_t dx = static_cast<int32_t>(f.random() % (screen.width - width));
        const int32_t dy = static_cast<int32_t>(f.random() % (screen.height - height));
        const Blit2dCopy copy{source, screen, {dx, dy, dx + width, dy + height}, {sx, sy}};
        const auto clip = banded_clip(f.random, static_cast<int32_t>(screen.width),
                                      static_cast<int32_t>(screen.height));
        const auto status = f.engine.copy(copy, clip);
        expect(status == Blit2dStatus::success, "banded clip copy accepted");

        if (status == Blit2dStatus::success) {
            ++accepted;
            reference_copy(reference, copy, clip);
            // Interleave a fill so later copies depend on earlier packets.
            const Blit2dFill fill{screen, {1, 1, 9, 6}, iteration};
            f.engine.fill(fill, no_clip());
            reference_fill(reference, fill, no_clip());
            const Blit2dCopy again{screen, screen, {0, 0, 12, 8}, {2, 1}};
            expect(f.engine.copy(again, no_clip()) == Blit2dStatus::success,
                   "dependent follow-up copy accepted");
            reference_copy(reference, again, no_clip());
        }

        expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
               "random banded copy matches the reference");
    }

    std::printf("random banded copies: %u accepted\n", accepted);
}

bool order_exists(const std::vector<Blit2dRect>& rects, int32_t dx, int32_t dy) {
    std::vector<size_t> order(rects.size());

    for (size_t index = 0; index < order.size(); ++index) {
        order[index] = index;
    }

    const auto hit = [](const Blit2dRect& a, const Blit2dRect& b) {
        return std::max(a.left, b.left) < std::min(a.right, b.right) &&
               std::max(a.top, b.top) < std::min(a.bottom, b.bottom);
    };

    do {
        bool valid = true;

        for (size_t i = 0; i < order.size() && valid; ++i) {
            for (size_t j = i + 1; j < order.size() && valid; ++j) {
                const auto& later = rects[order[j]];
                valid = !hit(rects[order[i]], {later.left - dx, later.top - dy,
                                               later.right - dx, later.bottom - dy});
            }
        }

        if (valid) {
            return true;
        }
    } while (std::next_permutation(order.begin(), order.end()));

    return false;
}

// Unbanded rectangle lists: success exactly when some processing order exists.
void check_unbanded_order() {
    unsigned refused = 0;
    unsigned accepted = 0;
    std::mt19937 random(77);

    for (unsigned iteration = 0; iteration < 600; ++iteration) {
        std::vector<Blit2dRect> rects;

        while (rects.size() < 2 + iteration % 3) {
            const int32_t left = 4 + static_cast<int32_t>(random() % 22);
            const int32_t top = 4 + static_cast<int32_t>(random() % 18);
            const Blit2dRect rect{left, top, left + 1 + static_cast<int32_t>(random() % 8),
                                  top + 1 + static_cast<int32_t>(random() % 8)};
            // Every fourth list may overlap: GDI never produces one, but only
            // overlapping lists can form a dependency cycle (no cycle among
            // disjoint rectangles turned up in 2e7 random trials).
            bool disjoint = true;

            for (const auto& other : rects) {
                disjoint = disjoint &&
                    (std::max(rect.left, other.left) >= std::min(rect.right, other.right) ||
                     std::max(rect.top, other.top) >= std::min(rect.bottom, other.bottom));
            }

            if (disjoint || iteration % 4 == 0) {
                rects.push_back(rect);
            }
        }

        const int32_t dx = static_cast<int32_t>(random() % 7) - 3;
        const int32_t dy = static_cast<int32_t>(random() % 7) - 3;
        Fixture f(2, 1024, iteration % 2 ? 1 << 16 : 0);
        const auto screen = f.surface(40, 36, 40);
        RefMemory reference(f.gpu);
        Blit2dClipList clip{};

        for (const auto& rect : rects) {
            clip.rects[clip.count++] = rect;
        }

        // Keep every mapped source inside the surface.
        const Blit2dCopy bounded{screen, screen, {4, 4, 34, 30}, {4 - dx, 4 - dy}};
        const auto status = f.engine.copy(bounded, clip);
        const bool possible = order_exists(rects, dx, dy);
        expect(status == (possible ? Blit2dStatus::success : Blit2dStatus::unordered_overlap),
               "unbanded order accepted exactly when a valid order exists");

        if (status == Blit2dStatus::success) {
            ++accepted;
            reference_copy(reference, bounded, clip);
        } else {
            ++refused;
            expect(f.engine.report().copy_packets == 0, "refused order encodes nothing");
        }

        expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
               "unbanded copy matches the reference");
    }

    std::printf("unbanded orders: %u accepted, %u refused (cyclic)\n", accepted, refused);
    expect(refused != 0 && accepted != 0, "random unbanded lists exercise both outcomes");
}

void check_ib_recycling_and_busy() {
    Fixture f(blit2d_max_ib_slots, 256, 0);
    f.ring.outstanding_limit = 4;
    const auto screen = f.surface(64, 200, 64 + 3);
    RefMemory reference(f.gpu);
    expect(f.engine.fill({screen, {0, 0, 64, 200}, 0xCAFEu}, no_clip()) == Blit2dStatus::success,
           "fill larger than all IBs together accepted");
    reference_fill(reference, {screen, {0, 0, 64, 200}, 0xCAFEu}, no_clip());
    const auto report = f.engine.report();
    expect(report.ib_submissions >= 15 && report.busy_retries != 0 && report.waits != 0,
           "IBs split, ring-full retried and the oldest IB awaited");
    expect(f.ring.max_outstanding <= 4, "never more than four fences outstanding");
    expect(f.engine.synchronize() == Blit2dStatus::success && f.clean() && reference.matches(),
           "split fill matches the reference");
    expect(f.engine.report().in_flight == 0 && f.engine.report().open_dwords == 0,
           "synchronize leaves nothing open or in flight");

    // After observed completion, a dependent copy needs no barrier.
    const auto barriers = f.engine.report().barrier_nops;
    expect(f.engine.copy({screen, screen, {0, 100, 64, 110}, {0, 0}}, no_clip()) ==
           Blit2dStatus::success, "copy after synchronize accepted");
    expect(f.engine.report().barrier_nops == barriers, "completion clears the hazard tracker");
    reference_copy(reference, {screen, screen, {0, 100, 64, 110}, {0, 0}}, no_clip());
    f.ring.complete_on_query = true;
    expect(f.engine.flush() == Blit2dStatus::success && f.engine.retire() == Blit2dStatus::success,
           "flush and retire");
    expect(f.engine.report().in_flight == 0, "retire recycles signalled IBs");
    expect(f.clean() && reference.matches(), "retired copy matches the reference");
    expect(f.engine.shutdown() == Blit2dStatus::success, "idle engine shuts down");
}

void check_failures() {
    {
        Fixture f(2, 256, 0);
        const auto screen = f.surface(32, 32, 32);
        f.ring.fail_submit_after = 0;
        f.engine.fill({screen, {0, 0, 32, 2}, 1}, no_clip());
        expect(f.engine.synchronize() == Blit2dStatus::faulted, "failed submit faults the engine");
        expect(f.engine.report().last_failure == Blit2dStatus::submit_failed,
               "fault reason is submit_failed");
        expect(f.engine.fill({screen, {0, 0, 1, 1}, 1}, no_clip()) == Blit2dStatus::faulted,
               "faulted engine refuses new work");
        expect(f.engine.shutdown() == Blit2dStatus::faulted, "faulted engine refuses shutdown");
        expect(f.engine.release_after_reset() == Blit2dStatus::success, "reset release");
        memory::AllocationInfo info{};
        f.manager.describe(f.ib, info);
        expect(info.leases == 0, "reset release returns leases");
    }
    {
        Fixture f(2, 256, 0);
        const auto screen = f.surface(32, 64, 33);
        f.ring.hang = true;
        expect(f.engine.fill({screen, {0, 0, 32, 64}, 1}, no_clip()) == Blit2dStatus::faulted,
               "hung ring faults when an IB must be recycled");
        expect(f.engine.report().last_failure == Blit2dStatus::fence_timeout,
               "fault reason is fence_timeout");
        expect(f.engine.release_after_reset() == Blit2dStatus::success, "release after hang");
    }
    {
        Fixture f(2, 256, 0);
        const auto screen = f.surface(32, 8, 32);
        f.engine.fill({screen, {0, 0, 8, 8}, 1}, no_clip());
        expect(f.engine.flush() == Blit2dStatus::success, "flush submits");
        expect(f.engine.shutdown() == Blit2dStatus::busy, "shutdown refused while in flight");
        f.ring.fail_query = true;
        expect(f.engine.retire() == Blit2dStatus::faulted &&
               f.engine.report().last_failure == Blit2dStatus::fence_failed,
               "failed fence query faults the engine");
        f.engine.release_after_reset();
    }
    {
        Fixture f(2, 256, 0);
        const auto screen = f.surface(32, 8, 32);
        f.ring.outstanding_limit = 0;
        f.engine.fill({screen, {0, 0, 8, 8}, 1}, no_clip());
        expect(f.engine.flush() == Blit2dStatus::faulted &&
               f.engine.report().last_failure == Blit2dStatus::submit_failed,
               "ring held by other submitters fails closed");
        f.engine.release_after_reset();
    }
}

} // namespace

int main() {
    check_packet_encodings();
    check_hazard_tracker();
    check_initialize_and_shutdown();
    check_fill();
    check_fill_strategy_numbers();
    check_copy_between_surfaces();
    check_upload_readback();
    check_same_surface_overlap();
    check_random_banded_copies();
    check_unbanded_order();
    check_ib_recycling_and_busy();
    check_failures();
    std::printf("%u blit2d checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
