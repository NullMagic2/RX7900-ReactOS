/*
Module: rx7900.tests.radeon_2d_backend
Purpose: Drive the Radeon 2D IOCTL dispatcher end to end through Radeon2dBackend,
Blit2dEngine, Sdma6 kernel submission and FenceManager after the real AsicInit
bring-up, checking pixels against a CPU reference.
Created: 2026-10-22
Architecture: Host-only. Reuses the shared synthetic Navi31 (asic_test_device.hpp)
and MES model (mes_test_model.hpp) read-only. The shared SDMA model executes only
the bring-up packets, so this file adds a local SDMA execution layer in front of
it: SDMA doorbells run the ring here, decoding the frame exactly as the shared
model does and the IB as blit2d_tests.cpp does (NOP, WRITE_LINEAR,
CONSTANT_FILL, COPY_LINEAR_SUBWIN). It is not hardware evidence.
*/
#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include "asic_test_device.hpp"
#include "gfx11_interp.hpp"
#include "mes_test_model.hpp"
#include "ucode_test_files.hpp"
#include "radeon_2d_backend.hpp"
#include "radeon_display.hpp"
#include "gdi_engine_reference.hpp"
#include "atom_asic_init_rom.hpp"

namespace {

using namespace rx7900::hardware;
using namespace rx7900_test::ucode_files;
using rx7900_test::SimulatedNavi31;
namespace radeon2d = rx7900::reactos::radeon2d;
namespace abi = rx7900::reactos::radeon2d::abi;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// ------------------------------------------------------------ bring-up fixture
// Same inputs as tests/asic_init_tests.cpp (that file's helpers are local to it).

std::vector<uint8_t> sos_container() {
    const std::array<uint32_t, 6> identifiers{2, 1, 4, 3, 5, 6};
    constexpr uint32_t part_bytes = 64;
    std::vector<uint8_t> bytes(256 + identifiers.size() * part_bytes);
    const rx7900::sdma::CommandBuffer output(bytes.data(), bytes.size());
    output.write_u32(0, static_cast<uint32_t>(bytes.size()));
    output.write_u32(4, 36);
    bytes.at(8) = 2;
    bytes.at(12) = 13;
    output.write_u32(16, 0x310035);
    output.write_u32(20, static_cast<uint32_t>(identifiers.size() * part_bytes));
    output.write_u32(24, 256);
    output.write_u32(32, static_cast<uint32_t>(identifiers.size()));

    for (size_t index = 0; index < identifiers.size(); ++index) {
        output.write_u32(36 + index * 16, identifiers.at(index));
        output.write_u32(40 + index * 16, 0x310035);
        output.write_u32(44 + index * 16, static_cast<uint32_t>(index * part_bytes));
        output.write_u32(48 + index * 16, part_bytes);

        for (uint32_t word = 0; word < part_bytes / 4; ++word) {
            output.write_u32(256 + index * part_bytes + word * 4,
                             0x50000000u + identifiers.at(index) * 0x100 + word);
        }
    }

    return bytes;
}

// FencePlatform on the host: time advances by every block timeout.
struct HostFences final {
    uint64_t now = 0;
    static void lock(void*) noexcept {}
    static void unlock(void*) noexcept {}
    static uint64_t now_us(void* context) noexcept { return static_cast<HostFences*>(context)->now += 1000; }
    static void signal(void*, uint32_t) noexcept {}
    static void reset(void*, uint32_t) noexcept {}
    static bool block(void* context, uint32_t, uint64_t timeout) noexcept {
        static_cast<HostFences*>(context)->now += timeout;
        return false;
    }
    static void vm_fault(void*, const IrqVmFault&, uint32_t) noexcept {}
    static void hang(void*, uint32_t, FenceHangReason, uint32_t, uint32_t) noexcept {}
    static bool store(void*, uint8_t* address, uint64_t value) noexcept {
        const rx7900::sdma::CommandBuffer out(address, 8);
        return out.write_u32(0, static_cast<uint32_t>(value)) && out.write_u32(4, static_cast<uint32_t>(value >> 32));
    }
    FencePlatform platform() noexcept {
        return {this, lock, unlock, now_us, signal, reset, block, vm_fault, hang};
    }
};

// Test VRAM outside the shared 16 MiB arena: the adopted primary at VRAM offset 0
// and the GPU-only offscreen range at the top of a 128 MiB test VRAM layout.
constexpr uint64_t test_vram_bytes = 128ULL << 20;
constexpr uint64_t offscreen_offset = 112ULL << 20;
constexpr uint64_t offscreen_bytes = 16ULL << 20;
constexpr uint32_t primary_width = 320;
constexpr uint32_t primary_height = 200;
constexpr uint32_t primary_pitch = 1344; // Not a multiple of the width: exercises pitch.
constexpr uint8_t offscreen_poison = 0xcd;

struct Region final {
    uint64_t base;
    std::vector<uint8_t> bytes;
};

// Local SDMA execution layer: SDMA doorbells execute the ring here; every other
// doorbell goes to the shared MES model (which forwards to SimulatedNavi31).
struct BlitSdma final {
    SimulatedNavi31* device = nullptr;
    rx7900_test::mes_model::MesModel* mes = nullptr;
    std::vector<Region> regions;
    unsigned errors = 0;
    unsigned ibs[2]{};
    unsigned fills = 0;
    unsigned copies = 0;
    unsigned linear_writes = 0;
    unsigned self_overlaps = 0;

    uint8_t* at(uint64_t mc) {
        for (auto& region : regions) {
            if (mc >= region.base && mc - region.base + 4 <= region.bytes.size()) {
                return region.bytes.data() + (mc - region.base);
            }
        }

        return device->cpu(mc);
    }

    bool read32(uint64_t mc, uint32_t& value) {
        const uint8_t* bytes = at(mc);

        if (bytes == nullptr || (mc & 3) != 0) {
            ++errors;
            return false;
        }

        return rx7900::ByteView(bytes, 4).read_u32(0, value);
    }

    bool write32(uint64_t mc, uint32_t value) {
        uint8_t* bytes = at(mc);

        if (bytes == nullptr || (mc & 3) != 0) {
            ++errors;
            return false;
        }

        return rx7900::sdma::CommandBuffer(bytes, 4).write_u32(0, value);
    }

    uint32_t word(uint64_t mc) {
        uint32_t value = 0;
        read32(mc, value);
        return value;
    }

    // IB contents: what blit2d and the Sdma6 IB test emit.
    void execute_ib(uint32_t instance, uint64_t ib, uint32_t dwords) {
        ++ibs[instance];
        uint32_t index = 0;

        while (index < dwords && errors == 0) {
            const uint32_t header = word(ib + index * 4ULL);
            const uint32_t op = header & 0xff;

            if (op == 0) {
                index += 1 + ((header >> 16) & 0x3fff);
            } else if (op == 0x02) {
                const uint64_t target = word(ib + (index + 1) * 4ULL) |
                                        (uint64_t{word(ib + (index + 2) * 4ULL)} << 32);
                const uint32_t count = word(ib + (index + 3) * 4ULL);

                for (uint32_t item = 0; item <= count; ++item) {
                    write32(target + item * 4ULL, word(ib + (index + 4 + item) * 4ULL));
                }

                ++linear_writes;
                index += 5 + count;
            } else if (header == 0x40000401u) {
                uint32_t dw[13]{};

                for (uint32_t item = 0; item < 13; ++item) {
                    dw[item] = word(ib + (index + item) * 4ULL);
                }

                const uint64_t src = dw[1] | (uint64_t{dw[2]} << 32);
                const uint64_t dst = dw[6] | (uint64_t{dw[7]} << 32);
                const uint32_t sx = dw[3] & 0x3fff;
                const uint32_t sy = dw[3] >> 16;
                const uint32_t sp = (dw[4] >> 13) + 1;
                const uint32_t dxp = dw[8] & 0x3fff;
                const uint32_t dyp = dw[8] >> 16;
                const uint32_t dp = (dw[9] >> 13) + 1;
                const uint32_t w = (dw[11] & 0x3fff) + 1;
                const uint32_t h = (dw[11] >> 16) + 1;

                if (dw[12] != 0 || sx + w > sp || dxp + w > dp ||
                    uint64_t{sp} * (sy + h) > dw[5] + 1ULL || uint64_t{dp} * (dyp + h) > dw[10] + 1ULL) {
                    ++errors;
                    return;
                }

                // The engine never relies on intra-packet order: a packet whose
                // destination overlaps its own source is a defect.
                const uint64_t src_first = src + (uint64_t{sy} * sp + sx) * 4;
                const uint64_t src_end = src + (uint64_t{sy + h - 1} * sp + sx + w) * 4;
                const uint64_t dst_first = dst + (uint64_t{dyp} * dp + dxp) * 4;
                const uint64_t dst_end = dst + (uint64_t{dyp + h - 1} * dp + dxp + w) * 4;

                if (src_first < dst_end && dst_first < src_end &&
                    (src != dst || sp != dp ||
                     (sx < dxp + w && dxp < sx + w && sy < dyp + h && dyp < sy + h))) {
                    ++self_overlaps;
                }

                std::vector<uint32_t> pixels(size_t{w} * h);

                for (uint32_t row = 0; row < h; ++row) {
                    for (uint32_t column = 0; column < w; ++column) {
                        read32(src + (uint64_t{sy + row} * sp + sx + column) * 4, pixels[size_t{row} * w + column]);
                    }
                }

                for (uint32_t row = 0; row < h; ++row) {
                    for (uint32_t column = 0; column < w; ++column) {
                        write32(dst + (uint64_t{dyp + row} * dp + dxp + column) * 4, pixels[size_t{row} * w + column]);
                    }
                }

                ++copies;
                index += 13;
            } else if (header == 0x8000000Bu) {
                const uint64_t dst = word(ib + (index + 1) * 4ULL) | (uint64_t{word(ib + (index + 2) * 4ULL)} << 32);
                const uint32_t value = word(ib + (index + 3) * 4ULL);
                const uint32_t count = word(ib + (index + 4) * 4ULL);

                if (((count + 1) & 3) != 0 || count + 1 > rx7900::sdma::max_transfer_bytes) {
                    ++errors;
                    return;
                }

                for (uint64_t address = dst; address < dst + count + 1; address += 4) {
                    write32(address, value);
                }

                ++fills;
                index += 5;
            } else {
                std::fprintf(stderr, "unexpected SDMA IB header 0x%08x\n", header);
                ++errors;
                return;
            }
        }
    }

    // Ring frame walk identical to SimulatedNavi31::sdma_execute.
    void execute(uint32_t instance, uint64_t wptr) {
        if (instance == device->sdma_hang_instance ||
            (device->reg(0xa000 + 0x589a + instance * 0x20) & 1u) != 0 ||
            (device->reg(SimulatedNavi31::sdma_queue(instance, 0x80)) & 1u) == 0) {
            return;
        }

        ++device->sdma_executions[instance];
        const uint64_t ring = (static_cast<uint64_t>(device->reg(SimulatedNavi31::sdma_queue(instance, 0x81))) << 8) |
                              (static_cast<uint64_t>(device->reg(SimulatedNavi31::sdma_queue(instance, 0x82))) << 40);
        const uint64_t size = 4ULL << ((device->reg(SimulatedNavi31::sdma_queue(instance, 0x80)) >> 1) & 0x1f);
        uint64_t& rptr = device->sdma_rptr[instance];

        while (rptr < wptr) {
            const auto ring_word = [&](uint64_t index) { return device->vram_u32(ring + ((rptr + index * 4) & (size - 1))); };
            const uint32_t header = ring_word(0);
            const uint32_t op = header & 0xff;

            if (op == 0) {
                rptr += 4;
            } else if (op == 0x11 || op == 0x09) {
                rptr += 20;
            } else if (op == 0x08) {
                rptr += 24;
            } else if (op == 0x06) {
                rptr += 8;
            } else if (op == 0x04) {
                execute_ib(instance, (static_cast<uint64_t>(ring_word(2)) << 32) | ring_word(1), ring_word(3));
                rptr += 24;
            } else if (header == 0x00030005u) {
                device->put_vram_u32(ring_word(1) | (static_cast<uint64_t>(ring_word(2)) << 32), ring_word(3));
                rptr += 16;
            } else {
                return;
            }
        }

        const uint64_t writeback = (static_cast<uint64_t>(device->reg(SimulatedNavi31::sdma_queue(instance, 0x88))) << 32) |
                                   device->reg(SimulatedNavi31::sdma_queue(instance, 0x89));
        device->put_vram_u32(writeback, static_cast<uint32_t>(rptr));
        device->put_vram_u32(writeback + 4, static_cast<uint32_t>(rptr >> 32));
    }

    static bool doorbell32(void* context, uint32_t index, uint32_t value) noexcept {
        return rx7900_test::mes_model::MesModel::doorbell32(static_cast<BlitSdma*>(context)->mes, index, value);
    }

    static bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
        auto& self = *static_cast<BlitSdma*>(context);

        for (uint32_t instance = 0; instance < 2; ++instance) {
            if ((self.device->reg(SimulatedNavi31::sdma_queue(instance, 0x92)) & 0x10000000u) != 0 &&
                ((self.device->reg(SimulatedNavi31::sdma_queue(instance, 0xab)) & 0x0ffffffcu) >> 2) == index) {
                self.device->doorbell_writes.emplace_back(index, value);

                if (!self.device->writes_ok) {
                    return false;
                }

                self.execute(instance, value);
                return true;
            }
        }

        return rx7900_test::mes_model::MesModel::doorbell64(self.mes, index, value);
    }

    DoorbellIo doorbells() noexcept { return {this, 0x10000, doorbell32, doorbell64}; }
};

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::mes_model::MesModel mes_firmware{device.get()};
    BlitSdma sdma{};
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    std::unique_ptr<UcodeCatalog> catalog = std::make_unique<UcodeCatalog>();
    std::unique_ptr<AsicInit> asic = std::make_unique<AsicInit>();
    std::vector<uint8_t> discovery = rx7900_test::navi31_discovery();
    std::vector<uint8_t> sos = sos_container();
    File pfp = gfx(0x40, 0x20);
    File me = gfx(0x40, 0x20);
    File mec = gfx(0x80, 0x40);
    File rlc_file = rlc(3);
    File mes_file = mes();
    File kiq = mes();
    File imu_file = imu();
    File smu_file = smu();
    File sdma_file = rx7900_test::ucode_files::sdma();
    File dmcub_file = dmcub();
    File ta_file = ta_navi31();
    AsicInitPolicy policy{{1000, 1000}, {1000, 50, 10}, {100, 3}, {1000, 2100000}, true};
    std::vector<uint8_t> board = std::vector<uint8_t>(smu13_board_table_bytes, 0x5a);
    SmuPcieLink pcie{3, 6};
    HostFences host{};
    std::unique_ptr<FenceManager> fences = std::make_unique<FenceManager>();
    std::unique_ptr<VramAllocator> vram = std::make_unique<VramAllocator>();

    Fixture() {
        backend.device = device.get();
        sdma.device = device.get();
        sdma.mes = &mes_firmware;
        mes_file.u32(56, 0xf0005000u);
        mes_file.u32(60, 0xffffffffu);
        kiq.u32(56, 0xf0005000u);
        kiq.u32(60, 0xffffffffu);
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset, rx7900_test::arena_bytes},
                            backend.backend());
        catalog->add(UcodeSource::pfp, pfp.view());
        catalog->add(UcodeSource::me, me.view());
        catalog->add(UcodeSource::mec, mec.view());
        catalog->add(UcodeSource::rlc, rlc_file.view());
        catalog->add(UcodeSource::mes, mes_file.view());
        catalog->add(UcodeSource::mes_kiq, kiq.view());
        catalog->add(UcodeSource::smu, smu_file.view());
        catalog->add(UcodeSource::sdma, sdma_file.view());
        catalog->add(UcodeSource::dmcub, dmcub_file.view());
        catalog->add(UcodeSource::imu, imu_file.view());
        sdma.regions.push_back({rx7900_test::fb_base, std::vector<uint8_t>(size_t{primary_pitch} * primary_height, 0)});
        sdma.regions.push_back({rx7900_test::fb_base + offscreen_offset,
                                std::vector<uint8_t>(offscreen_bytes, offscreen_poison)});
    }

    AsicInputs inputs() {
        return {mes_firmware.io(), {discovery.data(), discovery.size()}, {sos.data(), sos.size()},
                catalog.get(), ta_file.view(), manager.get(), nullptr, device->training_io(),
                {board.data(), board.size()}, pcie, 0x0000000012345000ULL,
                {device.get(), SimulatedNavi31::bus_master, nullptr, SimulatedNavi31::confirm}, false,
                0x000000e000000000ULL, sdma.doorbells(), device->gtt(), true, {nullptr, nullptr}};
    }

    // Bring-up, kernel submission and the test VRAM allocator (primary and the
    // shared arena reserved, so offscreen surfaces land in the top 16 MiB).
    bool start() {
        const VramRange reserved{0, offscreen_offset};
        return asic->run(inputs(), policy, {device.get(), SimulatedNavi31::confirm}) == AsicStatus::completed &&
               fences->initialize(host.platform()) == FenceStatus::ready &&
               asic->enable_kernel_submission(*fences, {&host, HostFences::store}) == AsicStatus::completed &&
               vram->initialize({test_vram_bytes, test_vram_bytes}, &reserved, 1) == VramStatus::success;
    }

    Radeon2dBackendConfig config() {
        Radeon2dBackendConfig value = radeon2d_default_config(asic->sdma_rings(), *fences, asic->gtt_manager(),
                                                              *vram, rx7900_test::fb_base);
        value.scratch_bytes = 128u << 10;
        return value;
    }

    Region& primary() { return sdma.regions.at(0); }
    Region& offscreen() { return sdma.regions.at(1); }
};

// ------------------------------------------------------------ pixel references

struct Image final {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint32_t> pixels;

    Image(uint32_t w, uint32_t h, uint32_t value) : width(w), height(h), pixels(size_t{w} * h, value) {}
    uint32_t& at(uint32_t x, uint32_t y) { return pixels.at(size_t{y} * width + x); }
    uint32_t at(uint32_t x, uint32_t y) const { return pixels.at(size_t{y} * width + x); }
};

uint32_t read_pixel(const std::vector<uint8_t>& bytes, size_t offset) {
    uint32_t value = 0;
    rx7900::ByteView(bytes.data(), bytes.size()).read_u32(offset, value);
    return value;
}

void write_pixel(std::vector<uint8_t>& bytes, size_t offset, uint32_t value) {
    rx7900::sdma::CommandBuffer(bytes.data(), bytes.size()).write_u32(offset, value);
}

bool primary_matches(Fixture& fixture, const Image& reference) {
    for (uint32_t y = 0; y < reference.height; ++y) {
        for (uint32_t x = 0; x < reference.width; ++x) {
            if (read_pixel(fixture.primary().bytes, size_t{y} * primary_pitch + x * 4u) !=
                reference.pixels.at(size_t{y} * reference.width + x)) {
                std::fprintf(stderr, "primary mismatch at %u,%u\n", x, y);
                return false;
            }
        }
    }

    return true;
}

// Snapshot semantics of a CopyRequest: every destination pixel takes the source
// pixel (dest + offset) from before the operation.
void reference_copy(Image& target, const Image& source_snapshot, const std::vector<abi::Rect>& rects,
                    int32_t dx, int32_t dy) {
    for (const auto& rect : rects) {
        for (int32_t y = rect.top; y < rect.bottom; ++y) {
            for (int32_t x = rect.left; x < rect.right; ++x) {
                target.at(static_cast<uint32_t>(x), static_cast<uint32_t>(y)) =
                    source_snapshot.pixels.at(static_cast<size_t>(y + dy) * source_snapshot.width +
                                              static_cast<size_t>(x + dx));
            }
        }
    }
}

// ------------------------------------------------------------ IOCTL client

struct Client final {
    radeon2d::Dispatcher* dispatcher = nullptr;
    abi::Header session{};
    uint32_t primary = 0;
    uint8_t* staging = nullptr;
    uint32_t staging_bytes = 0;

    template <typename Encode>
    radeon2d::DispatchResult call(uint32_t code, uint32_t bytes, Encode encode, uint32_t capacity = abi::max_reply_bytes) {
        std::vector<uint8_t> request(bytes, 0);

        if (!encode(rx7900::sdma::CommandBuffer(request.data(), request.size()))) {
            return {radeon2d::DispatchStatus::invalid_parameter, 0};
        }

        return dispatcher->dispatch(code, rx7900::ByteView(request.data(), request.size()), capacity);
    }

    bool receipt(radeon2d::DispatchResult result, abi::Receipt& value) const {
        return result.status == radeon2d::DispatchStatus::success && abi::decode_receipt(dispatcher->reply(), value);
    }

    bool open() {
        const auto result = call(abi::ioctl_open_session, abi::open_session_request_bytes,
                                 [](const rx7900::sdma::CommandBuffer& out) { return abi::encode_open_session(out); });
        abi::SessionGrant grant{};

        if (result.status != radeon2d::DispatchStatus::success ||
            !abi::decode_session_grant(dispatcher->reply(), grant)) {
            return false;
        }

        session = grant.session;
        primary = grant.primary_surface;
        // Test boundary: the grant carries the backend's kernel mapping of staging.
        staging = reinterpret_cast<uint8_t*>(static_cast<uintptr_t>(grant.staging_address));
        staging_bytes = grant.staging_bytes;
        // Take the primary: only the owning session may draw on it.
        abi::AssertPrimaryCommand command{};
        command.session = session;
        command.enable = 1;
        abi::Receipt value{};
        return receipt(call(abi::ioctl_assert_primary, abi::assert_primary_request_bytes,
                            [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_assert_primary(o, command); }),
                       value) && value.outcome == abi::outcome_completed;
    }

    void close() {
        const abi::Header header = session;
        call(abi::ioctl_close_session, abi::close_session_request_bytes,
             [&](const rx7900::sdma::CommandBuffer& out) { return abi::encode_close_session(out, header); });
    }

    static abi::RectList list(const std::vector<abi::Rect>& rects) {
        abi::RectList value{};
        value.count = static_cast<uint32_t>(rects.size());

        for (size_t index = 0; index < rects.size(); ++index) {
            value.rects[index] = rects[index];
        }

        return value;
    }

    bool fill(uint32_t surface, uint32_t color, const std::vector<abi::Rect>& rects, abi::Receipt& out) {
        abi::FillCommand command{};
        command.session = session;
        command.surface = surface;
        command.color = color;
        command.rects = list(rects);
        return receipt(call(abi::ioctl_fill, abi::fill_request_bytes,
                            [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_fill(o, command); }), out);
    }

    bool copy(uint32_t target, uint32_t source, int32_t dx, int32_t dy, const std::vector<abi::Rect>& rects,
              abi::Receipt& out) {
        abi::CopyCommand command{};
        command.session = session;
        command.target = target;
        command.source = source;
        command.source_dx = dx;
        command.source_dy = dy;
        command.rects = list(rects);
        return receipt(call(abi::ioctl_copy, abi::copy_request_bytes,
                            [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_copy(o, command); }), out);
    }

    bool transfer(bool upload, uint32_t surface, uint32_t offset, uint32_t pitch, int32_t origin_x, int32_t origin_y,
                  const std::vector<abi::Rect>& rects, abi::Receipt& out) {
        abi::TransferCommand command{};
        command.session = session;
        command.surface = surface;
        command.staging_offset = offset;
        command.staging_pitch = pitch;
        command.origin_x = origin_x;
        command.origin_y = origin_y;
        command.rects = list(rects);
        return receipt(call(upload ? abi::ioctl_upload : abi::ioctl_readback, abi::transfer_request_bytes,
                            [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_transfer(o, command); }),
                       out);
    }

    bool create(uint32_t width, uint32_t height, abi::SurfaceGrant& grant) {
        abi::SurfaceCreateCommand command{};
        command.session = session;
        command.width = width;
        command.height = height;
        command.format = abi::format_xrgb8888;
        const auto result = call(abi::ioctl_surface_create, abi::surface_create_request_bytes,
                                 [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_surface_create(o, command); });
        return result.status == radeon2d::DispatchStatus::success &&
               abi::decode_surface_grant(dispatcher->reply(), grant);
    }

    radeon2d::DispatchResult create_status(uint32_t width, uint32_t height) {
        abi::SurfaceCreateCommand command{};
        command.session = session;
        command.width = width;
        command.height = height;
        command.format = abi::format_xrgb8888;
        return call(abi::ioctl_surface_create, abi::surface_create_request_bytes,
                    [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_surface_create(o, command); });
    }

    bool destroy(uint32_t surface, abi::Receipt& out) {
        abi::SurfaceDestroyCommand command{};
        command.session = session;
        command.surface = surface;
        return receipt(call(abi::ioctl_surface_destroy, abi::surface_destroy_request_bytes,
                            [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_surface_destroy(o, command); }),
                       out);
    }

    bool sync(uint64_t fence, abi::Receipt& out) {
        abi::SyncCommand command{};
        command.session = session;
        command.fence = fence;
        return receipt(call(abi::ioctl_sync, abi::sync_request_bytes,
                            [&](const rx7900::sdma::CommandBuffer& o) { return abi::encode_sync(o, command); }), out);
    }

    uint32_t staged(size_t offset) const {
        uint32_t value = 0;
        rx7900::ByteView(staging, staging_bytes).read_u32(offset, value);
        return value;
    }

    void stage(size_t offset, uint32_t value) const {
        rx7900::sdma::CommandBuffer(staging, staging_bytes).write_u32(offset, value);
    }
};

constexpr uint32_t staging_bytes = 64u << 10;

uint32_t pattern(uint32_t x, uint32_t y) {
    return 0x5a000000u ^ (y << 12) ^ (x * 0x9e37u);
}

// ------------------------------------------------------------ status mappings

void mappings() {
    using radeon2d::BackendStatus;
    expect(radeon2d_backend_status(Blit2dStatus::success) == BackendStatus::ok, "success maps to ok");

    for (const auto status : {Blit2dStatus::not_initialized, Blit2dStatus::invalid_argument,
                              Blit2dStatus::invalid_surface, Blit2dStatus::invalid_rect,
                              Blit2dStatus::too_many_rects, Blit2dStatus::out_of_bounds,
                              Blit2dStatus::overlapping_surfaces, Blit2dStatus::unordered_overlap,
                              Blit2dStatus::engine_memory_overlap, Blit2dStatus::invalid_staging,
                              Blit2dStatus::memory_failure, Blit2dStatus::busy}) {
        expect(radeon2d_backend_status(status) == BackendStatus::rejected,
               "refused-before-encoding statuses map to rejected");
    }

    for (const auto status : {Blit2dStatus::faulted, Blit2dStatus::submit_failed, Blit2dStatus::fence_timeout,
                              Blit2dStatus::fence_failed}) {
        expect(radeon2d_backend_status(status) == BackendStatus::uncertain, "fault statuses map to uncertain");
    }

    expect(radeon2d_submit_result(Sdma6Status::started) == Blit2dSubmitResult::submitted &&
           radeon2d_submit_result(Sdma6Status::ready) == Blit2dSubmitResult::submitted &&
           radeon2d_submit_result(Sdma6Status::ring_full) == Blit2dSubmitResult::busy &&
           radeon2d_submit_result(Sdma6Status::busy) == Blit2dSubmitResult::busy &&
           radeon2d_submit_result(Sdma6Status::ring_hung) == Blit2dSubmitResult::failed &&
           radeon2d_submit_result(Sdma6Status::invalid_memory) == Blit2dSubmitResult::failed &&
           radeon2d_submit_result(Sdma6Status::quarantined) == Blit2dSubmitResult::failed,
           "Sdma6::submit statuses map as blit2d.hpp documents");
    expect(radeon2d_query_state(FenceStatus::signalled) == Blit2dFenceState::signalled &&
           radeon2d_query_state(FenceStatus::pending) == Blit2dFenceState::pending &&
           radeon2d_query_state(FenceStatus::ring_hung) == Blit2dFenceState::failed &&
           radeon2d_query_state(FenceStatus::invalid_argument) == Blit2dFenceState::failed &&
           radeon2d_wait_state(FenceStatus::signalled) == Blit2dFenceState::signalled &&
           radeon2d_wait_state(FenceStatus::timed_out) == Blit2dFenceState::pending &&
           radeon2d_wait_state(FenceStatus::busy) == Blit2dFenceState::failed &&
           radeon2d_wait_state(FenceStatus::ring_hung) == Blit2dFenceState::failed,
           "FenceManager query/wait statuses map as blit2d.hpp documents");
}

// ------------------------------------------------------------ end to end

void end_to_end() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();

    expect(fixture.asic->run(fixture.inputs(), fixture.policy, {fixture.device.get(), SimulatedNavi31::confirm}) ==
               AsicStatus::completed,
           "AsicInit bring-up completes with the local SDMA execution layer");
    expect(fixture.fences->initialize(fixture.host.platform()) == FenceStatus::ready, "fence manager ready");
    {
        const VramRange reserved{0, offscreen_offset};
        expect(fixture.vram->initialize({test_vram_bytes, test_vram_bytes}, &reserved, 1) == VramStatus::success,
               "test VRAM allocator ready");
    }
    expect(backend->initialize(fixture.config()) == Radeon2dStatus::submission_unavailable,
           "backend refuses an SDMA instance without kernel submission");
    expect(fixture.asic->enable_kernel_submission(*fixture.fences, {&fixture.host, HostFences::store}) ==
               AsicStatus::completed,
           "kernel submission enabled after bring-up");

    Radeon2dBackendConfig bad = fixture.config();
    bad.instance = 2;
    expect(backend->initialize(bad) == Radeon2dStatus::invalid_argument, "instance beyond SDMA1 refused");
    bad = fixture.config();
    bad.ib_bytes = 100;
    expect(backend->initialize(bad) == Radeon2dStatus::invalid_argument, "unaligned IB slot refused");

    const uint64_t sdma0_before = fixture.asic->sdma_report().submissions[0];
    const size_t gtt_before = fixture.device->system_used;
    expect(backend->initialize(fixture.config()) == Radeon2dStatus::ready, "backend ready on SDMA1");
    expect(backend->initialize(fixture.config()) == Radeon2dStatus::invalid_argument, "second initialize refused");
    std::printf("GTT system pages: %zu bytes used by bring-up, %zu after backend init\n", gtt_before,
                fixture.device->system_used);

    // Primary: the adopted scanout in VRAM, filled with a known pattern.
    Image primary(primary_width, primary_height, 0);

    for (uint32_t y = 0; y < primary_height; ++y) {
        for (uint32_t x = 0; x < primary_width; ++x) {
            primary.at(x, y) = pattern(x, y);
            write_pixel(fixture.primary().bytes, size_t{y} * primary_pitch + x * 4u, pattern(x, y));
        }
    }

    const radeon2d::EngineSurface scanout{rx7900_test::fb_base, primary_width, primary_height, primary_pitch,
                                          abi::format_xrgb8888};
    expect(dispatcher->configure(backend->table(), scanout, staging_bytes, 0x1234u), "dispatcher configured");

    Client client{dispatcher.get()};
    expect(client.open() && client.staging != nullptr && client.staging_bytes == staging_bytes,
           "session opened with a GTT staging mapping");
    expect(backend->report().live_staging == 1, "one staging buffer live");
    bool staging_zero = true;

    for (uint32_t offset = 0; offset < staging_bytes; offset += 4) {
        staging_zero = staging_zero && client.staged(offset) == 0;
    }

    expect(staging_zero, "staging is zeroed before it is handed out");

    // Fill: three disjoint rectangles, arbitrary dword (high byte included).
    abi::Receipt receipt{};
    const std::vector<abi::Rect> fill_rects{{4, 4, 40, 20}, {100, 0, 320, 3}, {0, 190, 7, 200}};
    expect(client.fill(client.primary, 0x80c0ffeeu, fill_rects, receipt) &&
               receipt.outcome == abi::outcome_completed && receipt.fence == 1 && receipt.completed_fence == 1,
           "fill accepted with fence 1 and observed retired");

    for (const auto& rect : fill_rects) {
        for (int32_t y = rect.top; y < rect.bottom; ++y) {
            for (int32_t x = rect.left; x < rect.right; ++x) {
                primary.at(static_cast<uint32_t>(x), static_cast<uint32_t>(y)) = 0x80c0ffeeu;
            }
        }
    }

    expect(primary_matches(fixture, primary), "fill pixels match the CPU reference (pitch respected)");

    // Overlapping same-surface copy: content moves down-right by (4, 3).
    {
        const std::vector<abi::Rect> rects{{10, 10, 110, 60}, {10, 60, 60, 90}, {200, 100, 260, 140}};
        const Image snapshot = primary;
        expect(client.copy(client.primary, client.primary, -4, -3, rects, receipt) &&
                   receipt.outcome == abi::outcome_completed && receipt.fence == 2,
               "overlapping primary copy accepted");
        reference_copy(primary, snapshot, rects, -4, -3);
        expect(primary_matches(fixture, primary), "overlapping copy equals copying through a temporary");
    }

    // One-line scroll up (bounced through scratch) and a long horizontal shift (strips).
    {
        const std::vector<abi::Rect> scroll{{0, 0, 120, 100}};
        Image snapshot = primary;
        expect(client.copy(client.primary, client.primary, 0, 1, scroll, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "one-line scroll accepted");
        reference_copy(primary, snapshot, scroll, 0, 1);
        const std::vector<abi::Rect> shift{{0, 150, 318, 160}};
        snapshot = primary;
        expect(client.copy(client.primary, client.primary, 2, 0, shift, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "two-pixel left shift accepted");
        reference_copy(primary, snapshot, shift, 2, 0);
        // Move down by 6 inside a 10-row rectangle: two dependent strips.
        const std::vector<abi::Rect> strips{{0, 166, 300, 176}};
        snapshot = primary;
        expect(client.copy(client.primary, client.primary, 0, -6, strips, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "six-row move down accepted");
        reference_copy(primary, snapshot, strips, 0, -6);
        expect(primary_matches(fixture, primary), "scroll and shift match the CPU reference");
        const auto engine = backend->report().engine;
        expect(engine.bounced_rects >= 1 && engine.strip_rects >= 1 && engine.barrier_nops >= 1,
               "scratch bounce, strips and barriers were all exercised");
    }

    // Upload: two rectangles from staging (offset 256, pitch 512, origin (200, 100)).
    {
        const uint32_t offset = 256;
        const uint32_t pitch = 512;
        const std::vector<abi::Rect> rects{{200, 100, 230, 110}, {240, 104, 250, 120}};

        for (uint32_t row = 0; row < 20; ++row) {
            for (uint32_t column = 0; column < 50; ++column) {
                client.stage(offset + row * pitch + column * 4, 0x11000000u | (row << 8) | column);
            }
        }

        expect(client.transfer(true, client.primary, offset, pitch, 200, 100, rects, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "upload accepted");

        for (const auto& rect : rects) {
            for (int32_t y = rect.top; y < rect.bottom; ++y) {
                for (int32_t x = rect.left; x < rect.right; ++x) {
                    primary.at(static_cast<uint32_t>(x), static_cast<uint32_t>(y)) =
                        0x11000000u | (static_cast<uint32_t>(y - 100) << 8) | static_cast<uint32_t>(x - 200);
                }
            }
        }

        expect(primary_matches(fixture, primary), "uploaded pixels match the staging bytes");
    }

    // Readback: primary rectangle into staging offset 0, pitch 256, origin (30, 20).
    {
        const std::vector<abi::Rect> rects{{30, 20, 70, 50}};
        expect(client.transfer(false, client.primary, 0, 256, 30, 20, rects, receipt) &&
                   receipt.outcome == abi::outcome_completed && receipt.completed_fence == receipt.fence,
               "readback completed and synchronized before the reply");
        bool same = true;

        for (uint32_t y = 20; y < 50; ++y) {
            for (uint32_t x = 30; x < 70; ++x) {
                same = same && client.staged((y - 20) * 256 + (x - 30) * 4) == primary.at(x, y);
            }
        }

        expect(same, "readback bytes match the CPU reference");
    }

    // Offscreen surface: GPU-only VRAM, cleared, filled, copied both ways, read back.
    {
        const uint64_t usage_before = fixture.vram->usage_bytes();
        abi::SurfaceGrant grant{};
        expect(client.create(100, 40, grant) && grant.width == 100 && grant.height == 40 && grant.pitch == 512 &&
                   grant.format == abi::format_xrgb8888,
               "offscreen surface created with a 256-byte aligned pitch");
        expect(fixture.vram->usage_bytes() == usage_before + 512u * 40u, "surface allocated from the VRAM allocator");
        size_t cleared = 0;
        size_t first = SIZE_MAX;

        for (size_t index = 0; index < fixture.offscreen().bytes.size(); ++index) {
            if (fixture.offscreen().bytes[index] != offscreen_poison) {
                cleared += fixture.offscreen().bytes[index] == 0 ? 1 : 0;
                first = first == SIZE_MAX ? index : first;
            }
        }

        bool contiguous = first != SIZE_MAX && first + 512u * 40u <= offscreen_bytes;

        for (size_t index = 0; contiguous && index < 512u * 40u; ++index) {
            contiguous = fixture.offscreen().bytes[first + index] == 0;
        }

        expect(cleared == 512u * 40u && contiguous && first >= offscreen_bytes - (64u << 10),
               "the whole surface (pitch included) was cleared, in the topmost VRAM block (TOPDOWN)");

        Image surface(100, 40, 0);
        const std::vector<abi::Rect> fill{{0, 0, 100, 40}};
        expect(client.fill(grant.surface, 0x00abcdefu, {{5, 5, 50, 30}}, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "offscreen fill accepted");

        for (uint32_t y = 5; y < 30; ++y) {
            for (uint32_t x = 5; x < 50; ++x) {
                surface.at(x, y) = 0x00abcdefu;
            }
        }

        // Primary (60..120, 70..90) -> offscreen (40..100, 10..30).
        const std::vector<abi::Rect> into{{40, 10, 100, 30}};
        expect(client.copy(grant.surface, client.primary, 20, 60, into, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "primary to offscreen copy accepted");
        reference_copy(surface, primary, into, 20, 60);
        // Offscreen (0..60, 0..25) -> primary (250..310, 160..185).
        const std::vector<abi::Rect> out{{250, 160, 310, 185}};
        expect(client.copy(client.primary, grant.surface, -250, -160, out, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "offscreen to primary copy accepted");
        reference_copy(primary, surface, out, -250, -160);
        expect(primary_matches(fixture, primary), "primary matches after the offscreen copy");

        expect(client.transfer(false, grant.surface, 0, 400, 0, 0, fill, receipt) &&
                   receipt.outcome == abi::outcome_completed,
               "offscreen readback completed");
        bool same = true;

        for (uint32_t y = 0; y < 40; ++y) {
            for (uint32_t x = 0; x < 100; ++x) {
                same = same && client.staged(y * 400 + x * 4) == surface.at(x, y);
            }
        }

        expect(same, "offscreen readback matches the CPU reference");
        expect(client.destroy(grant.surface, receipt) && receipt.outcome == abi::outcome_completed &&
                   fixture.vram->usage_bytes() == usage_before && backend->report().live_surfaces == 0,
               "offscreen surface destroyed and its VRAM returned");
    }

    // Sync of everything this session submitted.
    expect(client.sync(0, receipt) && receipt.outcome == abi::outcome_completed &&
               receipt.fence == backend->report().fence && receipt.completed_fence == receipt.fence,
           "sync(0) proves every fence retired");
    expect(fixture.sdma.errors == 0 && fixture.sdma.self_overlaps == 0 && fixture.sdma.ibs[1] > 0 && fixture.sdma.fills > 0 && fixture.sdma.copies > 0,
           "every IB decoded cleanly on SDMA1");
    expect(fixture.asic->sdma_report().submissions[0] == sdma0_before &&
               fixture.asic->sdma_report().submissions[1] > 0,
           "SDMA0 untouched: all 2D work went to instance 1");

    // Direct contract call with differing source offsets per rectangle: one
    // engine copy per rectangle in array order (the IOCTL path never sends this).
    {
        const radeon2d::CopyRect rects[2]{{{0, 120, 30, 10}, 2, 121}, {{40, 120, 20, 10}, 100, 20}};
        Image expected = primary;

        for (const auto& rect : rects) {
            const Image snapshot = expected;

            for (uint32_t y = 0; y < rect.destination.height; ++y) {
                for (uint32_t x = 0; x < rect.destination.width; ++x) {
                    expected.at(rect.destination.x + x, rect.destination.y + y) =
                        snapshot.pixels.at(size_t{rect.source_y + y} * primary_width + rect.source_x + x);
                }
            }
        }

        uint64_t fence = 0;
        const uint64_t before = backend->report().fence;
        expect(backend->copy(scanout, scanout, rects, 2, fence) == radeon2d::BackendStatus::ok &&
                   fence == before + 1 && backend->sync(fence) == radeon2d::BackendStatus::ok,
               "per-rectangle copy accepted with the next fence and synchronized");
        primary = expected;
        expect(primary_matches(fixture, primary), "per-rectangle copies execute in array order with memmove semantics");
    }

    // Direct contract checks: refusals change nothing and are reported rejected.
    {
        const auto before = backend->report();
        uint64_t fence = 7;
        const radeon2d::EngineRect outside{300, 190, 30, 20};
        expect(backend->fill(scanout, 1, &outside, 1, fence) == radeon2d::BackendStatus::rejected && fence == 0,
               "out-of-bounds fill rejected by the engine");
        radeon2d::EngineSurface alias = scanout;
        alias.gpu_address += 4;
        const radeon2d::CopyRect overlap{{0, 0, 8, 8}, 0, 0};
        expect(backend->copy(scanout, alias, &overlap, 1, fence) == radeon2d::BackendStatus::rejected,
               "distinct overlapping surfaces rejected (overlapping_surfaces)");
        radeon2d::EngineSurface wrong = scanout;
        wrong.format = 7;
        expect(backend->fill(wrong, 1, &outside, 1, fence) == radeon2d::BackendStatus::rejected,
               "non-XRGB8888 surface rejected");
        expect(backend->sync(backend->report().fence + 1) == radeon2d::BackendStatus::rejected,
               "sync of a fence never handed out is rejected");
        const auto after = backend->report();
        expect(after.rejected == before.rejected + 3 && after.engine.ib_submissions == before.engine.ib_submissions &&
                   after.status == Radeon2dStatus::ready,
               "rejections submit nothing and do not fault the backend");
        radeon2d::StagingBuffer forged{};
        forged.allocation = 1;
        const radeon2d::TransferRect rect{{0, 0, 1, 1}, 0};
        expect(backend->transfer(true, scanout, forged, 4, &rect, 1, fence) == radeon2d::BackendStatus::rejected,
               "staging not owned by the backend rejected");
    }

    expect(backend->shutdown() == Radeon2dStatus::busy && backend->report().status == Radeon2dStatus::ready,
           "backend shutdown refused while a session's staging is live");
    client.close();
    expect(backend->report().live_staging == 0, "close released the staging buffer");
    expect(dispatcher->shutdown(), "dispatcher shutdown proves retirement");
    const size_t gtt_releases = fixture.device->gtt_releases;
    expect(backend->shutdown() == Radeon2dStatus::released && backend->report().status == Radeon2dStatus::uninitialized &&
               fixture.device->gtt_releases >= gtt_releases + 2,
           "backend shutdown releases the engine, IB ring and scratch pages");
    expect(fixture.asic->shutdown() == AsicStatus::completed, "ASIC shutdown completes after the backend released");
}

// A hung SDMA1: the backend becomes uncertain, poisons sessions, quarantines.
void hung_engine() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready,
           "second fixture: bring-up and backend ready");
    const radeon2d::EngineSurface scanout{rx7900_test::fb_base, primary_width, primary_height, primary_pitch,
                                          abi::format_xrgb8888};
    expect(dispatcher->configure(backend->table(), scanout, staging_bytes, 0x99u), "dispatcher configured");
    Client client{dispatcher.get()};
    abi::Receipt receipt{};
    abi::SurfaceGrant grant{};
    expect(client.open() && client.create(64, 64, grant), "session and surface before the hang");
    const uint64_t usage = fixture.vram->usage_bytes();

    fixture.device->sdma_hang_instance = 1;
    expect(client.fill(client.primary, 0xffffffffu, {{0, 0, 16, 16}}, receipt) &&
               receipt.outcome == abi::outcome_completed && receipt.completed_fence < receipt.fence,
           "work accepted onto the hung ring stays unretired");
    expect(client.sync(0, receipt) && receipt.outcome == abi::outcome_lost,
           "sync on the hung ring reports lost");
    const auto report = backend->report();
    expect(report.status == Radeon2dStatus::faulted &&
               (report.fault_reason == Blit2dStatus::fence_timeout || report.fault_reason == Blit2dStatus::fence_failed),
           "backend faulted with the fence reason");
    uint64_t fence = 0;
    const radeon2d::EngineRect rect{0, 0, 1, 1};
    expect(backend->fill(scanout, 0, &rect, 1, fence) == radeon2d::BackendStatus::uncertain && fence == 0,
           "faulted backend: every later operation is uncertain (sticky)");
    expect(client.fill(client.primary, 0, {{0, 0, 1, 1}}, receipt) && receipt.outcome == abi::outcome_lost,
           "poisoned session reports lost");
    // The surface was never used, so the dispatcher frees it without a sync;
    // the faulted backend cannot prove its clear retired and quarantines it.
    expect(client.destroy(grant.surface, receipt) && receipt.outcome == abi::outcome_completed &&
               fixture.vram->usage_bytes() == usage && backend->report().quarantined_surfaces == 1,
           "unproven surface destruction keeps the VRAM allocated (backend quarantine)");
    Client second{dispatcher.get()};
    expect(!second.open(), "no new staging is handed out by a faulted backend");
    client.close();
    expect(backend->report().live_staging == 1, "staging of the unretired session stays allocated");
    expect(dispatcher->shutdown(), "dispatcher shutdown (the closed session is already quarantined)");
    expect(backend->shutdown() == Radeon2dStatus::faulted, "backend shutdown refuses while faulted");
    // A proven reset (simulated here) is the only recovery.
    expect(backend->release_after_reset() == Radeon2dStatus::released &&
               fixture.vram->usage_bytes() == usage - 64u * 256u,
           "release_after_reset returns every allocation, quarantined ones included");
}


// Sends rects in requests of at most abi::max_rects, in order (the display
// DLL's batching); true when every request completed.
template <typename Send>
bool batched(const std::vector<abi::Rect>& rects, unsigned& requests, Send send) {
    bool completed = true;

    for (size_t first = 0; first < rects.size(); first += abi::max_rects) {
        const size_t last = std::min(rects.size(), first + abi::max_rects);
        const std::vector<abi::Rect> batch(rects.begin() + static_cast<std::ptrdiff_t>(first),
                                           rects.begin() + static_cast<std::ptrdiff_t>(last));
        abi::Receipt receipt{};
        completed = send(batch, receipt) && receipt.outcome == abi::outcome_completed && completed;
        ++requests;
    }

    return completed;
}

// A y-x banded region (as CLIPOBJ enumerates it), sorted top-down when the
// source lies below (dy > 0) and left-to-right when it lies right (dx > 0).
std::vector<abi::Rect> banded_region(std::mt19937& random, int32_t margin, int32_t dx, int32_t dy) {
    std::vector<abi::Rect> region;
    int32_t top = margin;

    while (top < static_cast<int32_t>(primary_height) - margin - 2) {
        const int32_t bottom = std::min(top + 1 + static_cast<int32_t>(random() % 7),
                                        static_cast<int32_t>(primary_height) - margin);
        int32_t left = margin + static_cast<int32_t>(random() % 4);

        while (left < static_cast<int32_t>(primary_width) - margin - 1) {
            const int32_t right = std::min(left + 1 + static_cast<int32_t>(random() % 9),
                                           static_cast<int32_t>(primary_width) - margin);
            region.push_back({left, top, right, bottom});
            left = right + 1 + static_cast<int32_t>(random() % 4);
        }

        top = bottom + static_cast<int32_t>(random() % 2);
    }

    std::sort(region.begin(), region.end(), [&](const abi::Rect& a, const abi::Rect& b) {
        if (a.top != b.top) {
            return dy >= 0 ? a.top < b.top : a.top > b.top;
        }

        return dx >= 0 ? a.left < b.left : a.left > b.left;
    });
    return region;
}

bool surface_matches(Client& client, uint32_t surface, const Image& reference) {
    abi::Receipt receipt{};
    const uint32_t pitch = reference.width * 4u;

    if (!client.transfer(false, surface, 0, pitch, 0, 0,
                         {{0, 0, static_cast<int32_t>(reference.width), static_cast<int32_t>(reference.height)}},
                         receipt) ||
        receipt.outcome != abi::outcome_completed) {
        return false;
    }

    for (uint32_t y = 0; y < reference.height; ++y) {
        for (uint32_t x = 0; x < reference.width; ++x) {
            if (client.staged(size_t{y} * pitch + x * 4u) != reference.pixels.at(size_t{y} * reference.width + x)) {
                std::fprintf(stderr, "surface mismatch at %u,%u\n", x, y);
                return false;
            }
        }
    }

    return true;
}

// Clip lists far above 64 rectangles in ordered requests, screen<->offscreen and
// offscreen<->offscreen copies, then eviction to GTT across a parked backend.
void batching_and_eviction() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready,
           "third fixture: bring-up and backend ready");
    Image primary(primary_width, primary_height, 0);

    for (uint32_t y = 0; y < primary_height; ++y) {
        for (uint32_t x = 0; x < primary_width; ++x) {
            primary.at(x, y) = pattern(x, y);
            write_pixel(fixture.primary().bytes, size_t{y} * primary_pitch + x * 4u, pattern(x, y));
        }
    }

    const radeon2d::EngineSurface scanout{rx7900_test::fb_base, primary_width, primary_height, primary_pitch,
                                          abi::format_xrgb8888};
    expect(dispatcher->configure(backend->table(), scanout, staging_bytes, 0x77u), "dispatcher configured");
    Client client{dispatcher.get()};
    expect(client.open(), "batching session");
    std::mt19937 random(0x0b47c4edU);
    unsigned requests = 0;

    // Fill: a 20 x 12 grid of cells, 240 rectangles in four requests.
    {
        std::vector<abi::Rect> cells;

        for (int32_t row = 0; row < 12; ++row) {
            for (int32_t column = 0; column < 20; ++column) {
                cells.push_back({column * 16, row * 16, column * 16 + 13, row * 16 + 11});
            }
        }

        const unsigned before = requests;
        expect(batched(cells, requests, [&](const std::vector<abi::Rect>& batch, abi::Receipt& receipt) {
                   return client.fill(client.primary, 0x00406080u, batch, receipt);
               }) && requests - before == 4,
               "240-rectangle fill completes in four ordered requests");

        for (const auto& rect : cells) {
            for (int32_t y = rect.top; y < rect.bottom; ++y) {
                for (int32_t x = rect.left; x < rect.right; ++x) {
                    primary.at(static_cast<uint32_t>(x), static_cast<uint32_t>(y)) = 0x00406080u;
                }
            }
        }

        expect(primary_matches(fixture, primary), "batched fill pixels match the reference");
    }

    // Same-surface scrolls of banded regions with hundreds of rectangles.
    {
        const int32_t offsets[][2] = {{0, 3}, {0, -5}, {4, 0}, {-6, 0}, {3, 2}, {-2, -7}, {5, -1}, {-1, 6}};
        bool exact = true;
        size_t largest = 0;

        for (const auto& offset : offsets) {
            const int32_t dx = offset[0];
            const int32_t dy = offset[1];
            const std::vector<abi::Rect> region = banded_region(random, 10, dx, dy);
            const Image snapshot = primary;
            exact = batched(region, requests, [&](const std::vector<abi::Rect>& batch, abi::Receipt& receipt) {
                        return client.copy(client.primary, client.primary, dx, dy, batch, receipt);
                    }) && exact;
            reference_copy(primary, snapshot, region, dx, dy);
            exact = primary_matches(fixture, primary) && exact;
            largest = std::max(largest, region.size());
        }

        expect(exact && largest > 3 * abi::max_rects,
               "banded same-surface scrolls over hundreds of rectangles equal snapshot semantics across requests");
        std::printf("batched scrolls: largest region %zu rectangles, %u requests so far, %llu barriers\n", largest,
                    requests, static_cast<unsigned long long>(backend->report().engine.barrier_nops));
    }

    // Offscreen <-> offscreen and screen <-> offscreen with batched clips.
    const uint64_t vram_without_surfaces = fixture.vram->usage_bytes();
    abi::SurfaceGrant first{};
    abi::SurfaceGrant second{};
    expect(client.create(96, 64, first) && client.create(96, 64, second), "two offscreen surfaces");
    Image first_image(96, 64, 0);
    Image second_image(96, 64, 0);
    {
        abi::Receipt receipt{};
        // Upload a pattern into the first surface (staging pitch 384).
        for (uint32_t y = 0; y < 64; ++y) {
            for (uint32_t x = 0; x < 96; ++x) {
                first_image.at(x, y) = 0x7f000000u ^ (y << 10) ^ (x * 77u);
                client.stage(y * 384u + x * 4u, first_image.at(x, y));
            }
        }

        expect(client.transfer(true, first.surface, 0, 384, 0, 0, {{0, 0, 96, 64}}, receipt) &&
                   receipt.outcome == abi::outcome_completed, "pattern uploaded into offscreen surface");
        std::vector<abi::Rect> cells;

        for (int32_t row = 0; row < 8; ++row) {
            for (int32_t column = 0; column < 12; ++column) {
                cells.push_back({column * 8, row * 8, column * 8 + 6, row * 8 + 7});
            }
        }

        expect(batched(cells, requests, [&](const std::vector<abi::Rect>& batch, abi::Receipt& out) {
                   return client.copy(second.surface, first.surface, 0, 0, batch, out);
               }), "96-rectangle offscreen-to-offscreen copy in two requests");
        reference_copy(second_image, first_image, cells, 0, 0);
        expect(surface_matches(client, second.surface, second_image), "offscreen copy matches the reference");

        // Second surface -> primary (dest (200..296, 120..184)), then primary -> first.
        std::vector<abi::Rect> onto;

        for (const auto& cell : cells) {
            onto.push_back({cell.left + 200, cell.top + 120, cell.right + 200, cell.bottom + 120});
        }

        expect(batched(onto, requests, [&](const std::vector<abi::Rect>& batch, abi::Receipt& out) {
                   return client.copy(client.primary, second.surface, -200, -120, batch, out);
               }), "offscreen-to-screen batched copy");
        reference_copy(primary, second_image, onto, -200, -120);
        expect(primary_matches(fixture, primary), "screen matches after offscreen-to-screen copy");
        expect(batched(cells, requests, [&](const std::vector<abi::Rect>& batch, abi::Receipt& out) {
                   return client.copy(first.surface, client.primary, 10, 10, batch, out);
               }), "screen-to-offscreen batched copy");
        reference_copy(first_image, primary, cells, 10, 10);
        expect(surface_matches(client, first.surface, first_image), "offscreen matches after screen-to-offscreen copy");
    }

    // Suspend: evict both surfaces to GTT, park, lose VRAM, restart, restore.
    {
        const auto before = backend->report();
        expect(backend->suspend() == Radeon2dStatus::busy && backend->report().status == Radeon2dStatus::ready,
               "backend refuses to park while surfaces still live in VRAM");
        expect(dispatcher->suspend(), "dispatcher suspend evicts every offscreen surface");
        const auto evicted = backend->report();
        expect(evicted.evictions == before.evictions + 2 && evicted.system_surfaces == 2 &&
                   evicted.live_surfaces == 2 && fixture.vram->usage_bytes() == vram_without_surfaces,
               "two surfaces now live in GTT and their VRAM is free");
        const uint64_t vram_parked = fixture.vram->usage_bytes();
        expect(backend->suspend() == Radeon2dStatus::suspended && backend->report().status == Radeon2dStatus::suspended &&
                   backend->report().live_staging == 1,
               "backend parks with the staging and GTT surfaces retained");
        // VRAM contents do not survive the power transition.
        std::fill(fixture.offscreen().bytes.begin(), fixture.offscreen().bytes.end(), offscreen_poison);
        expect(backend->initialize(fixture.config()) == Radeon2dStatus::ready, "backend restarts after resume");
        expect(dispatcher->resume(backend->table(), scanout), "dispatcher resumes");
        const auto restored = backend->report();
        expect(restored.restores == before.restores + 2 && restored.system_surfaces == 0 &&
                   fixture.vram->usage_bytes() >= vram_parked + 2u * 96u * 4u * 64u,
               "both surfaces restored into fresh VRAM");
        expect(surface_matches(client, first.surface, first_image) &&
                   surface_matches(client, second.surface, second_image),
               "offscreen contents survive suspend through GTT eviction");
        abi::Receipt receipt{};
        expect(client.fill(client.primary, 0x00102030u, {{0, 0, 8, 8}}, receipt) &&
                   receipt.outcome == abi::outcome_completed, "the same session and primary handle keep working");
        primary.at(0, 0) = 0x00102030u;

        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t x = 0; x < 8; ++x) {
                primary.at(x, y) = 0x00102030u;
            }
        }

        expect(primary_matches(fixture, primary), "primary drawing after resume");
    }

    // No GTT for an eviction: a 6 MiB surface exceeds the remaining simulated
    // system memory, so it is freed and reported lost; the small ones are saved.
    {
        abi::SurfaceGrant third{};
        expect(client.create(1536, 1024, third), "large third surface");
        const uint64_t vram_with_third = fixture.vram->usage_bytes();
        expect(dispatcher->suspend() && backend->report().eviction_failures == 1 &&
                   dispatcher->counters().lost == 1 && dispatcher->counters().evicted == 4 &&
                   fixture.vram->usage_bytes() <= vram_with_third - 6u * 1024u * 1024u,
               "eviction without system memory frees the surface (VRAM returned) and counts the loss");
        expect(backend->suspend() == Radeon2dStatus::suspended, "backend parks after the loss");
        expect(backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
                   dispatcher->resume(backend->table(), scanout), "restart after the lossy suspend");
        abi::Receipt receipt{};
        expect(client.fill(third.surface, 1, {{0, 0, 4, 4}}, receipt) &&
                   receipt.outcome == abi::outcome_surface_lost && receipt.lost_surface == third.surface,
               "the lost surface answers surface_lost naming it");
        expect(!client.destroy(third.surface, receipt), "exactly once: the reported handle is dead afterwards");
        expect(surface_matches(client, first.surface, first_image), "the other surfaces were saved again");
    }

    // Budget: a parked backend may restart with another budget.
    {
        expect(dispatcher->suspend() && backend->suspend() == Radeon2dStatus::suspended, "fourth suspend");
        Radeon2dBackendConfig tight = fixture.config();
        tight.surface_budget_bytes = backend->report().surface_bytes + 16u * 1024u;
        expect(backend->initialize(tight) == Radeon2dStatus::ready && dispatcher->resume(backend->table(), scanout),
               "restart with a tight budget");
        abi::SurfaceGrant fits{};
        expect(client.create(64, 64, fits), "a 16 KiB surface fits the remaining budget");
        expect(client.create_status(64, 64).status == radeon2d::DispatchStatus::busy,
               "the next surface exceeds the backend budget (busy)");
        Radeon2dBackendConfig foreign = fixture.config();
        foreign.vram_mc_base += 0x1000;
        expect(dispatcher->suspend() && backend->suspend() == Radeon2dStatus::suspended &&
                   backend->initialize(foreign) == Radeon2dStatus::invalid_argument &&
                   backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
                   dispatcher->resume(backend->table(), scanout),
               "a parked backend only restarts with the same memory owners");
    }

    std::printf("batching_and_eviction: %u batched requests\n", requests);
    client.close();
    expect(backend->report().live_surfaces == 0 && backend->report().live_staging == 0,
           "closing the session frees every surface and the staging");
    expect(dispatcher->shutdown() && backend->shutdown() == Radeon2dStatus::released, "orderly teardown");
    expect(fixture.sdma.errors == 0 && fixture.sdma.self_overlaps == 0, "every IB decoded cleanly");

    // Teardown of a parked backend: the dispatcher shutdown frees GTT surfaces.
    auto parked = std::make_unique<Radeon2dBackend>();
    auto parked_dispatcher = std::make_unique<radeon2d::Dispatcher>();
    Client parked_client{parked_dispatcher.get()};
    abi::SurfaceGrant kept{};
    expect(parked->initialize(fixture.config()) == Radeon2dStatus::ready &&
               parked_dispatcher->configure(parked->table(), scanout, staging_bytes, 0x78u) && parked_client.open() &&
               parked_client.create(64, 64, kept) && parked_dispatcher->suspend() &&
               parked->suspend() == Radeon2dStatus::suspended,
           "parked backend with a GTT surface");
    expect(parked->shutdown() == Radeon2dStatus::busy, "parked shutdown refused while the surface lives");
    expect(parked_dispatcher->shutdown() && parked->report().live_surfaces == 0 &&
               parked->report().live_staging == 0 && parked->shutdown() == Radeon2dStatus::released,
           "abandoning a resume: dispatcher shutdown frees everything, then the backend releases");
    expect(fixture.asic->shutdown() == AsicStatus::completed, "ASIC shutdown after both backends released");
}

// ------------------------------------------------------------ compute execution
// A deferred compute queue for the backend's compute submitter: submit records
// the IB, query reports pending, and wait executes every recorded IB in order
// (decoding SET_SH_REG and DISPATCH_DIRECT exactly as raster2d emits them) with
// the shared GFX11 interpreter on the fixture memory. Work that is not waited
// for therefore runs after later SDMA work, so a missing cross-engine wait shows
// up as wrong pixels.
struct ComputeExecutor final {
    Fixture* fixture = nullptr;
    rx7900::memory::Manager* gtt = nullptr;
    struct Pending final {
        rx7900::memory::Handle ib;
        uint64_t offset;
        uint32_t dwords;
        uint32_t sequence;
    };
    std::vector<Pending> pending;
    uint32_t sequence = 0;
    uint32_t executed = 0;
    unsigned dispatches = 0;
    unsigned errors = 0;
    unsigned waits = 0;
    bool hung = false;

    // A contiguous CPU view of [address, address + bytes) or null.
    uint8_t* span(uint64_t address, uint64_t bytes) const {
        uint8_t* first = fixture->sdma.at(address);

        if (first == nullptr || bytes == 0) {
            return nullptr;
        }

        for (uint64_t offset = 4096 - (address & 4095); offset < bytes; offset += 4096) {
            if (fixture->sdma.at(address + offset) != first + offset) {
                return nullptr;
            }
        }

        return fixture->sdma.at(address + bytes - 4) == first + bytes - 4 ? first : nullptr;
    }

    void run_dispatch(uint64_t program, uint64_t arguments, uint32_t groups_x, uint32_t groups_y) {
        uint8_t* code = span(program, 4096);
        uint8_t* args = span(arguments, raster2d_args_region_bytes);

        if (code == nullptr || args == nullptr) {
            ++errors;
            return;
        }

        gfx11_interp::Buffer args_buffer;
        args_buffer.base = arguments;
        args_buffer.bytes.assign(args, args + raster2d_args_region_bytes);
        std::vector<std::unique_ptr<gfx11_interp::Buffer>> buffers;
        std::vector<uint8_t*> origins;
        gfx11_interp::Memory memory;
        memory.buffers.push_back(&args_buffer);

        // Raw V# operands at argument dwords 0, 4, 8, 12 outside the args region.
        for (uint32_t word : {0u, 4u, 8u, 12u}) {
            uint32_t low = 0;
            uint32_t high = 0;
            uint32_t records = 0;
            const rx7900::ByteView view(args_buffer.bytes.data(), args_buffer.bytes.size());
            view.read_u32(word * 4u, low);
            view.read_u32(word * 4u + 4, high);
            view.read_u32(word * 4u + 8, records);
            const uint64_t base = (static_cast<uint64_t>(high & 0xffffu) << 32) | low;

            if (records == 0 || (base >= arguments && base < arguments + raster2d_args_region_bytes)) {
                continue;
            }

            uint8_t* bytes = span(base, records);

            if (bytes == nullptr) {
                ++errors;
                continue;
            }

            auto buffer = std::make_unique<gfx11_interp::Buffer>();
            buffer->base = base;
            buffer->bytes.assign(bytes, bytes + records);
            memory.buffers.push_back(buffer.get());
            // Every kernel stores only through the destination V# (dword 0).
            // Writing back a read operand's private copy would replay stale
            // bytes over a same-surface destination; the GPU has no copies.
            origins.push_back(word == 0 ? bytes : nullptr);
            buffers.push_back(std::move(buffer));
        }

        // Gradient-triangle row table (raster2d vertex form): a 64-bit address in
        // argument dwords 20/21 with row stride in dword 22, one record per group
        // row, outside the args region. Read-only for the kernel; other kernels'
        // dwords 20..22 name no mapped table and are skipped.
        {
            uint32_t low = 0;
            uint32_t high = 0;
            uint32_t stride = 0;
            const rx7900::ByteView view(args_buffer.bytes.data(), args_buffer.bytes.size());
            view.read_u32(20 * 4u, low);
            view.read_u32(21 * 4u, high);
            view.read_u32(22 * 4u, stride);
            const uint64_t table = (static_cast<uint64_t>(high) << 32) | low;
            const uint64_t bytes = uint64_t{stride == 0 ? raster2d_triangle_row_bytes : stride} * groups_y;

            if (stride == raster2d_triangle_row_bytes &&
                (table < arguments || table >= arguments + raster2d_args_region_bytes)) {
                if (uint8_t* rows = span(table, bytes)) {
                    auto buffer = std::make_unique<gfx11_interp::Buffer>();
                    buffer->base = table;
                    buffer->bytes.assign(rows, rows + bytes);
                    memory.buffers.push_back(buffer.get());
                    buffers.push_back(std::move(buffer));
                    origins.push_back(nullptr);   // never written back
                }
            }
        }

        for (uint32_t gy = 0; gy < groups_y; ++gy) {
            for (uint32_t gx = 0; gx < groups_x; ++gx) {
                for (uint32_t wave = 0; wave < 2; ++wave) {
                    gfx11_interp::Wave w;
                    w.exec = 0xffffffffu;
                    w.sgpr[0] = static_cast<uint32_t>(arguments);
                    w.sgpr[1] = static_cast<uint32_t>(arguments >> 32);
                    w.sgpr[2] = gx;
                    w.sgpr[3] = gy;

                    for (uint32_t lane = 0; lane < gfx11_interp::lanes; ++lane) {
                        w.vgpr[0][lane] = wave * gfx11_interp::lanes + lane;
                    }

                    gfx11_interp::run(code, 1024, w, &memory);
                }
            }
        }

        for (size_t index = 0; index < buffers.size(); ++index) {
            if (origins[index] != nullptr) {
                std::copy(buffers[index]->bytes.begin(), buffers[index]->bytes.end(), origins[index]);
            }
        }

        ++dispatches;
    }

    void execute(const Pending& ib) {
        rx7900::memory::AllocationInfo info{};

        if (gtt->describe(ib.ib, info) != rx7900::memory::Status::success || info.backing.cpu_bytes == nullptr ||
            ib.offset + uint64_t{ib.dwords} * 4u > info.backing.cpu_byte_count) {
            ++errors;
            return;
        }

        const rx7900::ByteView words(info.backing.cpu_bytes + ib.offset, uint64_t{ib.dwords} * 4u);
        uint32_t pgm_lo = 0;
        uint32_t pgm_hi = 0;
        uint32_t user0 = 0;
        uint32_t user1 = 0;

        for (uint32_t index = 0; index < ib.dwords;) {
            uint32_t header = 0;
            words.read_u32(index * 4u, header);
            const uint32_t opcode = (header >> 8) & 0xffu;
            const uint32_t count = ((header >> 16) & 0x3fffu) + 1;

            if ((header >> 30) != 3) {
                ++errors;
                return;
            }

            if (opcode == 0x76) {
                uint32_t reg = 0;
                words.read_u32((index + 1) * 4u, reg);

                for (uint32_t value = 0; value + 1 < count; ++value) {
                    uint32_t data = 0;
                    words.read_u32((index + 2 + value) * 4u, data);
                    const uint32_t target = reg + value;
                    pgm_lo = target == ((0xB830u - 0xB000u) >> 2) ? data : pgm_lo;
                    pgm_hi = target == ((0xB834u - 0xB000u) >> 2) ? data : pgm_hi;
                    user0 = target == ((0xB900u - 0xB000u) >> 2) ? data : user0;
                    user1 = target == ((0xB904u - 0xB000u) >> 2) ? data : user1;
                }
            } else if (opcode == 0x15) {
                uint32_t gx = 0;
                uint32_t gy = 0;
                words.read_u32((index + 1) * 4u, gx);
                words.read_u32((index + 2) * 4u, gy);
                run_dispatch((static_cast<uint64_t>(pgm_hi) << 40) | (static_cast<uint64_t>(pgm_lo) << 8),
                             (static_cast<uint64_t>(user1) << 32) | user0, gx, gy);
            } else if (opcode != 0x10 && opcode != 0x46 && opcode != 0x58) {
                // NOP, EVENT_WRITE (CS_PARTIAL_FLUSH) and ACQUIRE_MEM order and
                // invalidate only; the executor runs dispatches in order anyway.
                ++errors;
                return;
            }

            index += 1 + count;
        }
    }

    void drain(uint32_t through) {
        while (!pending.empty() && pending.front().sequence <= through) {
            execute(pending.front());
            executed = pending.front().sequence;
            pending.erase(pending.begin());
        }
    }

    static Raster2dSubmitResult submit(void* context, rx7900::memory::Handle ib, uint64_t offset, uint32_t dwords,
                                       uint32_t& sequence) noexcept {
        auto& self = *static_cast<ComputeExecutor*>(context);
        uint64_t byte_offset = 0;

        if (!radeon2d_compute_ib_offset(offset, radeon2d_compute_ib_bytes, radeon2d_compute_ib_count,
                                        byte_offset) || dwords == 0 || dwords % 8 != 0) {
            ++self.errors;
            return Raster2dSubmitResult::failed;
        }

        sequence = ++self.sequence;
        self.pending.push_back({ib, byte_offset, dwords, sequence});
        return Raster2dSubmitResult::submitted;
    }

    static Raster2dFenceState query(void* context, uint32_t sequence) noexcept {
        const auto& self = *static_cast<ComputeExecutor*>(context);
        return sequence <= self.executed ? Raster2dFenceState::signalled : Raster2dFenceState::pending;
    }

    static Raster2dFenceState wait(void* context, uint32_t sequence, uint64_t) noexcept {
        auto& self = *static_cast<ComputeExecutor*>(context);
        ++self.waits;

        // A hung compute queue: the wait budget runs out, nothing executes.
        if (self.hung) {
            return Raster2dFenceState::pending;
        }

        self.drain(sequence);
        return sequence <= self.executed ? Raster2dFenceState::signalled : Raster2dFenceState::failed;
    }

    Raster2dSubmitter submitter() { return {this, submit, query, wait}; }
};

uint32_t cpu_rop(uint32_t rop3, uint32_t p, uint32_t s, uint32_t d) {
    uint32_t result = 0;

    for (uint32_t bit = 0; bit < 32; ++bit) {
        const uint32_t index = (((p >> bit) & 1u) << 2) | (((s >> bit) & 1u) << 1) | ((d >> bit) & 1u);
        result |= ((rop3 >> index) & 1u) << bit;
    }

    return result;
}

Image snapshot_primary(Fixture& fixture) {
    Image image(primary_width, primary_height, 0);

    for (uint32_t y = 0; y < primary_height; ++y) {
        for (uint32_t x = 0; x < primary_width; ++x) {
            image.at(x, y) = read_pixel(fixture.primary().bytes, size_t{y} * primary_pitch + x * 4u);
        }
    }

    return image;
}

bool inside_any(const std::vector<abi::Rect>& rects, int32_t x, int32_t y) {
    for (const auto& rect : rects) {
        if (x >= rect.left && x < rect.right && y >= rect.top && y < rect.bottom) {
            return true;
        }
    }

    return false;
}

abi::RectList rect_list(const std::vector<abi::Rect>& rects) { return Client::list(rects); }

template <typename Command, typename Encode>
bool raster_call(Client& client, uint32_t code, uint32_t bytes, const Command& command, Encode encode,
                 abi::Receipt& receipt) {
    return client.receipt(client.call(code, bytes, [&](const rx7900::sdma::CommandBuffer& out) {
                              return encode(out, command);
                          }), receipt) && receipt.outcome == abi::outcome_completed;
}

abi::Operand staging_operand(uint32_t offset, uint32_t pitch, uint32_t width, uint32_t height, uint32_t format) {
    abi::Operand value{};
    value.kind = abi::operand_staging;
    value.handle = offset;
    value.pitch = pitch;
    value.width = width;
    value.height = height;
    value.format = format;
    return value;
}


// ---------------------------------------------------------------------------
// Guarded StretchBltROP (0xa17) and pattern lines end to end: the radeondisp
// plan (radeon_display.hpp setup/call/sample helpers, one band per DIB call,
// source and mask spans staged exactly as the DLL stages them) through the
// dispatcher, backend, Raster2dEngine IBs and interpreted kernels, against the
// tests/gdi_engine_reference.hpp port of the pinned engine fallback.
// ---------------------------------------------------------------------------
namespace ref = gdi_reference;
namespace display = rx7900::reactos::radeon_display;

ref::Bitmap32 to_bitmap(const Image& image) {
    ref::Bitmap32 b(int32_t(image.width), int32_t(image.height));
    b.px = image.pixels;
    return b;
}

// Stages rows of 32-bit values (pitch 4 * w) at offset.
void stage_block(Client& client, uint32_t offset, uint32_t w, uint32_t h, const std::vector<uint32_t>& values) {
    for (uint32_t i = 0; i < w * h; ++i) {
        client.stage(offset + i * 4u, values[i]);
    }
}

// Stages a 1bpp block (MSB first, pitch 4 * w bytes) from a reference bitmap.
void stage_bits(Client& client, uint32_t offset, const ref::Bitmap1& bits, int32_t x0, int32_t y0, uint32_t w,
                uint32_t h) {
    const uint32_t pitch = w * 4u;
    std::vector<uint8_t> bytes(size_t(pitch) * h, 0);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            if (bits.get(x0 + int32_t(x), y0 + int32_t(y)) != 0) {
                bytes[size_t(y) * pitch + x / 8] = uint8_t(bytes[size_t(y) * pitch + x / 8] | (0x80u >> (x & 7)));
            }
        }
    }
    for (size_t i = 0; i < bytes.size(); i += 4) {
        client.stage(offset + i, uint32_t(bytes[i]) | (uint32_t(bytes[i + 1]) << 8) | (uint32_t(bytes[i + 2]) << 16) |
                                     (uint32_t(bytes[i + 3]) << 24));
    }
}

enum class StretchKind { dib, mono, device };

struct StretchE2E final {
    ref::Rect destination, source;
    ref::Clip clip;
    uint32_t rop4;
    StretchKind kind;
    bool mask;
    bool tile;
    ref::Point origin;
};

// Returns true when the GPU result equals the engine fallback. submitted is
// false when the DLL plan sends nothing (no driver call, nothing drawn, or a
// refusal that returns FALSE to the engine before touching pixels).
bool run_stretch_e2e(Fixture& fixture, Client& client, const StretchE2E& c, std::mt19937& rng, bool& submitted) {
    submitted = false;
    const ref::DriverStretch drv = ref::int_eng_stretch_blt(c.destination, c.source, c.clip);
    if (drv.bitblt || drv.nothing) {
        return true;
    }

    ref::Bitmap32 dib(41, 33);
    for (auto& v : dib.px) v = rng();
    ref::Bitmap1 mono(41, 33), mask(37, 29);
    for (int32_t y = 0; y < 33; ++y) for (int32_t x = 0; x < 41; ++x) mono.set(x, y, (rng() & 1) != 0);
    for (int32_t y = 0; y < 29; ++y) for (int32_t x = 0; x < 37; ++x) mask.set(x, y, (rng() % 4) == 0);
    std::vector<uint32_t> tile(64);
    for (auto& v : tile) v = rng();

    const Image before = snapshot_primary(fixture);
    ref::Bitmap32 screen = to_bitmap(before);
    ref::Bitmap32 expected = screen;
    ref::Source source{};
    if (c.kind == StretchKind::dib) {
        source.colour = &dib;
    } else if (c.kind == StretchKind::mono) {
        source.mono = &mono;
        source.mono0 = 0x00203040u;
        source.mono1 = 0x00f0e0d0u;
    } else {
        source.colour = &screen;
        source.device = true;
    }
    ref::Brush brush{};
    brush.solid = c.tile ? 0xffffffffu : 0x0055aa33u;
    brush.tile = c.tile ? tile.data() : nullptr;
    ref::eng_stretch_blt_rop(expected, source, c.mask ? &mask : nullptr, &c.clip, &c.origin,
                             ref::fallback_destination(drv), drv.input, &brush, c.rop4);

    // radeondisp plan.
    using Rect = display::Rectangle;
    const Rect bounds{c.clip.bounds.left, c.clip.bounds.top, c.clip.bounds.right, c.clip.bounds.bottom};
    display::StretchRopSetup setup{};
    if (!display::stretch_rop_setup({drv.output.left, drv.output.top, drv.output.right, drv.output.bottom},
                                    {drv.input.left, drv.input.top, drv.input.right, drv.input.bottom}, drv.flip_x,
                                    drv.flip_y, c.rop4, &bounds, setup)) {
        return primary_matches(fixture, Image(before));
    }

    Rect domain{0, 0, 41, 33};
    int32_t mask_dx = 0, mask_dy = 0;
    if (c.kind == StretchKind::device && setup.uses_source) {
        domain = setup.domain;
        mask_dx = domain.left;
        mask_dy = domain.top;
        if (domain.left < 0 || domain.top < 0 || domain.right > int32_t(primary_width) ||
            domain.bottom > int32_t(primary_height) ||
            !rx7900::reactos::framebuffer::empty(rx7900::reactos::framebuffer::intersect(domain, setup.output))) {
            return true;   // live-window subset only; planned_stretch_end_to_end covers snapshots
        }
    }
    if (setup.uses_source && (setup.source_width < 0 || setup.source_height < 0)) {
        return true;       // signed extents: planned_stretch_end_to_end
    }

    std::vector<Rect> visits;
    if (c.clip.kind == ref::Complexity::trivial) {
        visits.push_back(setup.output);
    } else if (c.clip.kind == ref::Complexity::rect) {
        visits.push_back(bounds);
    } else {
        for (const ref::Rect& r : c.clip.rects) visits.push_back({r.left, r.top, r.right, r.bottom});
    }

    const Rect surface{0, 0, int32_t(primary_width), int32_t(primary_height)};
    bool ok = true;
    for (const Rect& visit : visits) {
        display::StretchCall call{};
        if (!display::stretch_rop_call(setup, c.clip.kind == ref::Complexity::trivial ? nullptr : &visit, call)) {
            continue;
        }
        const Rect visible = rx7900::reactos::framebuffer::intersect(call.destination, surface);
        if (rx7900::reactos::framebuffer::empty(visible)) {
            continue;
        }
        const display::StretchAxis ax = display::stretch_axis_x(call, setup.flip_x);
        const display::StretchAxis ay = display::stretch_axis_y(call, setup.flip_y);
        int64_t xl = 0, xh = 0, yl = 0, yh = 0;
        display::stretch_samples(ax, visible.left, visible.right, xl, xh);
        display::stretch_samples(ay, visible.top, visible.bottom, yl, yh);

        abi::StretchRopCommand command{};
        command.session = client.session;
        command.target = client.primary;
        command.rop4 = c.rop4;
        const bool tiled = c.tile && display::rop4_uses_pattern(c.rop4);
        command.flags = (setup.flip_x ? abi::stretch_flag_mirror_x : 0u) | (setup.flip_y ? abi::stretch_flag_mirror_y : 0u) |
                        (tiled ? abi::stretch_flag_tile : 0u);
        command.brush_colour = tiled ? 0u : brush.solid;
        command.brush_x = c.origin.x;
        command.brush_y = c.origin.y;
        for (uint32_t i = 0; tiled && i < 64; ++i) command.pattern[i] = tile[i];
        command.bounds = {call.destination.left, call.destination.top, call.destination.right, call.destination.bottom};
        command.rects = rect_list({{visible.left, visible.top, visible.right, visible.bottom}});
        uint32_t cursor = 0;
        int32_t ox = 0, oy = 0;
        if (setup.uses_source && c.kind == StretchKind::device) {
            command.source.kind = abi::operand_surface;
            command.source.handle = client.primary;
            command.source.format = abi::format_xrgb8888;
            command.window = {domain.left, domain.top, domain.right, domain.bottom};
        } else if (setup.uses_source) {
            int64_t fx = 0, fy = 0;
            int64_t w = display::clamp_span(xl, xh, domain.left, domain.right, fx);
            int64_t h = display::clamp_span(yl, yh, domain.top, domain.bottom, fy);
            std::vector<uint32_t> block;
            if (w == 0 || h == 0) {
                fx = xh + 1;
                fy = yh + 1;
                w = h = 1;
                block.assign(1, 0);
            }
            ox = int32_t(fx);
            oy = int32_t(fy);
            if (c.kind == StretchKind::mono && block.empty()) {
                stage_bits(client, 0, mono, ox, oy, uint32_t(w), uint32_t(h));
            } else {
                if (block.empty()) {
                    for (int64_t y = 0; y < h; ++y) for (int64_t x = 0; x < w; ++x) block.push_back(dib.get(ox + int32_t(x), oy + int32_t(y)));
                }
                stage_block(client, 0, uint32_t(w), uint32_t(h), block);
            }
            const bool mono_operand = c.kind == StretchKind::mono;
            command.source = staging_operand(0, uint32_t(w) * 4u, uint32_t(w), uint32_t(h),
                                             mono_operand ? abi::format_mono1 : abi::format_xrgb8888);
            command.window = {0, 0, int32_t(w), int32_t(h)};
            command.mono_foreground = mono_operand ? source.mono1 : 0u;
            command.mono_background = mono_operand ? source.mono0 : 0u;
            cursor = uint32_t(w * h * 4);
        }
        if (setup.uses_source) {
            command.map = {call.source.left - ox, call.source.top - oy, call.source.right - call.source.left,
                           call.source.bottom - call.source.top};
        }
        if (c.mask) {
            int64_t mx = 0, my = 0;
            const int64_t w = display::clamp_span(xl - mask_dx, xh - mask_dx, 0, mask.cx, mx);
            const int64_t h = display::clamp_span(yl - mask_dy, yh - mask_dy, 0, mask.cy, my);
            if (w == 0 || h == 0) {
                continue;   // every sample outside the mask: nothing drawn
            }
            stage_bits(client, cursor, mask, int32_t(mx), int32_t(my), uint32_t(w), uint32_t(h));
            command.mask = staging_operand(cursor, uint32_t(w) * 4u, uint32_t(w), uint32_t(h), abi::format_mono1);
            command.mask_x = ox - mask_dx - int32_t(mx);
            command.mask_y = oy - mask_dy - int32_t(my);
        }
        abi::Receipt receipt{};
        ok = ok && raster_call(client, abi::ioctl_stretch_rop, abi::stretch_rop_request_bytes, command,
                               abi::encode_stretch_rop, receipt) && client.sync(0, receipt);
        submitted = true;
    }

    Image want(primary_width, primary_height, 0);
    want.pixels = expected.px;
    return ok && primary_matches(fixture, want);
}

// ---------------------------------------------------------------------------
// The radeondisp guarded-stretch plan itself (radeon_display.hpp
// stretch_call_bands, stretch_band_command, stage_source_row,
// write_translation_table, snapshot_rects and copy_mono_bits, in the order
// accelerate_stretch_rop uses them) through the dispatcher, backend and
// interpreted kernels, against the gdi_reference port of the engine fallback:
// device windows leaving the surface and same-surface overlaps (GPU snapshot),
// negative source extents, masked tiles (running PatternX), 8/16/24bpp
// translated sources and bands split into column strips.
// ---------------------------------------------------------------------------
// demoted: a device bitmap whose pixels live in a driver DIB; the engine still
// reads it through an IntEngEnter window, which radeondisp stages.
enum class Planned { dib32, mono, device, index8, rgb16, rgb24, demoted };

struct PlannedCase final {
    ref::Rect destination, source;
    ref::Clip clip;
    uint32_t rop4 = 0xcccc;
    Planned kind = Planned::dib32;
    bool mask = false;
    bool tile = false;
    ref::Point origin{};
    uint32_t half_limit = 0;   // planner half bytes (0: the session's half)
    uint32_t variant = 0;      // XLATEOBJ flavour of a translated source
};

struct PlannedOutcome final {
    bool called = false;       // the DLL would plan this call (no bitblt, no NOOP)
    bool declined = false;     // the plan returns FALSE to the engine (nothing sent)
    bool exact = false;
    bool snapshot = false;
    bool negative = false;
    bool running = false;
    bool split = false;
    uint32_t bands = 0;
};

ref::Xlate planned_xlate(Planned kind, uint32_t variant, std::mt19937& rng) {
    ref::Xlate x{};
    if (kind == Planned::index8) {
        x.kind = ref::Xlate::Kind::table;
        x.table.resize(variant % 2 == 0 ? 256 : 200);   // cEntries below 256: high indices read 0
        for (auto& v : x.table) v = rng();
    } else if (kind == Planned::rgb16) {
        x.kind = variant % 2 == 0 ? ref::Xlate::Kind::x555_to_bgr : ref::Xlate::Kind::x565_to_bgr;
    } else if (kind == Planned::rgb24) {
        if (variant % 3 == 0) {
            x.kind = ref::Xlate::Kind::rgb_to_bgr;
        } else if (variant % 3 == 1) {
            x.kind = ref::Xlate::Kind::shift_and_mask;   // a bitfield source with rotated channels
            x.masks[0] = 0x00ff0000u; x.masks[1] = 0x0000ff00u; x.masks[2] = 0x000000ffu;
            x.shifts[0] = 8; x.shifts[1] = 24; x.shifts[2] = 16;
        } else {
            x.kind = ref::Xlate::Kind::trivial;
        }
    }
    return x;
}

// radeondisp keeps one snapshot surface per PDEV and reuses it while it is big
// enough (acquire_scratch); its stale pixels must be overwritten every time.
struct ScratchCache final {
    const Client* client = nullptr;
    uint32_t surface = 0, width = 0, height = 0;
};
ScratchCache scratch_caches[2];

ScratchCache& scratch_for(const Client& client) {
    for (ScratchCache& cache : scratch_caches) {
        if (cache.client == &client) return cache;
    }
    for (ScratchCache& cache : scratch_caches) {
        if (cache.client == nullptr) { cache.client = &client; return cache; }
    }
    return scratch_caches[0];
}

void release_scratch_cache(Client& client) {
    ScratchCache& cache = scratch_for(client);
    if (cache.surface != 0) {
        abi::Receipt r{};
        client.destroy(cache.surface, r);
    }
    cache = {};
}

PlannedOutcome run_planned_stretch(Fixture& fixture, Client& client, const PlannedCase& c, std::mt19937& rng) {
    PlannedOutcome out{};
    const ref::DriverStretch drv = ref::int_eng_stretch_blt(c.destination, c.source, c.clip);
    if (drv.bitblt || drv.nothing) {
        return out;
    }

    const int32_t bw = 41, bh = 33;
    ref::Bitmap32 dib(bw, bh);
    for (auto& v : dib.px) v = rng();
    ref::Bitmap1 mono(bw, bh), mask(37, 29);
    for (int32_t y = 0; y < bh; ++y) for (int32_t x = 0; x < bw; ++x) mono.set(x, y, (rng() & 1) != 0);
    for (int32_t y = 0; y < mask.cy; ++y) for (int32_t x = 0; x < mask.cx; ++x) mask.set(x, y, (rng() % 3) == 0);
    const uint32_t bpp = c.kind == Planned::index8 ? 1u : c.kind == Planned::rgb16 ? 2u : 3u;
    ref::BitmapBytes bytes(bw, bh, bpp);
    for (auto& b : bytes.bits) b = uint8_t(rng());
    const ref::Xlate xlate = planned_xlate(c.kind, c.variant, rng);
    std::vector<uint32_t> tile(64);
    for (auto& v : tile) v = rng();

    const Image before = snapshot_primary(fixture);
    ref::Bitmap32 screen = to_bitmap(before);
    ref::Bitmap32 expected = screen;
    ref::Source source{};
    if (c.kind == Planned::dib32) {
        source.colour = &dib;
    } else if (c.kind == Planned::mono) {
        source.mono = &mono;
        source.mono0 = 0x00203040u;
        source.mono1 = 0x00f0e0d0u;
    } else if (c.kind == Planned::device) {
        source.colour = &screen;
        source.device = true;
    } else if (c.kind == Planned::demoted) {
        source.colour = &dib;
        source.device = true;
    } else {
        source.bytes = &bytes;
        source.xlate = &xlate;
    }
    const bool tiled = c.tile && display::rop4_uses_pattern(c.rop4);
    ref::Brush brush{};
    brush.solid = c.tile ? 0xffffffffu : 0x0055aa33u;
    brush.tile = c.tile ? tile.data() : nullptr;
    ref::eng_stretch_blt_rop(expected, source, c.mask ? &mask : nullptr, &c.clip, &c.origin,
                             ref::fallback_destination(drv), drv.input, &brush, c.rop4);

    using Rect = display::Rectangle;
    namespace fb = rx7900::reactos::framebuffer;
    const Rect bounds{c.clip.bounds.left, c.clip.bounds.top, c.clip.bounds.right, c.clip.bounds.bottom};
    display::StretchRopSetup setup{};
    if (!display::stretch_rop_setup({drv.output.left, drv.output.top, drv.output.right, drv.output.bottom},
                                    {drv.input.left, drv.input.top, drv.input.right, drv.input.bottom}, drv.flip_x,
                                    drv.flip_y, c.rop4, &bounds, setup)) {
        out.exact = primary_matches(fixture, before);
        return out;
    }

    out.called = true;
    out.negative = setup.uses_source && (setup.source_width < 0 || setup.source_height < 0);
    const Rect surface{0, 0, int32_t(primary_width), int32_t(primary_height)};
    display::StretchSourcePlan plan{};
    if (setup.uses_source) {
        const Rect bitmap{0, 0, bw, bh};
        if (c.kind == Planned::device) {
            plan.operand = display::StretchOperand::surface;
            plan.surface = client.primary;
            plan.domain = setup.domain;
            plan.backed = fb::intersect(plan.domain, surface);
            plan.mask_dx = setup.domain.left;
            plan.mask_dy = setup.domain.top;
        } else if (c.kind == Planned::demoted) {
            plan.operand = display::StretchOperand::staged;
            plan.format = abi::format_xrgb8888;
            plan.domain = setup.domain;
            plan.backed = fb::intersect(plan.domain, bitmap);
            plan.mask_dx = setup.domain.left;
            plan.mask_dy = setup.domain.top;
            if (fb::empty(plan.domain)) {
                out.declined = true;
                out.exact = primary_matches(fixture, before);
                return out;
            }
        } else {
            plan.operand = display::StretchOperand::staged;
            plan.format = c.kind == Planned::dib32 ? abi::format_xrgb8888 : c.kind == Planned::mono ? abi::format_mono1 :
                          c.kind == Planned::index8 ? abi::format_index8 : c.kind == Planned::rgb16 ? abi::format_rgb16 :
                                                                             abi::format_rgb24;
            plan.domain = bitmap;
            plan.backed = bitmap;
        }
    }
    if (c.mask) {
        plan.mask_width = mask.cx;
        plan.mask_height = mask.cy;
        plan.running = tiled;
    }
    // accelerate_stretch_rop's admission and policies.
    const bool rgb16_small = c.kind == Planned::rgb16 && setup.destination_width * setup.destination_height < 4096;
    if (!display::stretch_rop_admitted(c.rop4, display::SurfaceKind::primary,
                                       setup.uses_source ? display::RasterSource::bitmap32 : display::RasterSource::none,
                                       c.mask, c.mask, tiled, tiled) ||
        rgb16_small || std::abs(setup.source_width) > 16384 || std::abs(setup.source_height) > 16384) {
        out.declined = true;
        out.exact = primary_matches(fixture, before);
        return out;
    }

    const uint32_t session_half = display::staging_half_bytes(client.staging_bytes);
    const uint32_t half = c.half_limit != 0 && c.half_limit < session_half ? c.half_limit : session_half;
    std::vector<Rect> visits;
    if (c.clip.kind == ref::Complexity::trivial) {
        visits.push_back(setup.output);
    } else if (c.clip.kind == ref::Complexity::rect) {
        visits.push_back(bounds);
    } else {
        for (const ref::Rect& r : c.clip.rects) visits.push_back({r.left, r.top, r.right, r.bottom});
    }

    bool snapshot = plan.operand == display::StretchOperand::surface && !display::stretch_window_on_surface(plan);
    uint32_t scratch = 0;
    bool ok = true;
    for (uint32_t pass = 0; pass < 2 && ok; ++pass) {
        if (pass == 1 && snapshot) {
            const int32_t w = plan.domain.right - plan.domain.left, h = plan.domain.bottom - plan.domain.top;
            ScratchCache& cache = scratch_for(client);
            if (cache.surface == 0 || cache.width < uint32_t(w) || cache.height < uint32_t(h)) {
                release_scratch_cache(client);
                ScratchCache& fresh = scratch_for(client);
                abi::SurfaceGrant grant{};
                ok = client.create((uint32_t(w) + 63u) & ~63u, (uint32_t(h) + 63u) & ~63u, grant);
                fresh.surface = grant.surface;
                fresh.width = grant.width;
                fresh.height = grant.height;
                // A reused allocation carries old pixels: model that up front.
                abi::Receipt r{};
                ok = ok && client.fill(fresh.surface, 0xdeadbeefu, {{0, 0, int32_t(grant.width), int32_t(grant.height)}}, r);
            }
            scratch = scratch_for(client).surface;
            Rect zeros[4]{};
            Rect copy{};
            const uint32_t count = display::snapshot_rects(plan.domain, plan.backed, zeros, copy);
            std::vector<abi::Rect> zero_rects;
            for (uint32_t i = 0; i < count; ++i) zero_rects.push_back({zeros[i].left, zeros[i].top, zeros[i].right, zeros[i].bottom});
            abi::Receipt r{};
            ok = ok && (zero_rects.empty() || (client.fill(scratch, 0, zero_rects, r) && r.outcome == abi::outcome_completed));
            ok = ok && (fb::empty(copy) || (client.copy(scratch, client.primary, plan.domain.left, plan.domain.top,
                                                        {{copy.left, copy.top, copy.right, copy.bottom}}, r) &&
                                            r.outcome == abi::outcome_completed));
            plan.surface = scratch;
            plan.origin_x = plan.domain.left;
            plan.origin_y = plan.domain.top;
            out.snapshot = true;
        }

        for (const Rect& visit : visits) {
            display::StretchCall call{};
            if (!display::stretch_rop_call(setup, c.clip.kind == ref::Complexity::trivial ? nullptr : &visit, call)) {
                continue;
            }
            const Rect visible = fb::intersect(call.destination, surface);
            if (fb::empty(visible)) {
                continue;
            }
            if (pass == 0 && plan.operand == display::StretchOperand::surface && display::stretch_call_meets_window(plan, call)) {
                snapshot = true;
            }
            const display::StretchPlanResult planned = display::stretch_call_bands(setup, call, plan, visible, half,
                [&](const display::StretchBand& band) {
                    if (pass == 0) {
                        return true;
                    }
                    out.split = out.split || band.rows.left != visible.left || band.rows.right != visible.right;
                    abi::StretchRopCommand command{};
                    command.session = client.session;
                    command.target = client.primary;
                    command.rop4 = c.rop4;
                    command.flags = tiled ? abi::stretch_flag_tile : 0u;
                    command.brush_colour = tiled ? 0u : brush.solid;
                    command.brush_x = c.origin.x;
                    command.brush_y = c.origin.y;
                    for (uint32_t i = 0; tiled && i < 64; ++i) command.pattern[i] = tile[i];
                    display::stretch_band_command(setup, call, plan, band, command);
                    if (plan.operand == display::StretchOperand::staged && plan.format == abi::format_mono1) {
                        command.mono_foreground = source.mono1;
                        command.mono_background = source.mono0;
                    }
                    out.running = out.running || (command.flags & abi::stretch_flag_running_phase) != 0;
                    uint8_t* base = client.staging;
                    if (plan.operand == display::StretchOperand::staged) {
                        const display::StretchStage& stage = band.source;
                        const uint32_t table = abi::staging_table_bytes(plan.format);
                        if (table != 0) {
                            display::write_translation_table(plan.format, [&](uint32_t v) { return xlate(v); },
                                rx7900::sdma::CommandBuffer(base + band.table_offset, table));
                        }
                        for (int32_t row = 0; row < stage.height; ++row) {
                            uint8_t* dst = base + stage.offset + size_t(row) * stage.pitch;
                            if (c.kind == Planned::mono) {
                                if (!stage.zero) {
                                    display::copy_mono_bits(&mono.bits[size_t(stage.y + row) * mono.stride],
                                                            uint32_t(stage.x), uint32_t(stage.width), dst);
                                } else {
                                    std::fill(dst, dst + stage.pitch, uint8_t(0));
                                }
                                continue;
                            }
                            const uint32_t pixel = display::staged_bytes_per_pixel(plan.format);
                            display::stage_source_row(plan, stage, row, pixel, [&](int32_t sy) {
                                return c.kind == Planned::dib32 || c.kind == Planned::demoted ?
                                    reinterpret_cast<const uint8_t*>(&dib.px[size_t(sy) * size_t(bw)]) :
                                    &bytes.bits[size_t(sy) * bytes.stride];
                            }, dst);
                        }
                    }
                    if (plan.mask_width != 0 && band.mask.height != 0) {
                        const display::StretchStage& stage = band.mask;
                        for (int32_t row = 0; row < stage.height; ++row) {
                            display::copy_mono_bits(&mask.bits[size_t(stage.y + row) * mask.stride], uint32_t(stage.x),
                                                    uint32_t(stage.width), base + stage.offset + size_t(row) * stage.pitch);
                        }
                    }
                    abi::Receipt receipt{};
                    const bool sent = raster_call(client, abi::ioctl_stretch_rop, abi::stretch_rop_request_bytes,
                                                  command, abi::encode_stretch_rop, receipt);
                    if (!sent) {
                        std::fprintf(stderr, "0xa17 refused (outcome %u): bounds %d,%d,%d,%d rows %d,%d,%d,%d src kind %u "
                                     "fmt %u %ux%u pitch %u map %d,%d,%d,%d window %d,%d,%d,%d flags %x\n",
                                     receipt.outcome, command.bounds.left, command.bounds.top, command.bounds.right,
                                     command.bounds.bottom, band.rows.left, band.rows.top, band.rows.right,
                                     band.rows.bottom, command.source.kind, command.source.format, command.source.width,
                                     command.source.height, command.source.pitch, command.map.left, command.map.top,
                                     command.map.width, command.map.height, command.window.left, command.window.top,
                                     command.window.right, command.window.bottom, command.flags);
                    }
                    ok = ok && sent && client.sync(0, receipt);
                    ++out.bands;
                    return ok;
                });
            if (planned == display::StretchPlanResult::unsupported) {
                if (pass == 0) {
                    out.declined = true;
                    out.exact = primary_matches(fixture, before);
                    return out;
                }
                ok = false;
            }
        }
    }


    Image want(primary_width, primary_height, 0);
    want.pixels = expected.px;
    out.exact = ok && primary_matches(fixture, want);
    return out;
}

void planned_stretch_end_to_end(Fixture& fixture, Client& client, Client* large) {
    std::mt19937 rng(0x5a17e2e);
    ref::Clip complex{};
    complex.kind = ref::Complexity::complex;
    complex.bounds = {4, 2, 300, 190};
    complex.rects = {{4, 2, 90, 40}, {100, 2, 300, 40}, {10, 40, 220, 120}, {4, 120, 60, 190}, {80, 120, 300, 190}};
    ref::Clip rect{};
    rect.kind = ref::Complexity::rect;
    rect.bounds = {6, 3, 280, 170};
    ref::Clip narrow{};   // cuts far into the output: negative source extents after the pre-adjust
    narrow.kind = ref::Complexity::rect;
    narrow.bounds = {60, 50, 120, 110};
    ref::Clip dc_trivial{};   // DC_TRIVIAL with real rclBounds: EngStretchBltROP pre-adjusts unscaled, so a
    dc_trivial.bounds = {30, 20, 70, 60};   // wide output over a narrow source turns the source extent negative
    const ref::Clip clips[] = {ref::Clip{}, rect, complex, narrow, dc_trivial};
    const uint32_t rops[] = {0xcccc, 0x6666, 0xc0c0, 0x8888, 0xf0f0, 0xb8b8, 0x5a5a};
    const Planned kinds[] = {Planned::dib32, Planned::mono, Planned::device, Planned::index8, Planned::rgb24,
                             Planned::device, Planned::demoted};
    unsigned called = 0, exact = 0, declined = 0, snapshots = 0, negatives = 0, running = 0, splits = 0;
    unsigned outside = 0, kinds_seen = 0, translated = 0, demoted_outside = 0, overlaps = 0;
    for (uint32_t trial = 0; trial < 490; ++trial) {
        PlannedCase c{};
        c.kind = kinds[trial % 7];
        const bool dx_neg = (trial & 1) != 0, dy_neg = (trial & 2) != 0;
        const int32_t w = 9 + int32_t(rng() % 90), h = 7 + int32_t(rng() % 70);
        const int32_t x = int32_t(rng() % 120) - 40, y = int32_t(rng() % 80) - 30;
        c.destination = {dx_neg ? x + w : x, dy_neg ? y + h : y, dx_neg ? x : x + w, dy_neg ? y : y + h};
        const int32_t sw = 2 + int32_t(rng() % 36), sh = 2 + int32_t(rng() % 30);
        int32_t sx = int32_t(rng() % 24) - 3, sy = int32_t(rng() % 18) - 3;
        if (c.kind == Planned::device) {
            // Windows straddling every edge of the 320x200 primary, and inside it.
            sx = (trial % 4 == 0) ? 300 + int32_t(rng() % 40) : (trial % 4 == 1) ? -20 + int32_t(rng() % 15) :
                 int32_t(rng() % 280);
            sy = (trial % 5 == 0) ? 185 + int32_t(rng() % 30) : (trial % 5 == 1) ? -15 + int32_t(rng() % 10) :
                 int32_t(rng() % 170);
            if (trial % 4 >= 2 && trial % 5 >= 2) {
                // Inside the surface and over the output: the same-surface snapshot.
                sx = std::max(0, std::min(x + int32_t(rng() % 30), 280));
                sy = std::max(0, std::min(y + int32_t(rng() % 30), 165));
            }
        }
        c.source = {sx, sy, sx + sw, sy + sh};
        if ((trial & 4) != 0) c.source = {sx + sw, sy, sx, sy + sh};
        c.clip = clips[(trial / 6) % 5];
        c.rop4 = rops[rng() % 7];
        c.tile = (trial % 3) == 1 || (trial % 4) == 2;
        c.mask = (trial % 4) == 2 && display::rop4_uses_source(c.rop4);
        c.origin = {int32_t(rng() % 17) - 8, int32_t(rng() % 17) - 8};
        c.half_limit = (trial % 5) >= 3 && (c.kind == Planned::dib32 || c.kind == Planned::mono || c.kind == Planned::demoted) ?
                       16u + (rng() % 16u) * 4u : 0u;
        c.variant = rng();
        const PlannedOutcome o = run_planned_stretch(fixture, client, c, rng);
        if (!o.called) continue;
        if (!o.exact) {
            std::printf("planned stretch %u: kind %d mask %d tile %d rop %04x clip %d half %u dst %d,%d,%d,%d "
                        "src %d,%d,%d,%d snapshot %d declined %d\n", trial, int(c.kind), c.mask, c.tile, c.rop4,
                        int(c.clip.kind), c.half_limit, c.destination.left, c.destination.top, c.destination.right,
                        c.destination.bottom, c.source.left, c.source.top, c.source.right, c.source.bottom,
                        o.snapshot, o.declined);
        }
        ++called;
        exact += o.exact ? 1u : 0u;
        declined += o.declined ? 1u : 0u;
        snapshots += o.snapshot ? 1u : 0u;
        negatives += o.negative && !o.declined ? 1u : 0u;
        running += o.running ? 1u : 0u;
        splits += o.split ? 1u : 0u;
        translated += (c.kind == Planned::index8 || c.kind == Planned::rgb24) && !o.declined ? 1u : 0u;
        kinds_seen |= o.declined ? 0u : 1u << uint32_t(c.kind);
        const ref::DriverStretch drv = ref::int_eng_stretch_blt(c.destination, c.source, c.clip);
        ref::Rect domain = drv.input;
        ref::make_well_ordered(domain);
        outside += c.kind == Planned::device && o.snapshot &&
                   (domain.left < 0 || domain.top < 0 || domain.right > int32_t(primary_width) ||
                    domain.bottom > int32_t(primary_height)) ? 1u : 0u;
        overlaps += c.kind == Planned::device && o.snapshot && o.exact && domain.left >= 0 && domain.top >= 0 &&
                    domain.right <= int32_t(primary_width) && domain.bottom <= int32_t(primary_height) ? 1u : 0u;
        demoted_outside += c.kind == Planned::demoted && o.exact && !o.declined &&
                           (domain.left < 0 || domain.top < 0 || domain.right > 41 || domain.bottom > 33) ? 1u : 0u;
    }
    std::printf("planned stretch: %u calls, %u exact, %u declined, %u snapshots (%u windows off the surface), "
                "%u negative extents, %u running phases, %u column splits, %u translated, kinds %x, "
                "%u staged device windows off their bitmap, %u same-surface overlap snapshots\n",
                called, exact, declined, snapshots, outside, negatives, running, splits, translated, kinds_seen,
                demoted_outside, overlaps);
    expect(called > 300 && exact == called && declined * 10 < called && snapshots > 30 && outside > 10 &&
               negatives > 10 && running > 10 && splits > 5 && translated > 40 && kinds_seen == 0x6fu && demoted_outside > 10 && overlaps > 10,
           "shared radeondisp stretch plan equals EngStretchBltROP (snapshots, signed extents, masked tiles, "
           "translated sources, column strips)");

    // Masked tiles whose DIB call starts left of the surface (an unclipped
    // CLIPOBJ): PatternX counts the drawn pixels the engine draws into its
    // temporary at x < 0, so the staged mask must reach back to the call's edge.
    unsigned left_runs = 0, left_exact = 0;
    for (uint32_t trial = 0; trial < 40; ++trial) {
        PlannedCase c{};
        c.kind = (trial & 1) != 0 ? Planned::mono : Planned::dib32;
        const int32_t x = -60 + int32_t(rng() % 55), w = 70 + int32_t(rng() % 120);
        const int32_t y = int32_t(rng() % 60), h = 8 + int32_t(rng() % 30);
        c.destination = (trial & 2) != 0 ? ref::Rect{x + w, y, x, y + h} : ref::Rect{x, y, x + w, y + h};
        const int32_t sx = int32_t(rng() % 8), sy = int32_t(rng() % 8);
        c.source = {sx, sy, sx + 8 + int32_t(rng() % 25), sy + 4 + int32_t(rng() % 20)};
        c.clip = ref::Clip{};
        c.rop4 = (trial & 4) != 0 ? 0xb8b8u : 0xc0c0u;
        c.tile = true;
        c.mask = true;
        c.origin = {int32_t(rng() % 17) - 8, int32_t(rng() % 17) - 8};
        const PlannedOutcome o = run_planned_stretch(fixture, client, c, rng);
        if (o.called && o.running && !o.declined) {
            ++left_runs;
            left_exact += o.exact ? 1u : 0u;
        }
    }
    std::printf("masked tiles left of the surface: %u of %u exact\n", left_exact, left_runs);
    expect(left_runs > 30 && left_exact == left_runs,
           "running PatternX counts from the DIB call's left edge even outside the surface");

    // rgb16 sources need a 256 KiB table: the 64 KiB session declines them
    // (exactly), a 1 MiB session draws them.
    unsigned big_calls = 0, big_exact = 0, big_drawn = 0, small_declined = 0, small_calls = 0;
    for (uint32_t trial = 0; trial < 40; ++trial) {
        PlannedCase c{};
        c.kind = Planned::rgb16;
        const int32_t w = 70 + int32_t(rng() % 120), h = 60 + int32_t(rng() % 100);
        const int32_t x = int32_t(rng() % 60) - 10, y = int32_t(rng() % 30) - 10;
        c.destination = (trial & 1) != 0 ? ref::Rect{x + w, y, x, y + h} : ref::Rect{x, y, x + w, y + h};
        const int32_t sx = int32_t(rng() % 10), sy = int32_t(rng() % 10);
        c.source = {sx, sy, sx + 5 + int32_t(rng() % 30), sy + 5 + int32_t(rng() % 20)};
        c.clip = clips[trial % 3];   // no narrow clip: keep rgb16 calls above the CPU-table policy size
        const uint32_t source_rops[] = {0xcccc, 0x6666, 0xc0c0, 0x8888, 0xb8b8};
        c.rop4 = source_rops[rng() % 5];
        c.tile = (trial % 3) == 1;
        c.mask = (trial % 4) == 2 && display::rop4_uses_source(c.rop4);
        c.origin = {int32_t(rng() % 9) - 4, int32_t(rng() % 9) - 4};
        c.variant = trial;
        const PlannedOutcome small = run_planned_stretch(fixture, client, c, rng);
        if (small.called) {
            ++small_calls;
            small_declined += small.declined && small.exact ? 1u : 0u;
        }
        if (large != nullptr) {
            const PlannedOutcome o = run_planned_stretch(fixture, *large, c, rng);
            if (o.called) {
                ++big_calls;
                big_exact += o.exact ? 1u : 0u;
                big_drawn += o.exact && !o.declined ? 1u : 0u;
            }
        }
    }
    std::printf("rgb16 stretch: %u of %u declined exactly with 64 KiB staging, %u of %u exact (%u drawn) with 1 MiB\n",
                small_declined, small_calls, big_exact, big_calls, big_drawn);
    expect(small_calls > 20 && small_declined == small_calls && big_calls == small_calls && big_exact == big_calls &&
               big_drawn > 30,
           "rgb16 sources: 65536-entry XLATEOBJ tables translate exactly; too-small staging declines exactly");
    release_scratch_cache(client);
    if (large != nullptr) {
        release_scratch_cache(*large);
    }
}

// DrvBitBlt with 8/16/24bpp engine sources (radeondisp compute_bitblt staging:
// the XLATEOBJ table, then rows at the translated pitch) through 0xa10/0xa11,
// the backend and the interpreted rop kernel, against the generated
// DIB_32BPP_BitBlt_* semantics: S = XLATEOBJ_iXlate(raw pixel), P the tile at
// ((x - org.x) mod 8, (y - org.y) mod 8) or the solid colour, BltMask choosing
// the foreground ROP3 where the mask bit is 1.
void translated_bitblt_end_to_end(Fixture& fixture, Client& client) {
    std::mt19937 rng(0xb177);
    unsigned runs = 0, exact = 0;
    for (uint32_t trial = 0; trial < 36; ++trial) {
        const Planned kind = trial % 3 == 0 ? Planned::index8 : trial % 3 == 1 ? Planned::rgb16 : Planned::rgb24;
        const uint32_t bpp = kind == Planned::index8 ? 1u : kind == Planned::rgb16 ? 2u : 3u;
        const uint32_t format = kind == Planned::index8 ? abi::format_index8 : kind == Planned::rgb16 ? abi::format_rgb16 :
                                                                                  abi::format_rgb24;
        const ref::Xlate xlate = planned_xlate(kind, trial / 3, rng);
        const int32_t w = 64 + int32_t(rng() % 90), h = 64 + int32_t(rng() % 40);   // rgb16 policy: >= 4096 pixels
        const int32_t x = int32_t(rng() % 100), y = int32_t(rng() % 60);
        ref::BitmapBytes bits(w + 7, h + 5, bpp);
        for (auto& b : bits.bits) b = uint8_t(rng());
        const int32_t sx = int32_t(rng() % 7), sy = int32_t(rng() % 5);
        const bool masked = trial % 4 == 1;
        const bool tiled = trial % 4 >= 2;
        const uint32_t rops[] = {0xcccc, 0x6666, 0xb8b8, 0x8888};
        const uint32_t rop = rops[trial % 4];
        const uint32_t rop4 = masked ? 0xaa00u | (rop & 0xffu) : rop;    // mask bit 1: rop, bit 0: keep D
        ref::Bitmap1 mask(w, h);
        for (int32_t my = 0; my < h; ++my) for (int32_t mx = 0; mx < w; ++mx) mask.set(mx, my, (rng() % 3) != 0);
        std::vector<uint32_t> tile(64);
        for (auto& v : tile) v = rng();
        const ref::Point origin{int32_t(rng() % 9) - 4, int32_t(rng() % 9) - 4};

        const Image before = snapshot_primary(fixture);
        Image want = before;
        for (int32_t py = 0; py < h; ++py) {
            for (int32_t px = 0; px < w; ++px) {
                const uint32_t s = xlate(bits.get(sx + px, sy + py));
                const uint32_t p = tiled ? tile[size_t(((y + py - origin.y) & 7) * 8 + ((x + px - origin.x) & 7))] :
                                           0x00336699u;
                const uint32_t d = want.at(uint32_t(x + px), uint32_t(y + py));
                const uint32_t r = masked && mask.get(px, py) == 0 ? ((rop4 >> 8) & 0xffu) : (rop4 & 0xffu);
                want.at(uint32_t(x + px), uint32_t(y + py)) = cpu_rop(r, p, s, d);
            }
        }

        // Staging exactly as compute_bitblt: table, rows, then the mask.
        const uint32_t table = abi::staging_table_bytes(format);
        display::write_translation_table(format, [&](uint32_t v) { return xlate(v); },
                                         rx7900::sdma::CommandBuffer(client.staging, table));
        const uint32_t pitch = uint32_t(display::staged_pitch(format, w));
        for (int32_t row = 0; row < h; ++row) {
            uint8_t* dst = client.staging + table + size_t(row) * pitch;
            std::fill(dst, dst + pitch, uint8_t(0));
            std::copy_n(&bits.bits[size_t(sy + row) * bits.stride + size_t(sx) * bpp], size_t(w) * bpp, dst);
        }
        const uint32_t mask_offset = table + pitch * uint32_t(h);
        for (int32_t row = 0; masked && row < h; ++row) {
            display::copy_mono_bits(&mask.bits[size_t(row) * mask.stride], 0, uint32_t(w),
                                    client.staging + mask_offset + size_t(row) * uint32_t(w) * 4u);
        }
        abi::RopCommand command{};
        command.session = client.session;
        command.target = client.primary;
        command.rop4 = rop4;
        command.brush_colour = tiled ? 0u : 0x00336699u;
        command.brush_x = origin.x;
        command.brush_y = origin.y;
        command.bounds = {x, y, x + w, y + h};
        command.source = staging_operand(0, pitch, uint32_t(w), uint32_t(h), format);
        command.map = {0, 0, w, h};
        if (masked) {
            command.mask = staging_operand(mask_offset, uint32_t(w) * 4u, uint32_t(w), uint32_t(h), abi::format_mono1);
        }
        command.has_pattern = tiled;
        for (uint32_t i = 0; tiled && i < 64; ++i) command.pattern[i] = tile[i];
        command.rects = rect_list({{x, y, x + w, y + h}});
        abi::Receipt receipt{};
        const uint32_t code = tiled ? abi::ioctl_pattern : abi::ioctl_rop;
        const uint32_t bytes = tiled ? abi::pattern_request_bytes : abi::rop_request_bytes;
        const bool sent = raster_call(client, code, bytes, command, abi::encode_rop, receipt) && client.sync(0, receipt);
        ++runs;
        exact += sent && primary_matches(fixture, want) ? 1u : 0u;
    }
    std::printf("translated DrvBitBlt: %u of %u exact (index8/rgb16/rgb24, masks, tiles)\n", exact, runs);
    expect(runs == 36 && exact == runs, "8/16/24bpp DrvBitBlt sources translate through staged XLATEOBJ tables exactly");
}

void stretch_lines_end_to_end(Fixture& fixture, Client& client) {
    std::mt19937 rng(0xe2e5);
    ref::Clip complex{};
    complex.kind = ref::Complexity::complex;
    complex.bounds = {4, 2, 150, 120};
    complex.rects = {{4, 2, 60, 40}, {70, 2, 150, 40}, {10, 40, 120, 90}, {4, 90, 50, 120}, {60, 90, 150, 120}};
    ref::Clip rect{};
    rect.kind = ref::Complexity::rect;
    rect.bounds = {6, 3, 140, 110};
    const ref::Clip clips[] = {ref::Clip{}, rect, complex};
    const uint32_t rops[] = {0xcccc, 0x6666, 0xc0c0, 0x8888, 0xf0f0};
    unsigned runs = 0, good = 0, flips = 0, kinds = 0, masks = 0, tiles = 0;

    for (uint32_t trial = 0; trial < 48; ++trial) {
        StretchE2E c{};
        const bool dx_neg = (trial & 1) != 0, dy_neg = (trial & 2) != 0;
        const int32_t w = 9 + int32_t(rng() % 70), h = 7 + int32_t(rng() % 60);
        const int32_t x = int32_t(rng() % 60) - 5, y = int32_t(rng() % 40) - 5;
        c.destination = {dx_neg ? x + w : x, dy_neg ? y + h : y, dx_neg ? x : x + w, dy_neg ? y : y + h};
        c.kind = StretchKind(trial % 3);
        const int32_t sw = 3 + int32_t(rng() % 30), sh = 3 + int32_t(rng() % 25);
        const int32_t sx = c.kind == StretchKind::device ? 200 + int32_t(rng() % 80) : int32_t(rng() % 20) - 2;
        const int32_t sy = c.kind == StretchKind::device ? 140 + int32_t(rng() % 20) : int32_t(rng() % 16) - 2;
        c.source = {sx, sy, sx + sw, sy + sh};
        if ((trial & 4) != 0) {
            c.source = {sx + sw, sy, sx, sy + sh};   // a source-side mirror
        }
        c.clip = clips[(trial / 3) % 3];
        c.rop4 = rops[rng() % 5];
        c.tile = (trial % 4) == 3;
        c.mask = !c.tile && (trial % 5) == 1 && display::rop4_uses_source(c.rop4);
        c.origin = {int32_t(rng() % 17) - 8, int32_t(rng() % 17) - 8};
        bool submitted = false;
        const bool same = run_stretch_e2e(fixture, client, c, rng, submitted);
        if (submitted && !same) {
            std::printf("stretch case %u: kind %d mask %d tile %d rop %04x clip %d dst %d,%d,%d,%d src %d,%d,%d,%d\n", trial,
                        int(c.kind), c.mask, c.tile, c.rop4, int(c.clip.kind), c.destination.left, c.destination.top,
                        c.destination.right, c.destination.bottom, c.source.left, c.source.top, c.source.right,
                        c.source.bottom);
        }

        if (submitted) {
            ++runs;
            good += same ? 1u : 0u;
            const ref::DriverStretch drv = ref::int_eng_stretch_blt(c.destination, c.source, c.clip);
            flips |= 1u << ((drv.flip_x ? 1 : 0) | (drv.flip_y ? 2 : 0));
            kinds |= 1u << uint32_t(c.kind);
            masks += c.mask ? 1u : 0u;
            tiles += c.tile ? 1u : 0u;
        }
    }

    std::printf("guarded stretch end to end: %u submitted cases, %u exact, flips mask %x, kinds %x, %u masked, %u tiled\n",
                runs, good, flips, kinds, masks, tiles);
    expect(runs > 30 && good == runs && flips == 0xfu && kinds == 7u && masks > 2 && tiles > 5,
           "0xa17 through the backend equals the EngStretchBltROP fallback (flips, clips, masks, tiles, sources)");

    // Pattern lines: every octant, a banded clip, the brush origin applied.
    std::vector<uint32_t> tile(64);
    for (auto& v : tile) v = rng();
    unsigned line_ok = 0;
    for (uint32_t octant = 0; octant < 8; ++octant) {
        const int32_t major = 40 + int32_t(octant) * 3, minor = 7 + int32_t(octant) * 2;
        int32_t ddx = (octant & 1) ? minor : major, ddy = (octant & 1) ? major : minor;
        if (octant & 2) ddx = -ddx;
        if (octant & 4) ddy = -ddy;
        const int32_t x0 = 160, y0 = 100;
        const ref::Point origin{int32_t(octant) * 3 - 9, 5 - int32_t(octant)};
        const std::vector<ref::Rect> clip = {{100, 40, 230, 95}, {100, 95, 155, 160}, {165, 95, 230, 160}};
        const Image before = snapshot_primary(fixture);
        ref::Bitmap32 expected = to_bitmap(before);
        ref::eng_line_to(clip, x0, y0, x0 + ddx, y0 + ddy, [&](int32_t x, int32_t y) {
            expected.put(x, y, ref::do_rop(0x5a, expected.get(x, y), 0, tile[size_t(((y - origin.y) & 7) * 8 + ((x - origin.x) & 7))]));
        });
        abi::LineCommand line{};
        line.session = client.session;
        line.target = client.primary;
        line.rop3 = 0x5a;
        line.flags = abi::line_flag_pattern;
        line.x0 = x0;
        line.y0 = y0;
        line.x1 = x0 + ddx;
        line.y1 = y0 + ddy;
        line.brush_x = origin.x;
        line.brush_y = origin.y;
        for (uint32_t i = 0; i < 64; ++i) line.pattern[i] = tile[i];
        std::vector<abi::Rect> rects;
        for (const ref::Rect& r : clip) rects.push_back({r.left, r.top, r.right, r.bottom});
        line.rects = rect_list(rects);
        abi::Receipt receipt{};
        Image want(primary_width, primary_height, 0);
        want.pixels = expected.px;
        line_ok += raster_call(client, abi::ioctl_line, abi::line_request_bytes, line, abi::encode_line, receipt) &&
                           client.sync(0, receipt) && primary_matches(fixture, want) ? 1u : 0u;
    }

    expect(line_ok == 8, "pattern PATINVERT lines in all octants equal the EngLineTo pixel set with the tile phase");
}

// Compute raster requests end to end: IOCTL frames, dispatcher, backend, the
// real Raster2dEngine IBs and kernels (interpreted), cross-engine ordering and
// parking/restart of the compute engine.
void compute_raster() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start(), "compute fixture: bring-up");
    ComputeExecutor executor{};
    executor.fixture = &fixture;
    executor.gtt = &fixture.asic->gtt_manager();
    Radeon2dBackendConfig config = fixture.config();
    expect(backend->initialize(config) == Radeon2dStatus::ready && !backend->report().compute &&
               backend->report().compute_status == Raster2dStatus::not_initialized &&
               backend->table().raster_operations == 0, "no compute engine without queues or a submitter");
    expect(backend->shutdown() == Radeon2dStatus::released, "SDMA-only backend released");
    config.compute_submitter = executor.submitter();
    expect(backend->initialize(config) == Radeon2dStatus::ready && backend->report().compute &&
               backend->report().compute_status == Raster2dStatus::success &&
               backend->table().raster_operations == abi::raster_operations,
           "compute engine generated its kernels into GTT and is advertised");

    for (uint32_t y = 0; y < primary_height; ++y) {
        for (uint32_t x = 0; x < primary_width; ++x) {
            write_pixel(fixture.primary().bytes, size_t{y} * primary_pitch + x * 4u, pattern(x, y));
        }
    }

    const radeon2d::EngineSurface scanout{rx7900_test::fb_base, primary_width, primary_height, primary_pitch,
                                          abi::format_xrgb8888};
    expect(dispatcher->configure(backend->table(), scanout, staging_bytes, 0x5150u), "dispatcher with compute");
    Client client{dispatcher.get()};
    expect(client.open(), "compute session");
    abi::Receipt receipt{};

    // 1. PATINVERT with a solid brush over three clip rectangles (reads D).
    {
        const Image before = snapshot_primary(fixture);
        abi::RopCommand command{};
        command.session = client.session;
        command.target = client.primary;
        command.rop4 = 0x5a5a;
        command.brush_colour = 0x80f0a050u;
        command.bounds = {10, 10, 200, 100};
        command.map = {0, 0, 1, 1};
        const std::vector<abi::Rect> clip{{10, 10, 70, 40}, {100, 20, 200, 30}, {12, 60, 300 - 110, 100}};
        command.rects = rect_list(clip);
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, command, abi::encode_rop, receipt) &&
                   client.sync(0, receipt), "PATINVERT accepted and synchronized");
        Image expected = before;

        for (uint32_t y = 0; y < primary_height; ++y) {
            for (uint32_t x = 0; x < primary_width; ++x) {
                if (inside_any(clip, static_cast<int32_t>(x), static_cast<int32_t>(y))) {
                    expected.at(x, y) = cpu_rop(0x5a, 0x80f0a050u, 0, before.pixels.at(size_t{y} * primary_width + x));
                }
            }
        }

        expect(primary_matches(fixture, expected), "compute PATINVERT pixels equal DIB_DoRop");
    }

    // 2. SRCINVERT from a staging source, a 1-bpp mono source and an 8x8 pattern.
    {
        const Image before = snapshot_primary(fixture);
        const uint32_t pitch = 64 * 4;

        for (uint32_t y = 0; y < 16; ++y) {
            for (uint32_t x = 0; x < 64; ++x) {
                client.stage(y * pitch + x * 4u, 0x01020304u * (x + 1) ^ (y << 20));
            }
        }

        abi::RopCommand command{};
        command.session = client.session;
        command.target = client.primary;
        command.rop4 = 0x6666;
        command.bounds = {150, 150, 214, 166};
        command.source = staging_operand(0, pitch, 64, 16, abi::format_xrgb8888);
        command.map = {0, 0, 64, 16};
        command.rects = rect_list({{150, 150, 214, 166}});
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, command, abi::encode_rop, receipt) &&
                   client.sync(0, receipt), "SRCINVERT from staging");
        Image expected = before;

        for (uint32_t y = 0; y < 16; ++y) {
            for (uint32_t x = 0; x < 64; ++x) {
                const uint32_t s = 0x01020304u * (x + 1) ^ (y << 20);
                expected.at(150 + x, 150 + y) = s ^ before.pixels.at(size_t{150 + y} * primary_width + 150 + x);
            }
        }

        expect(primary_matches(fixture, expected), "SRCINVERT pixels");

        // Mono source SRCCOPY: 40 x 8 bits, MSB first, pitch 160 (4 bytes/pixel rule).
        const uint32_t mono_offset = 8192;
        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t byte = 0; byte < 160; byte += 4) {
                client.stage(mono_offset + y * 160 + byte, byte < 8 ? (0xa5c3f00fu ^ (y * 0x11111111u)) : 0);
            }
        }

        abi::RopCommand mono{};
        mono.session = client.session;
        mono.target = client.primary;
        mono.rop4 = 0xcccc;
        mono.bounds = {20, 120, 60, 128};
        mono.source = staging_operand(mono_offset, 160, 40, 8, abi::format_mono1);
        mono.map = {0, 0, 40, 8};
        mono.mono_foreground = 0x00ff0000u;
        mono.mono_background = 0x000000ffu;
        mono.rects = rect_list({{20, 120, 60, 128}});
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, mono, abi::encode_rop, receipt) &&
                   client.sync(0, receipt), "mono source SRCCOPY");
        Image mono_expected = snapshot_primary(fixture);
        bool mono_ok = true;

        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t x = 0; x < 40; ++x) {
                const uint32_t byte = client.staged(mono_offset + y * 160 + (x / 32) * 4) >> (8 * ((x % 32) / 8));
                const bool bit = ((byte >> (7 - (x % 8))) & 1u) != 0;
                mono_ok = mono_ok && mono_expected.at(20 + x, 120 + y) == (bit ? 0x00ff0000u : 0x000000ffu);
            }
        }

        expect(mono_ok, "mono expansion: 1 bits take the foreground, 0 bits the background (MSB first)");

        abi::RopCommand tile{};
        tile.session = client.session;
        tile.target = client.primary;
        tile.rop4 = 0xf0f0;
        tile.has_pattern = true;
        tile.brush_x = 3;
        tile.brush_y = 5;
        tile.bounds = {200, 20, 232, 44};
        tile.map = {0, 0, 1, 1};
        tile.rects = rect_list({{200, 20, 232, 44}});

        for (uint32_t index = 0; index < 64; ++index) {
            tile.pattern[index] = 0x00100000u * index + 7;
        }

        expect(raster_call(client, abi::ioctl_pattern, abi::pattern_request_bytes, tile, abi::encode_rop, receipt) &&
                   client.sync(0, receipt), "pattern PATCOPY");
        const Image after = snapshot_primary(fixture);
        bool tile_ok = true;

        for (uint32_t y = 20; y < 44; ++y) {
            for (uint32_t x = 200; x < 232; ++x) {
                const uint32_t px = (x - 3) & 7u;
                const uint32_t py = (y - 5) & 7u;
                tile_ok = tile_ok && after.at(x, y) == tile.pattern[py * 8 + px];
            }
        }

        expect(tile_ok, "8x8 pattern aligned to the brush origin");
    }

    // 3. Glyphs: a mono mask (ROP4_MASKPAINT) and a coverage8 mask.
    {
        const Image before = snapshot_primary(fixture);
        const uint32_t offset = 16384;
        const uint32_t pitch = 32 * 4;

        for (uint32_t y = 0; y < 12; ++y) {
            for (uint32_t word = 0; word < pitch; word += 4) {
                client.stage(offset + y * pitch + word, word == 0 ? (0x0ff0f00fu ^ (y * 0x01010101u)) : 0);
            }
        }

        abi::TextCommand text{};
        text.session = client.session;
        text.target = client.primary;
        text.mode = abi::text_mode_mono;
        text.colour = 0x00123456u;
        text.bounds = {250, 150, 282, 162};
        text.mask = staging_operand(offset, pitch, 32, 12, abi::format_mono1);
        text.rects = rect_list({{250, 150, 282, 162}});
        expect(raster_call(client, abi::ioctl_text, abi::text_request_bytes, text, abi::encode_text, receipt) &&
                   client.sync(0, receipt), "mono glyph mask");
        const Image after = snapshot_primary(fixture);
        bool glyph_ok = true;

        for (uint32_t y = 0; y < 12; ++y) {
            const uint32_t word = client.staged(offset + y * pitch);

            for (uint32_t x = 0; x < 32; ++x) {
                const uint32_t byte = (word >> (8 * (x / 8))) & 0xffu;
                const bool ink = ((byte >> (7 - (x % 8))) & 1u) != 0;
                const uint32_t old = before.pixels.at(size_t{150 + y} * primary_width + 250 + x);
                glyph_ok = glyph_ok && after.at(250 + x, 150 + y) == (ink ? 0x00123456u : old);
            }
        }

        expect(glyph_ok, "mono glyphs paint the colour where the mask is 1 and keep the destination elsewhere");

        const uint32_t coverage = 24576;
        for (uint32_t y = 0; y < 10; ++y) {
            for (uint32_t x = 0; x < 80; x += 4) {
                uint32_t word = 0;
                for (uint32_t b = 0; b < 4; ++b) {
                    word |= (((x + b) * 37u + y * 11u) & 0xffu) << (8 * b);
                }
                client.stage(coverage + y * 80u + x, x < 20 ? word : 0);
            }
        }

        abi::TextCommand aa{};
        aa.session = client.session;
        aa.target = client.primary;
        aa.mode = abi::text_mode_coverage8;
        aa.colour = 0x00ff8040u;
        aa.bounds = {100, 180, 120, 190};
        aa.mask = staging_operand(coverage, 80, 20, 10, abi::format_coverage8);
        aa.rects = rect_list({{100, 180, 120, 190}});
        const Image under = snapshot_primary(fixture);
        expect(raster_call(client, abi::ioctl_text, abi::text_request_bytes, aa, abi::encode_text, receipt) &&
                   client.sync(0, receipt), "coverage8 glyph mask");
        const Image blended = snapshot_primary(fixture);
        bool aa_ok = true;

        for (uint32_t y = 0; y < 10; ++y) {
            for (uint32_t x = 0; x < 20; ++x) {
                const uint32_t cov = (x * 37u + y * 11u) & 0xffu;
                const uint32_t d = under.at(100 + x, 180 + y);
                const uint32_t c = 0x00ff8040u;
                uint32_t out = d;

                if (cov == 0xff) {
                    out = c;
                } else if (cov > 0) {
                    out = 0;
                    for (uint32_t shift : {16u, 8u, 0u}) {
                        const int cc = static_cast<int>((c >> shift) & 0xffu);
                        const int dd = static_cast<int>((d >> shift) & 0xffu);
                        out |= static_cast<uint32_t>(((static_cast<int>(cov) * (cc - dd) >> 8) + dd) & 0xff) << shift;
                    }
                }

                aa_ok = aa_ok && (blended.at(100 + x, 180 + y) & 0x00ffffffu) == (out & 0x00ffffffu);
            }
        }

        expect(aa_ok, "coverage8 composition equals EngMaskBitBlt");
    }

    // 4. Colour-key stretch (TransparentBlt shape) and AlphaBlend.
    {
        const uint32_t offset = 32768;
        const uint32_t pitch = 16 * 4;

        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t x = 0; x < 16; ++x) {
                client.stage(offset + y * pitch + x * 4u, ((x + y) % 3) == 0 ? 0xff00ff00u : (0x00010101u * (x * 8 + y)));
            }
        }

        const Image before = snapshot_primary(fixture);
        abi::RopCommand key{};
        key.session = client.session;
        key.target = client.primary;
        key.rop4 = 0xcccc;
        key.flags = abi::rop_flag_colour_key | abi::rop_flag_skip_outside_source;
        key.colour_key = 0x0000ff00u;
        key.bounds = {40, 140, 72, 164};
        key.source = staging_operand(offset, pitch, 16, 8, abi::format_xrgb8888);
        key.map = {0, 0, 16, 8};
        key.rects = rect_list({{40, 140, 72, 164}});
        expect(raster_call(client, abi::ioctl_stretch, abi::rop_request_bytes, key, abi::encode_rop, receipt) &&
                   client.sync(0, receipt), "colour-key stretch accepted");
        const Image after = snapshot_primary(fixture);
        bool key_ok = true;

        for (uint32_t y = 0; y < 24; ++y) {
            for (uint32_t x = 0; x < 32; ++x) {
                const uint32_t sx = x * 16 / 32;
                const uint32_t sy = y * 8 / 24;
                const uint32_t s = client.staged(offset + sy * pitch + sx * 4u);
                const uint32_t old = before.at(40 + x, 140 + y);
                key_ok = key_ok && after.at(40 + x, 140 + y) == ((s & 0x00ffffffu) == 0x0000ff00u ? old : s);
            }
        }

        expect(key_ok, "nearest 2x/3x stretch with the colour key masked to 24 bits");

        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t x = 0; x < 16; ++x) {
                client.stage(offset + y * pitch + x * 4u, ((x * 16u + y * 5u) << 24) | (0x00030507u * (x + 1)));
            }
        }

        const Image under = snapshot_primary(fixture);
        abi::AlphaCommand alpha{};
        alpha.session = client.session;
        alpha.target = client.primary;
        alpha.constant_alpha = 200;
        alpha.flags = abi::alpha_flag_source_alpha;
        alpha.bounds = {120, 60, 136, 68};
        alpha.source = staging_operand(offset, pitch, 16, 8, abi::format_xrgb8888);
        alpha.map = {0, 0, 16, 8};
        alpha.rects = rect_list({{120, 60, 136, 68}});
        expect(raster_call(client, abi::ioctl_alpha, abi::alpha_request_bytes, alpha, abi::encode_alpha, receipt) &&
                   client.sync(0, receipt), "AlphaBlend accepted");
        const Image blended = snapshot_primary(fixture);
        bool alpha_ok = true;

        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t x = 0; x < 16; ++x) {
                const uint32_t s = client.staged(offset + y * pitch + x * 4u);
                const uint32_t d = under.at(120 + x, 60 + y);
                const uint32_t ca = 200;
                uint32_t sc[4];
                for (uint32_t c = 0; c < 4; ++c) {
                    sc[c] = ((s >> (8 * c)) & 0xffu) * ca / 255u;
                }

                const uint32_t a = sc[3];
                uint32_t out = 0;
                for (uint32_t c = 0; c < 4; ++c) {
                    const uint32_t value = ((d >> (8 * c)) & 0xffu) * (255u - a) / 255u + sc[c];
                    out |= (value > 255u ? 255u : value) << (8 * c);
                }

                alpha_ok = alpha_ok && blended.at(120 + x, 60 + y) == out;
            }
        }

        expect(alpha_ok, "AlphaBlend AC_SRC_ALPHA with SourceConstantAlpha equals DIB_32BPP_AlphaBlend");
    }

    // 5. A cosmetic line clipped by two rectangles; gradients (rect H and a tall
    //    triangle split over clip rectangles and 64-row chunks).
    {
        const Image before = snapshot_primary(fixture);
        abi::LineCommand line{};
        line.session = client.session;
        line.target = client.primary;
        line.colour = 0x00abcdefu;
        line.rop3 = 0xf0;
        line.x0 = 5;
        line.y0 = 7;
        line.x1 = 290;
        line.y1 = 90;
        const std::vector<abi::Rect> clip{{0, 0, 100, 60}, {150, 0, 320, 200}};
        line.rects = rect_list(clip);
        expect(raster_call(client, abi::ioctl_line, abi::line_request_bytes, line, abi::encode_line, receipt) &&
                   client.sync(0, receipt), "clipped cosmetic line");
        Image expected = before;
        const int32_t major = 285;
        const int32_t minor = 83;

        for (int32_t k = 0; k < major; ++k) {
            const int32_t x = 5 + k;
            const int32_t y = 7 + ((major >> 1) + k * minor) / major;
            if (inside_any(clip, x, y)) {
                expected.at(static_cast<uint32_t>(x), static_cast<uint32_t>(y)) = 0x00abcdefu;
            }
        }

        expect(primary_matches(fixture, expected), "line pixels follow eng/lineto.c Bresenham inside the clip only");

        // The paired IntEngLineTo bridge's one-segment style mask flows through
        // the real Radeon2D line ABI, dispatcher, backend and interpreted GFX11
        // kernel. EngLineTo on radeondisp's DIB view starts run 0 at the first
        // point (Translate 0), so the expected phase is step % (2 + 1).
        abi::LineCommand dash{};
        dash.session = client.session;
        dash.target = client.primary;
        dash.colour = 0x0056a3d2U;
        dash.rop3 = 0xf0;
        dash.flags = abi::line_flag_styled;
        dash.x0 = 10;
        dash.y0 = 130;
        dash.x1 = 90;
        dash.y1 = 142;
        const uint32_t dash_runs[2]{2, 1};
        expect(rx7900::reactos::radeon_display::engine_style_bits(
                   dash.x0, dash.y0, dash.x1, dash.y1, dash_runs, 2,
                   dash.style, dash.style_bit_count),
               "engine-compatible cosmetic style admitted");
        dash.rects = rect_list({{0, 0, 320, 200}});
        const Image dash_before = snapshot_primary(fixture);
        Image dash_expected = dash_before;
        for (int32_t step = 0; step < 80; ++step) {
            if (step % 3 != 2) {
                const int32_t y = 130 + ((80 >> 1) + step * 12) / 80;
                dash_expected.at(static_cast<uint32_t>(10 + step), static_cast<uint32_t>(y)) = dash.colour;
            }
        }
        expect(raster_call(client, abi::ioctl_line, abi::line_request_bytes, dash,
                           abi::encode_line, receipt) && client.sync(0, receipt) &&
                   primary_matches(fixture, dash_expected),
               "source-pinned styled cosmetic segment reaches the GFX11 line kernel exactly");

        // A styled segment under a banded DC_COMPLEX clip: the style state of
        // step k never depends on the clip walk, so every clip rectangle
        // replays the same bit stream (gdi_reference::eng_line_to_styled).
        {
            const std::vector<uint32_t> runs{3, 2, 1, 4};
            const std::vector<ref::Rect> bands{{20, 150, 70, 165}, {90, 150, 160, 165}, {35, 165, 140, 190}};
            abi::LineCommand styled{};
            styled.session = client.session;
            styled.target = client.primary;
            styled.colour = 0x00c0ffeeU;
            styled.rop3 = 0xf0;
            styled.flags = abi::line_flag_styled;
            styled.x0 = 150;
            styled.y0 = 151;
            styled.x1 = 24;
            styled.y1 = 188;
            expect(rx7900::reactos::radeon_display::engine_style_bits(styled.x0, styled.y0, styled.x1, styled.y1,
                       runs.data(), uint32_t(runs.size()), styled.style, styled.style_bit_count),
                   "reverse styled segment admitted");
            std::vector<abi::Rect> wire;
            for (const ref::Rect& r : bands) wire.push_back({r.left, r.top, r.right, r.bottom});
            styled.rects = rect_list(wire);
            const Image styled_before = snapshot_primary(fixture);
            ref::Bitmap32 styled_expected = to_bitmap(styled_before);
            ref::eng_line_to_styled(bands, styled.x0, styled.y0, styled.x1, styled.y1, runs, ref::Point{0, 0},
                                    [&](int32_t x, int32_t y) { styled_expected.put(x, y, styled.colour); });
            Image want(primary_width, primary_height, 0);
            want.pixels = styled_expected.px;
            expect(raster_call(client, abi::ioctl_line, abi::line_request_bytes, styled, abi::encode_line, receipt) &&
                       client.sync(0, receipt) && primary_matches(fixture, want),
                   "styled segment under a complex clip equals the pinned octant loop on the DIB view");
        }

        abi::GradientCommand rect{};
        rect.session = client.session;
        rect.target = client.primary;
        rect.mode = abi::gradient_rect_h;
        rect.vertices[0] = {260, 10, 0x0000, 0x8000, 0xff00};
        rect.vertices[1] = {300, 30, 0xff00, 0x8000, 0x0000};
        rect.rects = rect_list({{260, 10, 300, 30}});
        expect(raster_call(client, abi::ioctl_gradient, abi::gradient_request_bytes, rect, abi::encode_gradient,
                           receipt) && client.sync(0, receipt), "horizontal gradient rect");
        const Image ramp = snapshot_primary(fixture);
        bool ramp_ok = true;

        for (uint32_t x = 0; x < 40; ++x) {
            // HVSTEPCOL closed form: n = floor((k * dc + dy - 1 - (dy >> 1)) / dy).
            const uint32_t dy = 40;
            const uint32_t r = (x * 255u + dy - 1 - dy / 2) / dy;
            const uint32_t b = 255u - r;
            for (uint32_t y = 10; y < 30; ++y) {
                ramp_ok = ramp_ok && (ramp.at(260 + x, y) & 0x00ffffffu) == ((r << 16) | (0x80u << 8) | b);
            }
        }

        expect(ramp_ok, "gradient rect columns step the colour exactly as IntEngGradientFillRect");

        abi::GradientCommand tri{};
        tri.session = client.session;
        tri.target = client.primary;
        tri.mode = abi::gradient_triangle;
        tri.extents = {0, 0, 320, 200};
        tri.vertices[0] = {20, 5, 0xff00, 0x0000, 0x0000};
        tri.vertices[1] = {300, 40, 0x0000, 0xff00, 0x0000};
        tri.vertices[2] = {90, 195, 0x0000, 0x0000, 0xff00};
        const std::vector<abi::Rect> tri_clip{{0, 0, 160, 100}, {160, 0, 320, 100}, {0, 100, 320, 200}};
        tri.rects = rect_list(tri_clip);
        const Image under = snapshot_primary(fixture);
        expect(raster_call(client, abi::ioctl_gradient, abi::gradient_request_bytes, tri, abi::encode_gradient,
                           receipt) && client.sync(0, receipt), "190-row gradient triangle over three clip rectangles");
        static Raster2dGradientSpan plan[radeon2d_triangle_rows][raster2d_triangle_spans];
        Raster2dRect extents{};
        uint32_t rows = 0;
        raster2d_plan_triangle({20, 5, 0xff00, 0, 0}, {300, 40, 0, 0xff00, 0}, {90, 195, 0, 0, 0xff00}, plan,
                               radeon2d_triangle_rows, extents, rows);
        Image tri_expected = under;

        for (uint32_t row = 0; row < rows; ++row) {
            const int32_t y = 5 + static_cast<int32_t>(row);
            for (const auto& span : plan[row]) {
                for (int32_t x = span.low; x <= span.high; ++x) {
                    if (x < 0 || x >= static_cast<int32_t>(primary_width) || !inside_any(tri_clip, x, y)) {
                        continue;
                    }

                    const uint32_t k = static_cast<uint32_t>(span.reverse ? span.high - x : x - span.low);
                    uint32_t colour = 0;
                    for (uint32_t c = 0; c < 3; ++c) {
                        const int64_t n = (int64_t{k} * span.delta[c] + span.bias) / span.divisor;
                        const int64_t value = span.negative[c] ? span.colour[c] - n : span.colour[c] + n;
                        colour |= static_cast<uint32_t>(value & 0xff) << (16 - 8 * c);
                    }

                    tri_expected.at(static_cast<uint32_t>(x), static_cast<uint32_t>(y)) = colour;
                }
            }
        }

        bool tri_ok = rows == 190;
        const Image shaded = snapshot_primary(fixture);
        for (uint32_t y = 0; y < primary_height && tri_ok; ++y) {
            for (uint32_t x = 0; x < primary_width; ++x) {
                if ((shaded.at(x, y) & 0x00ffffffu) != (tri_expected.at(x, y) & 0x00ffffffu)) {
                    std::fprintf(stderr, "triangle mismatch at %u,%u\n", x, y);
                    tri_ok = false;
                    break;
                }
            }
        }

        expect(tri_ok, "triangle rows evaluate the planned spans in order, clipped, across 64-row chunks");
    }

    // 6. Cross-engine ordering: compute writes, then SDMA reads the same pixels
    //    (and the reverse) while the compute queue only runs work it was waited for.
    {
        const auto before = backend->report();
        abi::RopCommand invert{};
        invert.session = client.session;
        invert.target = client.primary;
        invert.rop4 = 0x5555;   // DSTINVERT
        invert.bounds = {0, 0, 32, 32};
        invert.map = {0, 0, 1, 1};
        invert.rects = rect_list({{0, 0, 32, 32}});
        const Image start = snapshot_primary(fixture);
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, invert, abi::encode_rop, receipt),
               "compute DSTINVERT queued");
        // SDMA copy of the inverted block elsewhere: must observe the inversion.
        expect(client.copy(client.primary, client.primary, -100, -100, {{100, 100, 132, 132}}, receipt) &&
                   receipt.outcome == abi::outcome_completed, "SDMA copy of compute output");
        // SDMA fill, then compute reads it (PATINVERT over the fill).
        expect(client.fill(client.primary, 0x00777777u, {{40, 0, 72, 32}}, receipt) &&
                   receipt.outcome == abi::outcome_completed, "SDMA fill");
        invert.bounds = {40, 0, 72, 32};
        invert.rects = rect_list({{40, 0, 72, 32}});
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, invert, abi::encode_rop, receipt) &&
                   client.sync(0, receipt), "compute after SDMA");
        const Image end = snapshot_primary(fixture);
        bool ordered = true;

        for (uint32_t y = 0; y < 32; ++y) {
            for (uint32_t x = 0; x < 32; ++x) {
                ordered = ordered && end.at(100 + x, 100 + y) == ~start.at(x, y) && end.at(x, y) == ~start.at(x, y) &&
                          end.at(40 + x, y) == ~0x00777777u;
            }
        }

        const auto after = backend->report();
        expect(ordered && after.cross_engine_waits >= before.cross_engine_waits + 1,
               "SDMA after compute and compute after SDMA see each other's results (fence waits inserted)");
        // Independent work does not wait.
        const auto quiet = backend->report().cross_engine_waits;
        invert.bounds = {200, 150, 220, 170};
        invert.rects = rect_list({{200, 150, 220, 170}});
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, invert, abi::encode_rop, receipt) &&
                   client.fill(client.primary, 1, {{0, 190, 10, 200}}, receipt) &&
                   backend->report().cross_engine_waits == quiet && client.sync(0, receipt),
               "disjoint SDMA and compute work is not serialized");
        // Two dependent compute operations wait for each other too.
        const auto self = backend->report().compute_self_waits;
        const Image twice = snapshot_primary(fixture);
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, invert, abi::encode_rop, receipt) &&
                   raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, invert, abi::encode_rop, receipt) &&
                   client.sync(0, receipt) && backend->report().compute_self_waits == self + 1 &&
                   primary_matches(fixture, twice), "dependent compute work is ordered by its own fence");
    }

    // 7. Offscreen and evicted surfaces as compute targets; park and restart.
    {
        abi::SurfaceGrant grant{};
        expect(client.create(64, 32, grant), "offscreen compute target");
        abi::RopCommand white{};
        white.session = client.session;
        white.target = grant.surface;
        white.rop4 = 0xffff;
        white.bounds = {0, 0, 64, 32};
        white.map = {0, 0, 1, 1};
        white.rects = rect_list({{0, 0, 64, 32}});
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, white, abi::encode_rop, receipt),
               "WHITENESS on an offscreen surface");
        expect(dispatcher->suspend() && backend->suspend() == Radeon2dStatus::suspended &&
                   !backend->report().compute, "compute engine drained and released at park");
        expect(backend->initialize(config) == Radeon2dStatus::ready && backend->report().compute &&
                   dispatcher->resume(backend->table(), scanout), "compute engine regenerated on restart");
        white.rop4 = 0x0000;
        white.bounds = {0, 0, 32, 32};
        white.rects = rect_list({{0, 0, 32, 32}});
        expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, white, abi::encode_rop, receipt), "BLACKNESS half");
        Image reference(64, 32, 0xffffffffu);
        for (uint32_t y = 0; y < 32; ++y) {
            for (uint32_t x = 0; x < 32; ++x) {
                reference.at(x, y) = 0;
            }
        }

        expect(surface_matches(client, grant.surface, reference),
               "offscreen compute results survive eviction and park, then compose after restart");
    }

    stretch_lines_end_to_end(fixture, client);
    {
        // A second dispatcher over the same backend whose sessions get 1 MiB of
        // staging (the miniport's shipped size) for the rgb16 table.
        auto large_dispatcher = std::make_unique<radeon2d::Dispatcher>();
        const bool large_ready = large_dispatcher->configure(backend->table(), scanout, 1u << 20, 0x5152u);
        Client large{large_dispatcher.get()};
        const bool large_open = large_ready && large.open();
        expect(large_open, "1 MiB staging session for translated rgb16 stretches");
        planned_stretch_end_to_end(fixture, client, large_open ? &large : nullptr);
        if (large_open) {
            translated_bitblt_end_to_end(fixture, large);
            large.close();
        }
    }

    expect(executor.errors == 0 && executor.dispatches > 0, "every compute IB decoded and executed");
    std::printf("compute raster: %u compute IBs, %u dispatches, %u fence waits, %llu cross-engine waits\n",
                executor.sequence, executor.dispatches, executor.waits,
                static_cast<unsigned long long>(backend->report().cross_engine_waits));
    client.close();
    expect(dispatcher->shutdown() && backend->shutdown() == Radeon2dStatus::released,
           "compute backend teardown releases the code, args and IB allocations");
    expect(fixture.sdma.errors == 0, "every SDMA IB decoded cleanly");
    expect(fixture.asic->shutdown() == AsicStatus::completed, "ASIC shutdown after the compute backend");
}

// Pure compute status mappings and the IB offset normalization.
void compute_mappings() {
    expect(radeon2d_compute_submit_result(Gfx11QueuesStatus::running) == Raster2dSubmitResult::submitted &&
               radeon2d_compute_submit_result(Gfx11QueuesStatus::ready) == Raster2dSubmitResult::submitted &&
               radeon2d_compute_submit_result(Gfx11QueuesStatus::ring_full) == Raster2dSubmitResult::busy &&
               radeon2d_compute_submit_result(Gfx11QueuesStatus::busy) == Raster2dSubmitResult::busy &&
               radeon2d_compute_submit_result(Gfx11QueuesStatus::invalid_memory) == Raster2dSubmitResult::failed,
           "Gfx11Queues submit status mapping");
    expect(radeon2d_raster_status(Raster2dStatus::success) == radeon2d::BackendStatus::ok &&
               radeon2d_raster_status(Raster2dStatus::out_of_bounds) == radeon2d::BackendStatus::rejected &&
               radeon2d_raster_status(Raster2dStatus::too_many_rects) == radeon2d::BackendStatus::rejected &&
               radeon2d_raster_status(Raster2dStatus::busy) == radeon2d::BackendStatus::rejected &&
               radeon2d_raster_status(Raster2dStatus::invalid_memory) == radeon2d::BackendStatus::uncertain &&
               radeon2d_raster_status(Raster2dStatus::fence_timeout) == radeon2d::BackendStatus::uncertain,
           "Raster2dStatus mapping");
    uint64_t offset = 0;
    const uint32_t slot = radeon2d_compute_ib_bytes;
    expect(radeon2d_compute_ib_offset(0, slot, 4, offset) && offset == 0 &&
               radeon2d_compute_ib_offset(slot * 2, slot, 4, offset) && offset == slot * 2 &&
               radeon2d_compute_ib_offset(uint64_t{slot} * 3, slot, 4, offset) && offset == slot * 3 &&
               !radeon2d_compute_ib_offset(uint64_t{slot} * 3 * 4, slot, 4, offset) &&
               !radeon2d_compute_ib_offset(slot + 4, slot, 4, offset) &&
               !radeon2d_compute_ib_offset(uint64_t{slot} * 16, slot, 4, offset) &&
               !radeon2d_compute_ib_offset(0, slot, 5, offset),
           "compute IB offsets accept slot byte offsets, nothing else");

    const uint32_t runs[2]{2, 1};
    uint8_t bits[abi::style_bytes]{};
    uint32_t count = 0;
    using rx7900::reactos::radeon_display::engine_style_bits;
    expect(engine_style_bits(0, 1, 8, 1, runs, 2, bits, count) &&
               count == 8 && bits[0] == 0xdb,
           "pinned EngLineTo forward run pattern starts at the first point (DIB view, Translate 0)");
    expect(engine_style_bits(5, 1, 13, 1, runs, 2, bits, count) &&
               count == 8 && bits[0] == 0xdb,
           "pinned EngLineTo style phase does not depend on the device coordinate");
    expect(engine_style_bits(8, 1, 0, 1, runs, 2, bits, count) &&
               count == 8 && bits[0] == 0xb6,
           "pinned EngLineTo reverse run changes and start inclusion");
    const uint32_t invalid_runs[2]{2, 0};
    expect(!engine_style_bits(0, 0, 8, 0, invalid_runs, 2, bits, count) &&
               !engine_style_bits(0, 0, 2049, 0, runs, 2, bits, count),
           "unrepresentable styled paths decline before submission");
    // A diagonal tie seeds lStyleMax on y (y0 + 2 = 2) but steps x from 3: the
    // limit is never met, so every step stays in run 0.
    expect(engine_style_bits(3, 0, 7, 4, runs, 2, bits, count) && count == 4 && bits[0] == 0xf0,
           "diagonal tie keeps the y-seeded limit of HandleStyles while stepping x");
    expect(engine_style_bits(-1, 0, 8, 0, runs, 2, bits, count) && count == 9 && bits[0] == 0xdb &&
               bits[1] == 0x00,
           "negative first points start run 0 there too (no modulo of the origin)");
    // Sweep against the direct octant-loop port with an unbounded clip.
    {
        std::mt19937 rng(0x5717);
        unsigned agree = 0, total = 0;
        for (uint32_t trial = 0; trial < 6000; ++trial) {
            std::vector<uint32_t> style(1 + rng() % 6);
            for (auto& s : style) s = 1 + rng() % 7;
            const int32_t sx = int32_t(rng() % 200) - 60, sy = int32_t(rng() % 200) - 60;
            int32_t ex = sx + int32_t(rng() % 121) - 60, ey = sy + int32_t(rng() % 121) - 60;
            if ((trial % 7) == 0) ey = sy + (ex - sx) * ((trial & 8) != 0 ? -1 : 1);   // ties
            uint8_t sb[abi::style_bytes]{};
            uint32_t sc = 0;
            if (!engine_style_bits(sx, sy, ex, ey, style.data(), uint32_t(style.size()), sb, sc)) {
                continue;
            }
            std::vector<std::pair<int32_t, int32_t>> want, got;
            ref::eng_line_to_styled({{-100000, -100000, 100000, 100000}}, sx, sy, ex, ey, style, ref::Point{0, 0},
                                    [&](int32_t x, int32_t y) { want.push_back({x, y}); });
            // The Bresenham pixel of every step, in step order (the line kernel's model).
            {
                const int32_t ddx = ex - sx, ddy = ey - sy;
                const int32_t adx = ddx < 0 ? -ddx : ddx, ady = ddy < 0 ? -ddy : ddy;
                const int32_t delta = adx > ady ? adx : ady;
                int32_t x = sx, y = sy, error = delta >> 1;
                for (int32_t i = 0; i < delta; ++i) {
                    got.push_back({x, y});
                    if (adx < ady) {
                        y += ddy < 0 ? -1 : 1;
                        error += adx;
                        if (ady <= error) { x += ddx < 0 ? -1 : 1; error -= ady; }
                    } else {
                        x += ddx < 0 ? -1 : 1;
                        error += ady;
                        if (adx <= error) { y += ddy < 0 ? -1 : 1; error -= adx; }
                    }
                }
            }
            std::vector<std::pair<int32_t, int32_t>> kept;
            for (uint32_t step = 0; step < got.size() && step < sc; ++step) {
                if (((sb[step >> 3] >> (7 - (step & 7))) & 1u) != 0) kept.push_back(got[step]);
            }
            ++total;
            agree += kept == want ? 1u : 0u;
        }
        std::printf("styled segments: %u of %u equal the DIB-view octant loops\n", agree, total);
        expect(total > 5000 && agree == total, "engine_style_bits equals EngLineTo HandleStyles on the DIB view");
    }
    // The same sweep under banded DC_COMPLEX regions: the octant loops walk the
    // clip in their own CD_* order, the GPU model keeps step k when its style bit
    // is set and any rectangle contains its pixel (one kernel stream per batch).
    {
        std::mt19937 rng(0xc11b);
        unsigned agree = 0, total = 0, clipped = 0;
        for (uint32_t trial = 0; trial < 6000; ++trial) {
            std::vector<uint32_t> style(1 + rng() % 6);
            for (auto& s : style) s = 1 + rng() % 7;
            // A y-x banded region: bands of random height, 1..3 disjoint spans each.
            std::vector<ref::Rect> region;
            for (int32_t top = -40 + int32_t(rng() % 20); top < 150;) {
                const int32_t bottom = top + 1 + int32_t(rng() % 25);
                int32_t left = -50 + int32_t(rng() % 30);
                const uint32_t spans = 1 + rng() % 3;
                for (uint32_t s = 0; s < spans && left < 150; ++s) {
                    const int32_t right = left + 1 + int32_t(rng() % 60);
                    if ((rng() % 4) != 0) region.push_back({left, top, right, bottom});
                    left = right + 1 + int32_t(rng() % 20);
                }
                top = bottom + int32_t(rng() % 3);
            }
            if (region.empty()) continue;
            const int32_t sx = int32_t(rng() % 160) - 40, sy = int32_t(rng() % 160) - 40;
            int32_t ex = sx + int32_t(rng() % 121) - 60, ey = sy + int32_t(rng() % 121) - 60;
            if ((trial % 7) == 0) ey = sy + (ex - sx) * ((trial & 8) != 0 ? -1 : 1);
            uint8_t sb[abi::style_bytes]{};
            uint32_t sc = 0;
            if (!engine_style_bits(sx, sy, ex, ey, style.data(), uint32_t(style.size()), sb, sc)) continue;
            std::vector<std::pair<int32_t, int32_t>> want, kept;
            ref::eng_line_to_styled(region, sx, sy, ex, ey, style, ref::Point{0, 0},
                                    [&](int32_t x, int32_t y) { want.push_back({x, y}); });
            const int32_t ddx = ex - sx, ddy = ey - sy;
            const int32_t adx = ddx < 0 ? -ddx : ddx, ady = ddy < 0 ? -ddy : ddy;
            const int32_t delta = adx > ady ? adx : ady;
            int32_t x = sx, y = sy, error = delta >> 1;
            uint32_t visible = 0;
            for (int32_t i = 0; i < delta; ++i) {
                bool inside = false;
                for (const ref::Rect& r : region) inside = inside || (r.left <= x && x < r.right && r.top <= y && y < r.bottom);
                visible += inside ? 1u : 0u;
                if (inside && ((sb[i >> 3] >> (7 - (i & 7))) & 1u) != 0) kept.push_back({x, y});
                if (adx < ady) {
                    y += ddy < 0 ? -1 : 1;
                    error += adx;
                    if (ady <= error) { x += ddx < 0 ? -1 : 1; error -= ady; }
                } else {
                    x += ddx < 0 ? -1 : 1;
                    error += ady;
                    if (adx <= error) { y += ddy < 0 ? -1 : 1; error -= adx; }
                }
            }
            std::sort(want.begin(), want.end());
            std::sort(kept.begin(), kept.end());
            ++total;
            clipped += visible != 0 && visible < uint32_t(delta) ? 1u : 0u;
            agree += kept == want ? 1u : 0u;
        }
        std::printf("styled segments under complex clips: %u of %u equal (%u partly clipped)\n", agree, total,
                    clipped);
        expect(total > 5000 && agree == total && clipped > total / 4,
               "styled octant loops under DC_COMPLEX equal the clip-independent style stream per rectangle");
    }
}
} // namespace


// ------------------------------------------------------------ destructive reset recovery
// The full recovery sequence on the shared synthetic Navi31: a hang or fault in
// 2D work, Radeon2dRecovery quiescing the dispatcher, the explicit AsicInit
// mode-1 reset and release (the backend parks inside the reset window),
// restart_after_reset with an ATOM re-POST and the retained GTT rebound, kernel
// submission again, then the backend restart and the dispatcher's recovery.
// The reset erases the test VRAM regions; system memory keeps its bytes.

constexpr uint8_t lost_vram_byte = 0xee;

struct RecoveryHarness final {
    Fixture* fixture = nullptr;
    Radeon2dBackend* backend = nullptr;
    ComputeExecutor* executor = nullptr;
    AtomFirmwareInfo atom{};
    std::vector<uint8_t> rom = rx7900_test::asic_init_rom(0x5c9, rx7900_test::AsicInitRom::posting);
    // Fault knobs: the hang survives the reset (a broken engine), a hang starts
    // right before the backend restarts (a fault during the restore), the
    // adopted scanout no longer matches, the interrupt service refuses.
    bool hang_survives = false;
    bool hang_on_restore = false;
    bool scanout_ok = true;
    bool stop_ok = true;
    unsigned quiesces = 0;
    unsigned resumes = 0;
    unsigned stops = 0;
    unsigned resets = 0;
    unsigned restarts = 0;
    unsigned rearms = 0;

    RecoveryHarness(Fixture& owner, Radeon2dBackend& engine) : fixture(&owner), backend(&engine) {
        atom.bios_scratch_reg = 0x5c9;
    }

    static RecoveryHarness& self(void* context) { return *static_cast<RecoveryHarness*>(context); }
    static bool pci(void*) noexcept { return true; }
    static void quiesce(void* context) noexcept { ++self(context).quiesces; }
    static void resume(void* context) noexcept { ++self(context).resumes; }
    static bool stop(void* context) noexcept {
        ++self(context).stops;
        return self(context).stop_ok;
    }

    static AsicStatus reset_and_release(void* context) noexcept {
        auto& h = self(context);
        ++h.resets;

        if (!h.fixture->asic->reset_proven()) {
            const AsicStatus reset = h.fixture->asic->mode1_reset({&h, pci, pci, pci});

            if (reset != AsicStatus::completed) {
                return reset;
            }
        }

        // The reset stops the engines (their queued work is gone) and erases
        // VRAM; the firmware models start over.
        if (!h.hang_survives) {
            h.fixture->device->sdma_hang_instance = UINT32_MAX;
        }

        if (h.executor != nullptr) {
            h.executor->pending.clear();
            h.executor->executed = h.executor->sequence;
            h.executor->hung = false;
        }

        std::fill(h.fixture->primary().bytes.begin(), h.fixture->primary().bytes.end(), lost_vram_byte);
        std::fill(h.fixture->offscreen().bytes.begin(), h.fixture->offscreen().bytes.end(), lost_vram_byte);
        h.fixture->mes_firmware.reset();
        return h.fixture->asic->release_after_reset(h.backend, Radeon2dRecovery::retire_after_reset);
    }

    static bool proven(void* context) noexcept { return self(context).fixture->asic->reset_proven(); }

    static AsicStatus restart(void* context) noexcept {
        auto& h = self(context);
        ++h.restarts;
        AsicInputs inputs = h.fixture->inputs();
        inputs.atom = &h.atom;
        inputs.atom_rom = {h.rom.data(), h.rom.size()};
        const AsicStatus status = h.fixture->asic->restart_after_reset(
            inputs, h.fixture->policy, {h.fixture->device.get(), SimulatedNavi31::confirm});

        if (status != AsicStatus::completed) {
            return status;
        }

        return h.fixture->asic->enable_kernel_submission(*h.fixture->fences, {&h.fixture->host, HostFences::store});
    }

    static void rearm(void* context) noexcept { ++self(context).rearms; }

    static bool scanout(void* context) noexcept {
        auto& h = self(context);

        if (h.hang_on_restore) {
            h.fixture->device->sdma_hang_instance = 1;
        }

        return h.scanout_ok;
    }

    Radeon2dRecoveryPlatform platform() noexcept {
        return {this, quiesce, resume, stop, reset_and_release, proven, restart, rearm, scanout};
    }
};

const radeon2d::EngineSurface recovery_scanout{rx7900_test::fb_base, primary_width, primary_height, primary_pitch,
                                               abi::format_xrgb8888};

// A 64 x 32 surface holding image (uploaded through staging and synchronized).
bool make_surface(Client& client, const Image& image, abi::SurfaceGrant& grant) {
    abi::Receipt receipt{};

    if (!client.create(image.width, image.height, grant)) {
        return false;
    }

    for (uint32_t y = 0; y < image.height; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
            client.stage(size_t{y} * image.width * 4u + x * 4u, image.at(x, y));
        }
    }

    return client.transfer(true, grant.surface, 0, image.width * 4u, 0, 0,
                           {{0, 0, static_cast<int32_t>(image.width), static_cast<int32_t>(image.height)}}, receipt) &&
           receipt.outcome == abi::outcome_completed && client.sync(0, receipt) &&
           receipt.outcome == abi::outcome_completed;
}

Image gradient_image(uint32_t seed) {
    Image image(64, 32, 0);

    for (uint32_t y = 0; y < image.height; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
            image.at(x, y) = pattern(x + seed, y * 3u + seed) ^ (seed << 24);
        }
    }

    return image;
}

// Moves every offscreen surface of the dispatcher to GTT and keeps it there:
// suspend, park, restart while VRAM is exhausted (the restore is refused), free.
bool keep_in_gtt(Fixture& fixture, Radeon2dBackend& backend, radeon2d::Dispatcher& dispatcher) {
    if (!dispatcher.suspend() || backend.suspend() != Radeon2dStatus::suspended) {
        return false;
    }

    std::vector<VramAllocationHandle> blockers;

    for (;;) {
        VramAllocationHandle handle{};

        if (fixture.vram->allocate({256u << 10, rx7900::memory::page_bytes, vram_kernel_bo_flags(false)}, handle) !=
            VramStatus::success) {
            break;
        }

        blockers.push_back(handle);
    }

    const bool resumed = backend.initialize(fixture.config()) == Radeon2dStatus::ready &&
                         dispatcher.resume(backend.table(), recovery_scanout);

    for (auto& handle : blockers) {
        fixture.vram->release(handle);
    }

    return resumed && backend.report().system_surfaces != 0;
}

abi::Capabilities capabilities_of(radeon2d::Dispatcher& dispatcher) {
    abi::Capabilities value{};
    const auto result = dispatcher.dispatch(abi::ioctl_query, rx7900::ByteView(nullptr, 0), abi::capabilities_bytes);
    if (result.status != radeon2d::DispatchStatus::success ||
        !abi::decode_capabilities(dispatcher.reply(), value)) {
        value.recovery_state = 0xffffffffu;
    }

    return value;
}

// Timeout during an SDMA fill: GTT and DIB contents survive exactly, VRAM is
// lost and reported once, the session and its staging survive.
void recovery_after_fill_timeout() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
               dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x1234u),
           "recovery fixture: bring-up, backend and dispatcher");
    Client client{dispatcher.get()};
    expect(client.open(), "recovery session");
    const Image kept_image = gradient_image(1);
    const Image dirty_image = gradient_image(2);
    const Image vram_image = gradient_image(3);
    const Image dib = gradient_image(4);
    abi::SurfaceGrant kept{};
    abi::SurfaceGrant dirty{};
    abi::SurfaceGrant vram_surface{};
    expect(make_surface(client, kept_image, kept) && make_surface(client, dirty_image, dirty),
           "two offscreen surfaces with known pixels");
    expect(keep_in_gtt(fixture, *backend, *dispatcher) && backend->report().system_surfaces == 2,
           "both live in GTT (system memory) after a suspend whose restore found no VRAM");
    expect(make_surface(client, vram_image, vram_surface) && backend->report().system_surfaces == 2,
           "a third surface lives in VRAM");
    // A staging marker the DLL might hold (upper part, untouched by transfers).
    const size_t marker = staging_bytes - 64;
    for (uint32_t index = 0; index < 16; ++index) {
        client.stage(marker + index * 4u, 0xa5000000u + index);
    }
    uint8_t* const staging_view = client.staging;

    // The SDMA engine hangs: a fill on the primary and one on the dirty GTT
    // surface are accepted but never execute; sync times out.
    fixture.device->sdma_hang_instance = 1;
    abi::Receipt receipt{};
    expect(client.fill(client.primary, 0x00abcdefu, {{0, 0, 32, 32}}, receipt) &&
               receipt.outcome == abi::outcome_completed &&
               client.fill(dirty.surface, 0x00777777u, {{0, 0, 8, 8}}, receipt) &&
               receipt.outcome == abi::outcome_completed,
           "work accepted onto the hung SDMA ring");
    expect(client.sync(0, receipt) && receipt.outcome == abi::outcome_lost && backend->faulted() &&
               Radeon2dRecovery::needed(*backend, *fixture.fences),
           "the timeout poisons the session and a recovery is due");

    RecoveryHarness harness(fixture, *backend);
    Radeon2dRecovery recovery{};
    const unsigned resets_before = fixture.device->mode1_resets;
    const auto result = recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform());
    const auto report = recovery.report();
    expect(result == Radeon2dRecoveryResult::recovered && report.recovered == 1 && !report.disabled &&
               harness.quiesces == 1 && harness.stops == 1 && harness.resets == 1 && harness.restarts == 1 &&
               harness.rearms == 1 && harness.resumes == 1 && fixture.device->mode1_resets == resets_before + 1,
           "orderly recovery: quiesce, interrupts, one mode-1 reset, restart, rearm, display resume");
    expect(fixture.asic->restarts() == 1 && fixture.asic->repost_report().posted &&
               fixture.asic->report().gart_rebound_allocations >= 3,
           "the restart re-POSTed and rebound the retained staging and GTT surfaces");
    const auto parked = backend->report();
    expect(backend->resets() == 1 && parked.reset_dropped_surfaces == 1 && parked.status == Radeon2dStatus::ready,
           "the backend parked inside the reset window (one VRAM surface dropped) and restarted");
    expect(!Radeon2dRecovery::needed(*backend, *fixture.fences), "nothing is hung after the recovery");

    const abi::Capabilities caps = capabilities_of(*dispatcher);
    const auto counters = dispatcher->counters();
    expect(caps.recovery_generation == 1 && caps.recovery_state == abi::recovery_operational &&
               caps.reset_lost_surfaces == 2 && caps.reset_restored_surfaces == 1 &&
               counters.revived_sessions == 1 && (caps.flags & abi::capability_fill) != 0,
           "query reports generation 1, one surface restored and two lost (VRAM and the uncertain write)");
    expect(client.staging == staging_view && client.sync(0, receipt) && receipt.outcome == abi::outcome_completed,
           "the session survives with its staging mapping and syncs again");
    bool marker_ok = true;
    for (uint32_t index = 0; index < 16; ++index) {
        marker_ok = marker_ok && client.staged(marker + index * 4u) == 0xa5000000u + index;
    }
    expect(marker_ok, "staging bytes survive the reset unchanged (system memory)");
    expect(surface_matches(client, kept.surface, kept_image), "the untouched GTT surface is restored exactly");
    expect(backend->report().system_surfaces == 0, "the restored surface moved back into VRAM");

    // Lost handles: answered exactly once, then dead; never aliased.
    expect(client.fill(vram_surface.surface, 1, {{0, 0, 4, 4}}, receipt) &&
               receipt.outcome == abi::outcome_surface_lost && receipt.lost_surface == vram_surface.surface,
           "the VRAM surface answers surface_lost naming its handle");
    expect(!client.fill(vram_surface.surface, 1, {{0, 0, 4, 4}}, receipt) &&
               !client.transfer(false, vram_surface.surface, 0, 256, 0, 0, {{0, 0, 4, 4}}, receipt),
           "the reported handle is dead for every later request");
    expect(client.transfer(false, dirty.surface, 0, 256, 0, 0, {{0, 0, 4, 4}}, receipt) &&
               receipt.outcome == abi::outcome_surface_lost && receipt.lost_surface == dirty.surface &&
               !client.destroy(dirty.surface, receipt),
           "the GTT surface with an uncertain write is lost (once) too");
    abi::SurfaceGrant fresh{};
    expect(make_surface(client, dib, fresh) && fresh.surface != vram_surface.surface &&
               fresh.surface != dirty.surface && surface_matches(client, fresh.surface, dib),
           "a DIB promoted after the recovery uploads exactly into a new handle");
    expect(!client.fill(vram_surface.surface, 1, {{0, 0, 1, 1}}, receipt),
           "the old handle still never aliases the new allocation");
    Image screen(primary_width, primary_height, 0);
    for (uint32_t y = 0; y < primary_height; ++y) {
        for (uint32_t x = 0; x < primary_width; ++x) {
            screen.at(x, y) = read_pixel(fixture.primary().bytes, size_t{y} * primary_pitch + x * 4u);
        }
    }
    expect(screen.at(0, 0) == 0xeeeeeeeeu, "the primary's VRAM contents were erased by the reset");
    expect(client.fill(client.primary, 0x00102030u, {{0, 0, 8, 8}}, receipt) &&
               receipt.outcome == abi::outcome_completed && client.sync(0, receipt),
           "the primary keeps its handle and owner and draws again");
    for (uint32_t y = 0; y < 8; ++y) {
        for (uint32_t x = 0; x < 8; ++x) {
            screen.at(x, y) = 0x00102030u;
        }
    }
    expect(primary_matches(fixture, screen), "primary pixels after the recovery");
    expect(fixture.sdma.errors == 0, "every IB after the restart decoded cleanly");
    client.close();
    expect(dispatcher->shutdown() && backend->shutdown() == Radeon2dStatus::released &&
               fixture.asic->shutdown() == AsicStatus::completed,
           "orderly teardown of the recovered incarnation");
}

// A fault while evicting for a power transition: the move is uncertain, the
// earlier GTT surface survives, the surface being moved is reported lost.
void recovery_after_eviction_fault() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
               dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x4321u),
           "eviction fixture ready");
    Client client{dispatcher.get()};
    const Image kept_image = gradient_image(5);
    const Image moving_image = gradient_image(6);
    abi::SurfaceGrant kept{};
    abi::SurfaceGrant moving{};
    expect(client.open() && make_surface(client, kept_image, kept) && keep_in_gtt(fixture, *backend, *dispatcher),
           "one surface already in GTT");
    expect(make_surface(client, moving_image, moving), "a VRAM surface to evict");
    fixture.device->sdma_hang_instance = 1;
    expect(!dispatcher->suspend() && dispatcher->suspended() && backend->faulted() &&
               backend->report().quarantined_surfaces == 1,
           "the eviction copy hangs: suspend unprovable, the moving surface quarantined");
    RecoveryHarness harness(fixture, *backend);
    Radeon2dRecovery recovery{};
    expect(recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform()) ==
               Radeon2dRecoveryResult::recovered && !dispatcher->suspended(),
           "recovery from the half-suspended dispatcher");
    abi::Receipt receipt{};
    expect(surface_matches(client, kept.surface, kept_image), "the surface evicted earlier is exact");
    expect(client.fill(moving.surface, 0, {{0, 0, 1, 1}}, receipt) && receipt.outcome == abi::outcome_surface_lost &&
               receipt.lost_surface == moving.surface && !client.destroy(moving.surface, receipt),
           "the surface whose move was uncertain is reported lost once");
    const auto counters = dispatcher->counters();
    expect(counters.reset_lost == 1 && counters.reset_restored == 1 && counters.revived_sessions == 1 &&
               backend->report().quarantined_surfaces == 0 && backend->report().live_surfaces == 1,
           "accounting: one lost, one restored, the quarantine released by the reset proof");
}

// Reset during suspend: the backend is parked and AsicInit suspended (its own
// closing mode-1 reset), then the resume is abandoned for a destructive recovery.
void recovery_during_suspend() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
               dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x5555u),
           "suspend fixture ready");
    Client client{dispatcher.get()};
    const Image first_image = gradient_image(7);
    const Image second_image = gradient_image(8);
    abi::SurfaceGrant first{};
    abi::SurfaceGrant second{};
    expect(client.open() && make_surface(client, first_image, first) && make_surface(client, second_image, second),
           "two VRAM surfaces before the suspend");
    expect(dispatcher->suspend() && backend->suspend() == Radeon2dStatus::suspended,
           "2D evicted to GTT and parked");
    RecoveryHarness harness(fixture, *backend);
    const SuspendPlatform suspend{{&harness, RecoveryHarness::pci, RecoveryHarness::pci, RecoveryHarness::pci},
                                  nullptr, nullptr};
    const unsigned resets_before = fixture.device->mode1_resets;
    expect(fixture.asic->suspend(suspend) == AsicStatus::completed && fixture.asic->reset_proven() &&
               fixture.device->mode1_resets == resets_before + 1,
           "AsicInit suspend completed with its mode-1 reset");
    Radeon2dRecovery recovery{};
    expect(recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform()) ==
               Radeon2dRecoveryResult::recovered &&
               fixture.device->mode1_resets == resets_before + 1,
           "recovery reuses the suspend's reset proof (no second reset)");
    expect(surface_matches(client, first.surface, first_image) && surface_matches(client, second.surface, second_image),
           "both evicted surfaces are restored exactly");
    expect(dispatcher->counters().reset_restored == 2 && dispatcher->counters().reset_lost == 0 &&
               !dispatcher->suspended() && backend->report().system_surfaces == 0,
           "no loss: nothing was VRAM-resident at the reset");
}

// A compute queue timeout: the backend faults through the compute engine and
// the restarted backend regenerates its kernels.
void recovery_after_compute_timeout() {
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start(), "compute recovery fixture");
    ComputeExecutor executor{};
    executor.fixture = &fixture;
    executor.gtt = &fixture.asic->gtt_manager();
    Radeon2dBackendConfig config = fixture.config();
    config.compute_submitter = executor.submitter();
    expect(backend->initialize(config) == Radeon2dStatus::ready && backend->report().compute &&
               dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x7777u),
           "compute backend and dispatcher");
    Client client{dispatcher.get()};
    expect(client.open(), "compute session");
    executor.hung = true;
    abi::RopCommand command{};
    command.session = client.session;
    command.target = client.primary;
    command.rop4 = 0x5a5a;
    command.brush_colour = 0x80f0a050u;
    command.bounds = {0, 0, 16, 16};
    command.map = {0, 0, 1, 1};
    command.rects = rect_list({{0, 0, 16, 16}});
    abi::Receipt receipt{};
    expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, command, abi::encode_rop, receipt) &&
               receipt.outcome == abi::outcome_completed,
           "compute PATINVERT accepted on the hung queue");
    expect(client.sync(0, receipt) && receipt.outcome == abi::outcome_lost && backend->faulted() &&
               Radeon2dRecovery::needed(*backend, *fixture.fences),
           "the compute wait times out and faults the backend");
    RecoveryHarness harness(fixture, *backend);
    harness.executor = &executor;
    Radeon2dRecovery recovery{};
    expect(recovery.recover(*dispatcher, *backend, config, recovery_scanout, harness.platform()) ==
               Radeon2dRecoveryResult::recovered && backend->report().compute &&
               backend->table().raster_operations == abi::raster_operations,
           "recovered with the compute engine regenerated");
    expect(raster_call(client, abi::ioctl_rop, abi::rop_request_bytes, command, abi::encode_rop, receipt) &&
               receipt.outcome == abi::outcome_completed && client.sync(0, receipt) &&
               receipt.outcome == abi::outcome_completed,
           "compute work runs again on the revived session");
    Image expected(primary_width, primary_height, 0);
    for (uint32_t y = 0; y < primary_height; ++y) {
        for (uint32_t x = 0; x < primary_width; ++x) {
            const uint32_t before = 0xeeeeeeeeu;
            expected.at(x, y) = x < 16 && y < 16 ? cpu_rop(0x5a, 0x80f0a050u, 0, before) : before;
        }
    }
    expect(primary_matches(fixture, expected), "compute pixels after the recovery equal DIB_DoRop");
}

// Second faults and refusals: every failure disables 2D for the device
// lifetime without freeing staging the DLL may still map.
void recovery_failures() {
    // The hang survives the reset: the restart's SDMA ring test fails.
    {
        Fixture fixture;
        auto backend = std::make_unique<Radeon2dBackend>();
        auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
        expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
                   dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x1111u),
               "restart-failure fixture ready");
        Client client{dispatcher.get()};
        expect(client.open(), "session before the fault");
        client.stage(0, 0x13579bdfu);
        fixture.device->sdma_hang_instance = 1;
        abi::Receipt receipt{};
        expect(client.fill(client.primary, 1, {{0, 0, 4, 4}}, receipt) && client.sync(0, receipt) &&
                   receipt.outcome == abi::outcome_lost, "hung SDMA");
        RecoveryHarness harness(fixture, *backend);
        harness.hang_survives = true;
        Radeon2dRecovery recovery{};
        const auto result = recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout,
                                             harness.platform());
        const auto report = recovery.report();
        expect(result == Radeon2dRecoveryResult::restart_failed && report.disabled && report.quiescent &&
                   report.restart_status != AsicStatus::completed && dispatcher->disabled(),
               "a second fault during the restart disables 2D (quiescent: the reset was proven)");
        const abi::Capabilities caps = capabilities_of(*dispatcher);
        expect(caps.recovery_state == abi::recovery_disabled_quiescent && caps.flags == 0 &&
                   dispatcher->dispatch(abi::ioctl_fill, rx7900::ByteView(nullptr, 0), abi::receipt_bytes).status ==
                       radeon2d::DispatchStatus::not_ready,
               "query reports the sticky state; every other request answers not_ready");
        expect(backend->report().live_staging == 1 && client.staged(0) == 0x13579bdfu && !dispatcher->shutdown() &&
                   backend->report().live_staging == 1,
               "staging stays allocated and readable; shutdown never frees it");
        const unsigned resets = harness.resets;
        expect(recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform()) ==
                   Radeon2dRecoveryResult::disabled && harness.resets == resets && recovery.report().failed == 1,
               "sticky: a later recovery does nothing");
        expect(!dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 1),
               "a disabled dispatcher cannot be configured again");
    }

    // A fault during the restore after a successful restart.
    {
        Fixture fixture;
        auto backend = std::make_unique<Radeon2dBackend>();
        auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
        expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
                   dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x2222u),
               "restore-failure fixture ready");
        Client client{dispatcher.get()};
        abi::SurfaceGrant kept{};
        expect(client.open() && make_surface(client, gradient_image(9), kept) &&
                   keep_in_gtt(fixture, *backend, *dispatcher), "a GTT surface to restore");
        fixture.device->sdma_hang_instance = 1;
        abi::Receipt receipt{};
        expect(client.fill(client.primary, 1, {{0, 0, 4, 4}}, receipt) && client.sync(0, receipt), "first hang");
        RecoveryHarness harness(fixture, *backend);
        harness.hang_on_restore = true;
        Radeon2dRecovery recovery{};
        expect(recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform()) ==
                   Radeon2dRecoveryResult::second_fault && !recovery.report().quiescent &&
                   capabilities_of(*dispatcher).recovery_state == abi::recovery_disabled_unproven,
               "an uncertain restore is a second fault: 2D off, GPU retirement unproven");
    }

    // The adopted scanout no longer matches after the reset.
    {
        Fixture fixture;
        auto backend = std::make_unique<Radeon2dBackend>();
        auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
        expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
                   dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x3333u),
               "mismatch fixture ready");
        Client client{dispatcher.get()};
        expect(client.open(), "mismatch session");
        fixture.device->sdma_hang_instance = 1;
        abi::Receipt receipt{};
        expect(client.fill(client.primary, 1, {{0, 0, 4, 4}}, receipt) && client.sync(0, receipt), "hang");
        RecoveryHarness harness(fixture, *backend);
        harness.scanout_ok = false;
        Radeon2dRecovery recovery{};
        expect(recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform()) ==
                   Radeon2dRecoveryResult::scanout_mismatch && recovery.report().scanout_mismatches == 1 &&
                   recovery.report().quiescent && harness.resumes == 0 &&
                   fixture.asic->report().status == AsicStatus::completed,
               "a changed DCN scanout keeps 2D off (counted); the ASIC itself runs again");
    }

    // The interrupt service cannot be quiesced: nothing is reset.
    {
        Fixture fixture;
        auto backend = std::make_unique<Radeon2dBackend>();
        auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
        expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
                   dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x4444u),
               "quiesce fixture ready");
        RecoveryHarness harness(fixture, *backend);
        harness.stop_ok = false;
        Radeon2dRecovery recovery{};
        const unsigned resets = fixture.device->mode1_resets;
        expect(recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform()) ==
                   Radeon2dRecoveryResult::quiesce_failed && harness.resets == 0 &&
                   fixture.device->mode1_resets == resets && !recovery.report().quiescent,
               "no reset without a quiet interrupt service (unproven, disabled)");
    }

    // Bounded: radeon2d_max_recoveries recoveries, then the next fault disables.
    {
        Fixture fixture;
        // The simulator's GTT is a bump allocator: give it room for four bring-ups.
        fixture.device->system_ram.assign(32u << 20, 0x5a);
        auto backend = std::make_unique<Radeon2dBackend>();
        auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
        expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
                   dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x6666u),
               "bound fixture ready");
        Client client{dispatcher.get()};
        expect(client.open(), "bound session");
        RecoveryHarness harness(fixture, *backend);
        Radeon2dRecovery recovery{};
        unsigned recovered = 0;

        for (uint32_t round = 0; round < radeon2d_max_recoveries + 1; ++round) {
            fixture.device->sdma_hang_instance = 1;
            abi::Receipt receipt{};
            client.fill(client.primary, round, {{0, 0, 4, 4}}, receipt);
            client.sync(0, receipt);
            const auto result = recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout,
                                                 harness.platform());
            recovered += result == Radeon2dRecoveryResult::recovered ? 1u : 0u;

            if (round == radeon2d_max_recoveries) {
                expect(result == Radeon2dRecoveryResult::limit_reached && harness.resets == radeon2d_max_recoveries,
                       "the fault after the last allowed recovery disables 2D without another reset");
            }
        }

        expect(recovered == radeon2d_max_recoveries && fixture.asic->restarts() == radeon2d_max_recoveries &&
                   capabilities_of(*dispatcher).recovery_state == abi::recovery_disabled_unproven,
               "exactly radeon2d_max_recoveries recoveries per device lifetime");
    }
}

// The DLL side of version 6: lost-handle memory and the revival decision.
void recovery_dll_policy() {
    namespace display = rx7900::reactos::radeon_display;
    display::LostSurfaces lost{};
    expect(!display::is_lost(lost, 0) && !display::is_lost(lost, 0x105), "nothing remembered initially");
    display::remember_lost(lost, 0x105);
    display::remember_lost(lost, 0x105);
    display::remember_lost(lost, 0);
    expect(display::is_lost(lost, 0x105) && !display::is_lost(lost, 0x205) && lost.next == 1,
           "a reported handle is remembered once; zero is ignored");
    for (uint32_t index = 0; index < display::lost_surface_slots; ++index) {
        display::remember_lost(lost, 0x1000u + index);
    }
    expect(!display::is_lost(lost, 0x105) && display::is_lost(lost, 0x1000u + display::lost_surface_slots - 1),
           "bounded FIFO over every surface slot");
    abi::Capabilities caps{};
    caps.recovery_generation = 2;
    expect(display::revival_decision(caps, 2, 1) == display::Revival::wait &&
               display::revival_decision(caps, 1, 1) == display::Revival::revive &&
               display::revival_decision(caps, 2, display::max_revival_queries) == display::Revival::give_up,
           "revive only after a new generation; bounded waiting");
    caps.recovery_state = abi::recovery_disabled_quiescent;
    expect(display::revival_decision(caps, 1, 1) == display::Revival::quiescent, "proven-quiescent disabled 2D");
    caps.recovery_state = abi::recovery_disabled_unproven;
    expect(display::revival_decision(caps, 1, 1) == display::Revival::give_up, "unproven disabled 2D gives up");
}


// The hand-off between detecting paths and the recovery worker thread.
void recovery_gate_tests() {
    Radeon2dRecoveryGate gate{};
    expect(!gate.pending() && !gate.begin(false, true), "an idle gate runs nothing");
    expect(gate.request(false) && gate.pending() && !gate.request(false) && gate.report().coalesced == 1,
           "the first request wakes the worker, a second is absorbed");
    expect(!gate.begin(true, true) && gate.pending() && gate.report().deferred == 1,
           "never during low power: the request stays pending");
    expect(!gate.begin(false, false) && !gate.pending() && gate.report().dropped == 1,
           "a hang request no longer due (nothing hung) is dropped");
    expect(gate.request(true) && gate.begin(false, false) && gate.report().state == Radeon2dRecoveryState::running,
           "a forced request (failed power transition) runs without a hang");
    expect(!gate.request(false) && gate.pending() && !gate.begin(false, true),
           "a request while running is absorbed; no second concurrent run");
    expect(!gate.stop() && gate.stopping(), "teardown waits for a running recovery");
    gate.finish(false);
    expect(!gate.pending() && gate.stop() && !gate.request(true) && gate.report().refused == 1 &&
               !gate.begin(false, true), "after stop nothing is accepted or run");
    Radeon2dRecoveryGate sticky{};
    expect(sticky.request(false) && sticky.begin(false, true), "run");
    sticky.finish(true);
    expect(!sticky.request(true) && sticky.report().disabled && sticky.report().runs == 1,
           "a disabling recovery makes later requests refused");
}

// Detection on the IOCTL path only quiesces and requests; the worker runs the
// recovery later (here: on the test thread) on a dispatcher already quiesced.
void recovery_request_flow() {
    namespace display = rx7900::reactos::radeon_display;
    Fixture fixture;
    auto backend = std::make_unique<Radeon2dBackend>();
    auto dispatcher = std::make_unique<radeon2d::Dispatcher>();
    expect(fixture.start() && backend->initialize(fixture.config()) == Radeon2dStatus::ready &&
               dispatcher->configure(backend->table(), recovery_scanout, staging_bytes, 0x8888u),
           "request-flow fixture ready");
    Client client{dispatcher.get()};
    const Image kept_image = gradient_image(11);
    abi::SurfaceGrant kept{};
    expect(client.open() && make_surface(client, kept_image, kept) && keep_in_gtt(fixture, *backend, *dispatcher),
           "a GTT surface");
    fixture.device->sdma_hang_instance = 1;
    abi::Receipt receipt{};
    expect(client.fill(client.primary, 3, {{0, 0, 4, 4}}, receipt) && client.sync(0, receipt) &&
               receipt.outcome == abi::outcome_lost, "the IOCTL sees the fault");
    Radeon2dRecoveryGate gate{};
    // request_recovery as the IOCTL path does it.
    expect(Radeon2dRecovery::needed(*backend, *fixture.fences) && gate.request(false) &&
               dispatcher->begin_recovery(), "requested and quiesced at detection");
    const unsigned fills = fixture.sdma.fills;
    const abi::Capabilities pending = capabilities_of(*dispatcher);
    expect(!client.fill(client.primary, 4, {{0, 0, 4, 4}}, receipt) && pending.recovery_state == abi::recovery_pending &&
               pending.flags == 0 && fixture.sdma.fills == fills,
           "until the worker ran, requests answer not_ready and query reports pending");
    expect(display::revival_decision(pending, 0, display::max_revival_queries) == display::Revival::wait,
           "the DLL waits for a pending recovery without spending its query budget");
    // The worker thread.
    RecoveryHarness harness(fixture, *backend);
    Radeon2dRecovery recovery{};
    expect(gate.begin(false, Radeon2dRecovery::needed(*backend, *fixture.fences)), "the worker begins");
    const auto result = recovery.recover(*dispatcher, *backend, fixture.config(), recovery_scanout, harness.platform());
    gate.finish(recovery.disabled());
    expect(result == Radeon2dRecoveryResult::recovered && !gate.pending() && !dispatcher->recovering() &&
               capabilities_of(*dispatcher).recovery_generation == 1 &&
               capabilities_of(*dispatcher).recovery_state == abi::recovery_operational,
           "the worker's recovery completes on the pre-quiesced dispatcher");
    expect(display::revival_decision(capabilities_of(*dispatcher), 0, 0) == display::Revival::revive &&
               surface_matches(client, kept.surface, kept_image),
           "the DLL revives on the new generation; the GTT surface is exact");
}

int main() {
    mappings();
    end_to_end();
    hung_engine();
    batching_and_eviction();
    compute_mappings();
    compute_raster();
    recovery_after_fill_timeout();
    recovery_after_eviction_fault();
    recovery_during_suspend();
    recovery_after_compute_timeout();
    recovery_failures();
    recovery_dll_policy();
    recovery_gate_tests();
    recovery_request_flow();
    return checks.finish("radeon_2d_backend");
}
