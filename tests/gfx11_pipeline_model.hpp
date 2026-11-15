/*
Module: rx7900.tests.gfx11_pipeline_model
Purpose: Execute kernel-built GFX11 graphics IBs on the host: decode PM4, keep the
register state, run each draw's NGG and pixel programs through gl-shaders'
interpreter (gfx11_interp::graphics::run_draw) and apply the fixed-function
back end (scissor, depth, stencil, blend, logic op, colour mask, CB format and
swap, DB->CB copy) and CP DMA_DATA from the register values. 2026-10-24: MRT1
(GL_FRONT_AND_BACK) through CB_COLOR1_*, MRTZ depth and stencil-test-value
exports (DB_SHADER_CONTROL), the v4 PS inputs, and shader memory reached
through the port (PS constants, texture levels, pixel images).
Created: 2026-10-23
Architecture: Test-only. Every field is read by its AMD header name through
gfx11_header_db, never through src/opengl/gfx11_registers.hpp, so the model is an
independent reading of the emitted registers. Memory is reached through a
caller-supplied port over the session's real page tables. The DB keeps depth
and stencil in model arrays keyed by the programmed base addresses (the swizzled
64KB_Z_X layout is not modelled). Semantics that only hardware can confirm (FACE
orientation under the inverted viewport, DEPTH_COPY data placement, the GFX11
attribute-ring wave layout) are modelled as the code programs them.
*/
#pragma once

#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "gfx11_header_db.hpp"
#include "gfx11_interp.hpp"

namespace gfx11_pipeline_model {

struct MemoryPort final {
    std::function<bool(uint64_t, uint8_t*, size_t)> read;
    std::function<bool(uint64_t, const uint8_t*, size_t)> write;
};

struct Stats final {
    unsigned packets = 0;
    unsigned draws = 0;
    unsigned primitives = 0;
    unsigned fragments = 0;
    unsigned dma_rows = 0;
    unsigned errors = 0;
    std::string first_error;
};

class Machine final {
public:
    Machine(const gfx11_header_db::Database& db, MemoryPort port) : db_(db), port_(std::move(port)) {}

    Stats& stats() { return stats_; }

    void execute(const std::vector<uint32_t>& ib) {
        for (size_t i = 0; i < ib.size();) {
            const uint32_t header = ib[i];
            if (header == 0xffff1000u) { ++i; continue; }
            if ((header >> 30) != 3) { fail("non type-3 packet"); return; }
            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            const uint32_t opcode = (header >> 8) & 0xff;
            if (i + 1 + count > ib.size()) { fail("truncated packet"); return; }
            ++stats_.packets;
            const uint32_t* body = &ib[i + 1];
            if (opcode == 0x69 || opcode == 0x76 || opcode == 0x79) {
                const uint32_t base = opcode == 0x69 ? 0xa000 : opcode == 0x76 ? 0x2c00 : 0xc000;
                for (uint32_t k = 1; k < count; ++k) regs_[base + body[0] + k - 1] = body[k];
            } else if (opcode == 0x12) {
                // CLEAR_STATE: context registers return to defaults (zero here).
                for (auto it = regs_.begin(); it != regs_.end();) {
                    it = it->first >= 0xa000 && it->first < 0xc000 ? regs_.erase(it) : std::next(it);
                }
            } else if (opcode == 0x2d) {
                draw(body[0]);
            } else if (opcode == 0x50) {
                dma(body);
            }
            i += count + 1;
        }
    }

    // Depth/stencil planes the model keeps per DB base address (tests inspect).
    std::map<uint64_t, std::vector<float>> depth_planes;
    std::map<uint64_t, std::vector<uint8_t>> stencil_planes;
    // Host-backed regions not reachable through the port (attribute ring).
    std::vector<gfx11_interp::Buffer*> host_buffers;

private:
    void fail(const std::string& message) {
        ++stats_.errors;
        if (stats_.first_error.empty()) stats_.first_error = message;
    }

    uint32_t reg(const char* name) const {
        const auto* r = db_.find(name);
        if (r == nullptr) return 0;
        const auto found = regs_.find(r->absolute());
        return found == regs_.end() ? 0 : found->second;
    }
    uint32_t reg_at(const char* name, uint32_t index) const {
        const auto* r = db_.find(name);
        if (r == nullptr) return 0;
        const auto found = regs_.find(r->absolute() + index);
        return found == regs_.end() ? 0 : found->second;
    }
    uint32_t field(const char* reg_name, const char* field_name) const {
        const int64_t value = db_.get(reg_name, field_name, reg(reg_name));
        return value < 0 ? 0 : static_cast<uint32_t>(value);
    }
    static float as_float(uint32_t bits) { float f; std::memcpy(&f, &bits, 4); return f; }

    bool copy_in(uint64_t va, size_t bytes, gfx11_interp::Buffer& out) {
        out.base = va;
        out.bytes.assign(bytes, 0);
        for (auto* host : host_buffers) {
            if (va >= host->base && va + bytes <= host->base + host->bytes.size()) {
                std::memcpy(out.bytes.data(), host->bytes.data() + (va - host->base), bytes);
                return true;
            }
        }
        return port_.read(va, out.bytes.data(), bytes);
    }

    static uint64_t descriptor_base(uint32_t w0, uint32_t w1) { return w0 | (static_cast<uint64_t>(w1 & 0xffff) << 32); }

    void dma(const uint32_t* body) {
        const uint64_t source = body[1] | (static_cast<uint64_t>(body[2]) << 32);
        const uint64_t destination = body[3] | (static_cast<uint64_t>(body[4]) << 32);
        const uint32_t bytes = body[5] & 0x3ffffff;
        std::vector<uint8_t> data(bytes);
        if (((body[0] >> 29) & 3) != 3 || ((body[0] >> 20) & 3) != 3) { fail("DMA_DATA selects"); return; }
        if (!port_.read(source, data.data(), bytes) || !port_.write(destination, data.data(), bytes)) {
            fail("DMA_DATA translation fault");
            return;
        }
        ++stats_.dma_rows;
    }

    struct Window final { float x, y, z; };

    // A colour surface as the CB registers describe it (MRT0 or MRT1).
    struct Surface final {
        uint64_t base = 0;
        uint32_t pitch = 0, width = 0, height = 0, format = 0, swap = 0, number = 0;
    };

    Surface color_surface(uint32_t index = 0) const {
        const std::string n = "CB_COLOR" + std::to_string(index) + "_";
        Surface s;
        s.base = (static_cast<uint64_t>(field((n + "BASE").c_str(), "BASE_256B")) << 8) |
                 (static_cast<uint64_t>(field((n + "BASE_EXT").c_str(), "BASE_256B")) << 40);
        s.pitch = field((n + "ATTRIB2").c_str(), "MIP0_WIDTH") + 1;
        s.height = field((n + "ATTRIB2").c_str(), "MIP0_HEIGHT") + 1;
        s.width = field("PA_SC_WINDOW_SCISSOR_BR", "BR_X");
        s.format = field((n + "INFO").c_str(), "FORMAT");
        s.swap = field((n + "INFO").c_str(), "COMP_SWAP");
        s.number = field((n + "INFO").c_str(), "NUMBER_TYPE");
        return s;
    }

    static bool port_read(void* context, uint64_t address, uint8_t* bytes, size_t count) {
        return static_cast<Machine*>(context)->port_.read(address, bytes, count);
    }

    static float factor(uint32_t op, const float* s, const float* d, int channel) {
        const float sa = s[3], da = d[3];
        switch (op) {
        case 0: return 0; case 1: return 1;
        case 2: return s[channel]; case 3: return 1 - s[channel];
        case 4: return sa; case 5: return 1 - sa;
        case 6: return da; case 7: return 1 - da;
        case 8: return d[channel]; case 9: return 1 - d[channel];
        case 10: return channel == 3 ? 1.0f : std::fmin(sa, 1 - da);
        default: return 1;
        }
    }

    bool stencil_compare(uint32_t func, uint32_t ref, uint32_t value) const {
        switch (func) {
        case 0: return false; case 1: return ref < value; case 2: return ref == value;
        case 3: return ref <= value; case 4: return ref > value; case 5: return ref != value;
        case 6: return ref >= value; default: return true;
        }
    }
    static bool depth_compare(uint32_t func, float incoming, float stored) {
        switch (func) {
        case 0: return false; case 1: return incoming < stored; case 2: return incoming == stored;
        case 3: return incoming <= stored; case 4: return incoming > stored; case 5: return incoming != stored;
        case 6: return incoming >= stored; default: return true;
        }
    }
    static uint8_t stencil_op(uint32_t op, uint8_t value, uint8_t ref) {
        switch (op) {
        case 1: return 0;
        case 3: return ref;
        case 5: return value == 255 ? 255 : static_cast<uint8_t>(value + 1);
        case 6: return value == 0 ? 0 : static_cast<uint8_t>(value - 1);
        case 7: return static_cast<uint8_t>(~value);
        default: return value;
        }
    }

    void draw(uint32_t vertex_count) {
        ++stats_.draws;
        namespace g = gfx11_interp::graphics;
        // Programs and user data.
        const uint64_t vs = (static_cast<uint64_t>(field("SPI_SHADER_PGM_LO_ES", "MEM_BASE")) << 8) |
                            (static_cast<uint64_t>(field("SPI_SHADER_PGM_HI_ES", "MEM_BASE")) << 40);
        const uint64_t ps = (static_cast<uint64_t>(field("SPI_SHADER_PGM_LO_PS", "MEM_BASE")) << 8) |
                            (static_cast<uint64_t>(field("SPI_SHADER_PGM_HI_PS", "MEM_BASE")) << 40);
        gfx11_interp::Buffer vs_code, ps_code;
        if (!copy_in(vs, 16384, vs_code) || !copy_in(ps, 16384, ps_code)) { fail("shader fetch"); return; }
        g::DrawSetup setup{};
        setup.vs_code = vs_code.bytes.data();
        setup.vs_dwords = 4096;
        setup.ps_code = ps_code.bytes.data();
        setup.ps_dwords = 4096;
        for (uint32_t i = 0; i < 13; ++i) setup.vs_user[i] = reg_at("SPI_SHADER_USER_DATA_GS_0", i);
        for (uint32_t i = 0; i < 32; ++i) setup.ps_user[i] = reg_at("SPI_SHADER_USER_DATA_PS_0", i);
        setup.vs_user_count = field("SPI_SHADER_PGM_RSRC2_GS", "USER_SGPR");
        setup.ps_user_count = field("SPI_SHADER_PGM_RSRC2_PS", "USER_SGPR");
        const uint32_t prim = field("VGT_PRIMITIVE_TYPE", "PRIM_TYPE");
        const uint32_t vpp = prim == 1 ? 1 : prim == 2 ? 2 : 3;
        setup.vertices_per_primitive = vpp;
        setup.param_count = field("SPI_VS_OUT_CONFIG", "VS_EXPORT_COUNT") + 1;
        for (uint32_t slot = 0; slot < setup.param_count && slot < 32; ++slot) {
            if (db_.get("SPI_PS_INPUT_CNTL_0", "FLAT_SHADE", reg_at("SPI_PS_INPUT_CNTL_0", slot)) == 1)
                setup.flat_mask |= 1u << slot;
        }
        setup.provoking_last = field("PA_SU_SC_MODE_CNTL", "PROVOKING_VTX_LAST") != 0;
        setup.front_face_input = field("SPI_PS_INPUT_ENA", "FRONT_FACE_ENA") != 0;
        setup.ps_input_ena = reg("SPI_PS_INPUT_ENA");
        setup.z_format = field("SPI_SHADER_Z_FORMAT", "Z_EXPORT_FORMAT");
        // FACE=1 under the y-inverting viewport selects GL counter-clockwise fronts.
        setup.front_ccw = field("PA_SU_SC_MODE_CNTL", "FACE") != 0;
        setup.cull_front = field("PA_SU_SC_MODE_CNTL", "CULL_FRONT") != 0;
        setup.cull_back = field("PA_SU_SC_MODE_CNTL", "CULL_BACK") != 0;
        setup.point_size = static_cast<float>(field("PA_SU_POINT_SIZE", "WIDTH")) / 8.0f;

        const Surface cb = color_surface();
        const uint32_t height = cb.height;
        const float xs = as_float(reg("PA_CL_VPORT_XSCALE")), xo = as_float(reg("PA_CL_VPORT_XOFFSET"));
        const float ys = as_float(reg("PA_CL_VPORT_YSCALE")), yo = as_float(reg("PA_CL_VPORT_YOFFSET"));
        const float zs = as_float(reg("PA_CL_VPORT_ZSCALE")), zo = as_float(reg("PA_CL_VPORT_ZOFFSET"));
        const float zmin = as_float(reg("PA_SC_VPORT_ZMIN_0")), zmax = as_float(reg("PA_SC_VPORT_ZMAX_0"));
        // GL window viewport from the hardware transform (rows top-down).
        setup.viewport.width = static_cast<int32_t>(std::lround(2 * xs));
        setup.viewport.x = static_cast<int32_t>(std::lround(xo - xs));
        setup.viewport.height = static_cast<int32_t>(std::lround(-2 * ys));
        setup.viewport.y = static_cast<int32_t>(std::lround(static_cast<float>(height) - yo + ys));

        // Memory: vertex V#, attribute ring V#, T#.
        gfx11_interp::Buffer vertices, ring, texture, constants;
        const uint64_t vb = descriptor_base(setup.vs_user[0], setup.vs_user[1]);
        const uint64_t rb = descriptor_base(setup.vs_user[4], setup.vs_user[5]);
        if (!copy_in(vb, setup.vs_user[2], vertices)) { fail("vertex fetch"); return; }
        if (!copy_in(rb, setup.vs_user[6], ring)) { fail("ring fetch"); return; }
        constants.base = 0x7f0000000000ULL;
        gfx11_interp::Memory memory{};
        memory.buffers = {&vertices, &ring, &constants};
        memory.fallback = port_read;
        memory.fallback_context = this;
        if (setup.ps_user[1] != 0 || setup.ps_user[0] != 0) {
            const uint64_t tb = (static_cast<uint64_t>(setup.ps_user[0]) << 8) |
                                (static_cast<uint64_t>(setup.ps_user[1] & 0xff) << 40);
            const uint32_t th = ((setup.ps_user[2] >> 14) & 0x3fff) + 1;
            const uint32_t pitch = ((setup.ps_user[4] & 0x1fff) | (((setup.ps_user[4] >> 13) & 1) << 13)) + 1;
            if (!copy_in(tb, static_cast<size_t>(pitch) * th * 4, texture)) { fail("texture fetch"); return; }
            memory.buffers.push_back(&texture);
        }
        setup.memory = &memory;
        setup.ring_base = rb;
        setup.ring_bytes = setup.vs_user[6];
        setup.width = cb.width;
        setup.height = height;
        // The SC only scan-converts inside the viewport scissor (rows top-down).
        setup.clip_x0 = static_cast<int32_t>(field("PA_SC_VPORT_SCISSOR_0_TL", "TL_X"));
        setup.clip_x1 = static_cast<int32_t>(field("PA_SC_VPORT_SCISSOR_0_BR", "BR_X"));
        setup.clip_y0 = static_cast<int32_t>(height) - static_cast<int32_t>(field("PA_SC_VPORT_SCISSOR_0_BR", "BR_Y"));
        setup.clip_y1 = static_cast<int32_t>(height) - static_cast<int32_t>(field("PA_SC_VPORT_SCISSOR_0_TL", "TL_Y"));

        // Window positions for depth interpolation (GL y up).
        std::vector<Window> windows(vertex_count);
        for (uint32_t v = 0; v < vertex_count; ++v) {
            float c[4];
            for (int k = 0; k < 4; ++k) {
                uint32_t bits = 0;
                std::memcpy(&bits, vertices.bytes.data() + v * 80 + k * 4, 4);
                c[k] = as_float(bits);
            }
            const float inv = 1.0f / c[3];
            const float y_rows = c[1] * inv * ys + yo;
            windows[v] = {c[0] * inv * xs + xo, static_cast<float>(height) - y_rows, c[2] * inv * zs + zo};
        }

        const bool depth_copy = field("DB_RENDER_CONTROL", "DEPTH_COPY") != 0;
        const bool z_export = field("DB_SHADER_CONTROL", "Z_EXPORT_ENABLE") != 0;
        const bool stencil_export = field("DB_SHADER_CONTROL", "STENCIL_TEST_VAL_EXPORT_ENABLE") != 0;
        std::vector<g::FragmentOutput> outputs;
        setup.outputs = &outputs;
        for (uint32_t first = 0; first + vpp <= vertex_count; first += vpp) {
            ++stats_.primitives;
            setup.indices.clear();
            for (uint32_t k = 0; k < vpp; ++k) setup.indices.push_back(first + k);
            outputs.clear();
            std::vector<uint32_t> unused(1);
            setup.framebuffer = unused.data();
            const g::DrawStats run = g::run_draw(setup);
            if (run.errors != 0) {
                fail(std::string("interpreter: ") + (run.first_error ? run.first_error : ""));
                return;
            }
            for (const g::FragmentOutput& out : outputs) {
                const uint32_t x = static_cast<uint32_t>(out.x);
                const uint32_t y = static_cast<uint32_t>(out.y);
                float z = windows[first].z;
                const float px = static_cast<float>(x) + 0.5f, py = static_cast<float>(y) + 0.5f;
                if (vpp == 3) {
                    const Window& p0 = windows[first];
                    const Window& p1 = windows[first + 1];
                    const Window& p2 = windows[first + 2];
                    const float area = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
                    if (area != 0) {
                        const float w0 = ((p1.x - px) * (p2.y - py) - (p1.y - py) * (p2.x - px)) / area;
                        const float w1 = ((p2.x - px) * (p0.y - py) - (p2.y - py) * (p0.x - px)) / area;
                        z = w0 * p0.z + w1 * p1.z + (1 - w0 - w1) * p2.z;
                    }
                } else if (vpp == 2) {
                    const Window& p0 = windows[first];
                    const Window& p1 = windows[first + 1];
                    const float dx = p1.x - p0.x, dy = p1.y - p0.y, len = dx * dx + dy * dy;
                    float t = len > 0 ? ((px - p0.x) * dx + (py - p0.y) * dy) / len : 0;
                    t = t < 0 ? 0 : t > 1 ? 1 : t;
                    z = p0.z + (p1.z - p0.z) * t;
                }
                if (z_export) {
                    if (!out.depth) { fail("Z_EXPORT_ENABLE without an MRTZ depth export"); return; }
                    z = out.z;
                }
                if (stencil_export && !out.stencil) { fail("stencil export enabled without an MRTZ value"); return; }
                z = z < zmin ? zmin : z > zmax ? zmax : z;
                fragment(cb, x, height - 1 - y, z, out, depth_copy, stencil_export);
            }
        }
    }

    // One fragment at hardware row 'row' (top-down): scissors, the DB once, then
    // the CB for every exported MRT that has a target.
    void fragment(const Surface& cb, uint32_t x, uint32_t row, float z, const gfx11_interp::graphics::FragmentOutput& out,
                  bool depth_copy, bool stencil_export) {
        const uint32_t sx0 = field("PA_SC_VPORT_SCISSOR_0_TL", "TL_X");
        const uint32_t sy0 = field("PA_SC_VPORT_SCISSOR_0_TL", "TL_Y");
        const uint32_t sx1 = field("PA_SC_VPORT_SCISSOR_0_BR", "BR_X");
        const uint32_t sy1 = field("PA_SC_VPORT_SCISSOR_0_BR", "BR_Y");
        if (x < sx0 || x >= sx1 || row < sy0 || row >= sy1) return;
        if (x >= field("PA_SC_WINDOW_SCISSOR_BR", "BR_X") || row >= field("PA_SC_WINDOW_SCISSOR_BR", "BR_Y")) return;
        ++stats_.fragments;
        const bool has_db = field("DB_Z_INFO", "FORMAT") != 0;
        const uint64_t zbase = static_cast<uint64_t>(field("DB_Z_READ_BASE", "BASE_256B")) << 8;
        const size_t pixel = static_cast<size_t>(row) * cb.width + x;
        auto& zplane = depth_planes[zbase];
        auto& splane = stencil_planes[zbase];
        if (has_db) {
            zplane.resize(static_cast<size_t>(cb.width) * cb.height, 0.0f);
            splane.resize(static_cast<size_t>(cb.width) * cb.height, 0);
        }
        if (depth_copy) {
            if (!has_db) { fail("DEPTH_COPY without DB"); return; }
            uint8_t bytes[8] = {};
            std::memcpy(bytes, &zplane[pixel], 4);
            bytes[4] = splane[pixel];
            port_.write(cb.base + (static_cast<uint64_t>(row) * cb.pitch + x) * 8, bytes, 8);
            return;
        }
        if (has_db) {
            const bool stencil_on = field("DB_DEPTH_CONTROL", "STENCIL_ENABLE") != 0;
            const bool depth_on = field("DB_DEPTH_CONTROL", "Z_ENABLE") != 0;
            // STENCIL_TEST_VAL_EXPORT_ENABLE: the PS supplies the test value.
            const uint32_t ref = stencil_export ? out.stencil_value : field("DB_STENCILREFMASK", "STENCILTESTVAL");
            const uint32_t mask = field("DB_STENCILREFMASK", "STENCILMASK");
            const uint32_t wmask = field("DB_STENCILREFMASK", "STENCILWRITEMASK");
            uint8_t& s = splane[pixel];
            if (stencil_on && !stencil_compare(field("DB_DEPTH_CONTROL", "STENCILFUNC"), ref & mask, s & mask)) {
                const uint8_t next = stencil_op(field("DB_STENCIL_CONTROL", "STENCILFAIL"), s, static_cast<uint8_t>(ref));
                s = static_cast<uint8_t>((s & ~wmask) | (next & wmask));
                return;
            }
            const bool depth_pass = !depth_on || depth_compare(field("DB_DEPTH_CONTROL", "ZFUNC"), z, zplane[pixel]);
            if (stencil_on) {
                const uint32_t op = depth_pass ? field("DB_STENCIL_CONTROL", "STENCILZPASS") :
                                                 field("DB_STENCIL_CONTROL", "STENCILZFAIL");
                const uint8_t next = stencil_op(op, s, static_cast<uint8_t>(ref));
                s = static_cast<uint8_t>((s & ~wmask) | (next & wmask));
            }
            if (!depth_pass) return;
            if (depth_on && field("DB_DEPTH_CONTROL", "Z_WRITE_ENABLE") != 0) zplane[pixel] = z;
        }
        if (field("CB_COLOR_CONTROL", "MODE") == 0) return;
        for (uint32_t index = 0; index < 2; ++index) {
            if (((out.targets >> index) & 1u) == 0) continue;
            const uint32_t target = index == 0 ? field("CB_TARGET_MASK", "TARGET0_ENABLE")
                                               : field("CB_TARGET_MASK", "TARGET1_ENABLE");
            const uint32_t shader = index == 0 ? field("CB_SHADER_MASK", "OUTPUT0_ENABLE")
                                               : field("CB_SHADER_MASK", "OUTPUT1_ENABLE");
            if (target == 0 || shader == 0) continue;
            const Surface surface = index == 0 ? cb : color_surface(1);
            if (surface.format == 0) { fail("MRT1 export without a CB_COLOR1 surface"); return; }
            write_color(surface, x, row, out.color[index], target,
                        reg(index == 0 ? "CB_BLEND0_CONTROL" : "CB_BLEND1_CONTROL"));
        }
    }

    // CB blend, ROP3 and target mask on one 8_8_8_8 UNORM pixel (32_ABGR export).
    void write_color(const Surface& cb, uint32_t x, uint32_t row, const float* src, uint32_t target, uint32_t blend) {
        uint8_t mem[4] = {};
        const uint64_t address = cb.base + (static_cast<uint64_t>(row) * cb.pitch + x) * 4;
        port_.read(address, mem, 4);
        const bool alt = cb.swap == 1;
        uint8_t dst8[4] = {alt ? mem[2] : mem[0], mem[1], alt ? mem[0] : mem[2], mem[3]};
        const float dst[4] = {dst8[0] / 255.0f, dst8[1] / 255.0f, dst8[2] / 255.0f, dst8[3] / 255.0f};
        uint8_t out8[4];
        const uint32_t rop = field("CB_COLOR_CONTROL", "ROP3");
        for (int c = 0; c < 4; ++c) {
            float value = src[c];
            if (db_.get("CB_BLEND0_CONTROL", "ENABLE", blend) == 1) {
                const uint32_t sf = static_cast<uint32_t>(db_.get("CB_BLEND0_CONTROL", "COLOR_SRCBLEND", blend));
                const uint32_t df = static_cast<uint32_t>(db_.get("CB_BLEND0_CONTROL", "COLOR_DESTBLEND", blend));
                value = src[c] * factor(sf, src, dst, c) + dst[c] * factor(df, src, dst, c);
            }
            value = value < 0 ? 0 : value > 1 ? 1 : value;
            uint8_t s8 = static_cast<uint8_t>(std::lround(value * 255.0f));
            if (rop != 0xcc) {
                uint8_t r8 = 0;
                for (int bit = 0; bit < 8; ++bit) {
                    const int sb = (s8 >> bit) & 1, db = (dst8[c] >> bit) & 1;
                    r8 |= static_cast<uint8_t>(((rop >> ((sb << 1) | db)) & 1) << bit);
                }
                s8 = r8;
            }
            out8[c] = (target >> c) & 1 ? s8 : dst8[c];
        }
        const uint8_t stored[4] = {alt ? out8[2] : out8[0], out8[1], alt ? out8[0] : out8[2], out8[3]};
        port_.write(address, stored, 4);
    }

    const gfx11_header_db::Database& db_;
    MemoryPort port_;
    std::map<uint32_t, uint32_t> regs_;
    Stats stats_;
};

} // namespace gfx11_pipeline_model
