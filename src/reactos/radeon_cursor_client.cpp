/*
Module: rx7900.reactos.radeon_cursor_client
Purpose: Normalize GDI pointer shapes and drive the rx7900video pointer IOCTLs from radeondisp.
Created: 2026-10-22
Architecture: Original C++ DDI helper compiled into radeondisp. Reuses
engine_bitmap.hpp (EngCopyBits normalization) and the CommandBuffer byte writer;
the IOCTL layout is radeon_cursor_policy.hpp. ReactOS contract inspected at the
shared work/reactos tree (a97c8a2c): win32ss/gdi/eng/mouse.c IntEngSetPointerShape
falls back to EngSetPointerShape for any result other than SPS_ACCEPT_NOEXCLUDE
and moves a hardware pointer only through DrvMovePointer.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <winioctl.h>
#include <ntddvdeo.h>
#include <string.h>

#include "radeon_cursor_client.hpp"
#include "radeon_cursor_policy.hpp"
#include "engine_bitmap.hpp"
#include "sdma.hpp"

namespace rx7900::reactos::radeon_cursor {
namespace {

namespace policy = rx7900::reactos::cursor_policy;
using lightning::reactos::EngineBitmap;

constexpr ULONG allocation_tag = 0x52435852U; // "RXCR"
constexpr FLONG accepted_shape_flags = SPS_CHANGE | SPS_ALPHA | SPS_ANIMATESTART | SPS_ANIMATEUPDATE |
                                       SPS_LENGTHMASK | SPS_FREQMASK;
constexpr uint32_t needed_formats = policy::flag_mono | policy::flag_color | policy::flag_premultiplied_alpha;

bool control(HANDLE driver, DWORD code, void* input, DWORD input_bytes) noexcept {
    DWORD returned = 0;
    return EngDeviceIoControl(driver, code, input, input_bytes, nullptr, 0, &returned) == 0;
}

void probe(PointerClient& client, HANDLE driver) noexcept {
    if (client.probed) {
        return;
    }

    client.probed = true;
    VIDEO_POINTER_CAPABILITIES capabilities{};
    DWORD returned = 0;

    if (EngDeviceIoControl(driver, IOCTL_VIDEO_QUERY_POINTER_CAPABILITIES, nullptr, 0, &capabilities,
                           sizeof(capabilities), &returned) != 0 ||
        returned != sizeof(capabilities) || (capabilities.Flags & needed_formats) != needed_formats ||
        capabilities.MaxWidth == 0 || capabilities.MaxHeight == 0) {
        return;
    }

    client.capable = true;
    client.formats = capabilities.Flags;
    client.max_width = capabilities.MaxWidth < policy::max_dimension ? capabilities.MaxWidth : policy::max_dimension;
    client.max_height = capabilities.MaxHeight < policy::max_dimension ? capabilities.MaxHeight :
                                                                          policy::max_dimension;
}

void report_bounds(const PointerClient& client, int16_t column, int16_t row, bool shown, RECTL* bounds) noexcept {
    if (bounds == nullptr) {
        return;
    }

    if (!shown) {
        *bounds = RECTL{-1, -1, -1, -1};
        return;
    }

    *bounds = RECTL{column, row, column + static_cast<LONG>(client.width), row + static_cast<LONG>(client.height)};
}

void hide(PointerClient& client, HANDLE driver) noexcept {
    if (client.hardware && client.visible) {
        control(driver, IOCTL_VIDEO_DISABLE_POINTER, nullptr, 0);
    }

    client.visible = false;
}

ULONG decline(PointerClient& client, HANDLE driver) noexcept {
    hide(client, driver);
    client.hardware = false;
    return SPS_DECLINE;
}

bool trivial(XLATEOBJ* translation) noexcept {
    return translation == nullptr || (translation->flXlate & XO_TRIVIAL) != 0;
}

} // namespace

ULONG set_pointer_shape(PointerClient& client, HANDLE driver, SURFOBJ* mask, SURFOBJ* color,
                        XLATEOBJ* translation, LONG hot_x, LONG hot_y, LONG x, LONG y, RECTL* bounds,
                        FLONG flags) noexcept {
    if (driver == nullptr) {
        return SPS_DECLINE;
    }

    probe(client, driver);
    const bool alpha = (flags & SPS_ALPHA) != 0;

    if (!client.capable || (flags & ~accepted_shape_flags) != 0 || (mask == nullptr && !alpha) ||
        (alpha && color == nullptr)) {
        return decline(client, driver);
    }

    // Dimensions: a mono mask holds AND over XOR (2 * height rows).
    SURFOBJ* dimensions = color != nullptr ? color : mask;
    const LONG width = dimensions->sizlBitmap.cx;
    const LONG height = color != nullptr ? color->sizlBitmap.cy : mask->sizlBitmap.cy / 2;

    if (width <= 0 || height <= 0 || width > static_cast<LONG>(client.max_width) ||
        height > static_cast<LONG>(client.max_height) || hot_x < 0 || hot_y < 0 || hot_x >= width ||
        hot_y >= height) {
        return decline(client, driver);
    }

    if (mask != nullptr && (mask->iBitmapFormat != BMF_1BPP || mask->sizlBitmap.cx != width ||
                            mask->sizlBitmap.cy != (color != nullptr ? height : height * 2))) {
        return decline(client, driver);
    }

    if (alpha && (color->iBitmapFormat != BMF_32BPP || !trivial(translation))) {
        return decline(client, driver);
    }

    const policy::PointerFormat format = color == nullptr ? policy::PointerFormat::monochrome :
        (alpha ? policy::PointerFormat::premultiplied_alpha : policy::PointerFormat::masked_color);
    const uint32_t w = static_cast<uint32_t>(width);
    const uint32_t h = static_cast<uint32_t>(height);
    const uint32_t width_in_bytes = format == policy::PointerFormat::monochrome ? policy::mask_stride(w) : w * 4;
    const uint32_t payload = policy::payload_bytes(format, w, h, width_in_bytes);
    const POINTL origin{0, 0};
    EngineBitmap normalized_mask;
    EngineBitmap normalized_color;

    // An SPS_ALPHA shape is drawn from its color alone (the engine ignores the mask).
    if (payload == 0 ||
        (!alpha && !normalized_mask.copy(mask, width, color != nullptr ? height : height * 2, BMF_1BPP, nullptr,
                                         origin, policy::max_attributes_bytes)) ||
        (color != nullptr && !normalized_color.copy(color, width, height, BMF_32BPP, alpha ? nullptr : translation,
                                                    origin, policy::max_attributes_bytes))) {
        return decline(client, driver);
    }

    int16_t column = 0;
    int16_t row = 0;
    const bool shown = x != -1 && policy::top_left(x, y, hot_x, hot_y, column, row);
    const uint32_t bytes = policy::attributes_header_bytes + payload;
    auto* buffer = static_cast<uint8_t*>(EngAllocMem(FL_ZERO_MEMORY, bytes, allocation_tag));

    if (buffer == nullptr) {
        return decline(client, driver);
    }

    const rx7900::sdma::CommandBuffer header(buffer, bytes);
    const uint32_t kind = format == policy::PointerFormat::monochrome ? policy::flag_mono :
        (alpha ? policy::flag_color | policy::flag_premultiplied_alpha : policy::flag_color);
    uint32_t offset = policy::attributes_header_bytes;
    // The normalized planes are exactly the policy strides: EngineBitmap lays out
    // DWORD-aligned rows, i.e. mask_stride(width) for 1 bpp and width * 4 for 32 bpp.
    bool built = header.write_u32(0, kind) && header.write_u32(4, w) && header.write_u32(8, h) &&
                 header.write_u32(12, width_in_bytes) && header.write_u32(16, shown ? 1U : 0U) &&
                 header.write_u32(20, static_cast<uint16_t>(column) |
                                          (static_cast<uint32_t>(static_cast<uint16_t>(row)) << 16));

    if (built && !alpha) {
        built = normalized_mask.stride() == policy::mask_stride(w) && normalized_mask.append(buffer, bytes, offset);
    }

    if (built && color != nullptr) {
        built = normalized_color.stride() == w * 4 && normalized_color.append(buffer, bytes, offset);
    }

    built = built && offset == bytes && control(driver, IOCTL_VIDEO_SET_POINTER_ATTR, buffer, bytes);
    EngFreeMem(buffer);

    if (!built) {
        return decline(client, driver);
    }

    client.hardware = true;
    client.visible = shown;
    client.hot_x = hot_x;
    client.hot_y = hot_y;
    client.width = w;
    client.height = h;
    report_bounds(client, column, row, shown, bounds);
    return SPS_ACCEPT_NOEXCLUDE;
}

void move_pointer(PointerClient& client, HANDLE driver, LONG x, LONG y, RECTL* bounds) noexcept {
    if (!client.hardware || driver == nullptr) {
        return;
    }

    int16_t column = 0;
    int16_t row = 0;

    if (x == -1 || !policy::top_left(x, y, client.hot_x, client.hot_y, column, row)) {
        hide(client, driver);
        report_bounds(client, 0, 0, false, bounds);
        return;
    }

    VIDEO_POINTER_POSITION position{};
    position.Column = column;
    position.Row = row;

    // A failed IOCTL cannot be reported through this VOID DDI; the next shape
    // change re-establishes (or declines) the hardware cursor.
    if (!control(driver, IOCTL_VIDEO_SET_POINTER_POSITION, &position, sizeof(position))) {
        return;
    }

    if (!client.visible && control(driver, IOCTL_VIDEO_ENABLE_POINTER, nullptr, 0)) {
        client.visible = true;
    }

    report_bounds(client, column, row, client.visible, bounds);
}

void retire_pointer(PointerClient& client, HANDLE driver) noexcept {
    if (driver != nullptr && client.hardware) {
        control(driver, IOCTL_VIDEO_DISABLE_POINTER, nullptr, 0);
    }

    client.hardware = false;
    client.visible = false;
}

} // namespace rx7900::reactos::radeon_cursor
