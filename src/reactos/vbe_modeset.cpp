/*
Module: rx7900.reactos.vbe_modeset
Purpose: Native VideoPort Int10 binding and boot-firmware probe for the VBE mode set.
Created: 2026-10-22
Architecture: Thin adapter. Every VBE structure rule and the call order live in
src/display/vbe; this file only translates registers and real-mode copies.
ReactOS 0.4.16 facts relied on (win32ss/drivers/videoprt/int10.c): VideoPortInt10
zeroes SegDs/SegEs, so buffers need the Int10 interface; the i386 HAL does not
export x86BiosCall at DLL_EXPORT_VERSION 0x502, so calls run in V86 mode inside
CSRSS; Int10ReadMemory/WriteMemory form the address as (Seg << 4) | Off without
a length check, so every address is normalized (Off below 16) and bounded here.
*/
#include "vbe_modeset.hpp"

namespace rx7900::reactos {
namespace {

// Real-mode segment:offset with the offset below 16, so that ReactOS' OR of
// the shifted segment and the offset equals their sum.
struct RealModeAddress final {
    USHORT segment;
    USHORT offset;
};

bool real_mode_range(uint32_t linear, uint32_t bytes, RealModeAddress& output) noexcept {
    if (bytes == 0 || linear >= display::vbe_real_mode_limit || bytes > display::vbe_real_mode_limit - linear) {
        return false;
    }

    output = {static_cast<USHORT>(linear >> 4), static_cast<USHORT>(linear & 0xfu)};
    return true;
}

// The queried interface; outlives the sequence (stack of set_vbe_linear_mode).
struct Int10Binding final {
    VIDEO_PORT_INT10_INTERFACE services;
};

bool int10_call(void* context, display::VbeRegisters& registers) noexcept {
    const auto& services = static_cast<Int10Binding*>(context)->services;
    INT10_BIOS_ARGUMENTS arguments{};
    arguments.Eax = registers.eax;
    arguments.Ebx = registers.ebx;
    arguments.Ecx = registers.ecx;
    arguments.Edx = registers.edx;
    arguments.Edi = registers.edi;
    arguments.SegEs = registers.es;

    if (services.Int10CallBios(services.Context, &arguments) != NO_ERROR) {
        return false;
    }

    registers.eax = arguments.Eax;
    registers.ebx = arguments.Ebx;
    registers.ecx = arguments.Ecx;
    registers.edx = arguments.Edx;
    registers.edi = arguments.Edi;
    registers.es = arguments.SegEs;
    return true;
}

// output holds bytes caller-owned bytes; nothing is retained after the call.
bool int10_read(void* context, uint32_t linear, uint8_t* output, uint32_t bytes) noexcept {
    const auto& services = static_cast<Int10Binding*>(context)->services;
    RealModeAddress address{};
    return output != nullptr && real_mode_range(linear, bytes, address) &&
           services.Int10ReadMemory(services.Context, address.segment, address.offset, output, bytes) == NO_ERROR;
}

// The SDK prototype takes a non-const PVOID; videoprt only copies from it.
bool int10_write(void* context, uint32_t linear, const uint8_t* input, uint32_t bytes) noexcept {
    const auto& services = static_cast<Int10Binding*>(context)->services;
    RealModeAddress address{};
    return input != nullptr && real_mode_range(linear, bytes, address) &&
           services.Int10WriteMemory(services.Context, address.segment, address.offset,
                                      const_cast<uint8_t*>(input), bytes) == NO_ERROR;
}

constexpr ULONG adapter_key_limit = 16;
constexpr wchar_t adapter_prefix[] = L"\\Registry\\Machine\\HARDWARE\\DESCRIPTION\\System\\MultifunctionAdapter\\";
constexpr wchar_t uefi_identifier[] = L"UEFI Internal";
constexpr wchar_t isa_identifier[] = L"ISA";
constexpr size_t prefix_characters = sizeof(adapter_prefix) / sizeof(wchar_t) - 1;

// A REG_SZ value equal to expected (with or without its terminating null).
bool identifier_is(ByteView data, const wchar_t* expected, size_t characters) noexcept {
    if (data.size() != characters * 2 && data.size() != characters * 2 + 2) {
        return false;
    }

    for (size_t index = 0; index < data.size() / 2; ++index) {
        uint16_t value = 0;
        data.read_u16(index * 2, value);

        if (value != (index < characters ? static_cast<uint16_t>(expected[index]) : 0u)) {
            return false;
        }
    }

    return true;
}

// Opens MultifunctionAdapter\index; false when it does not exist.
bool read_adapter_identifier(ULONG index, BootFirmware& firmware) noexcept {
    wchar_t path[prefix_characters + 3]{};

    for (size_t character = 0; character < prefix_characters; ++character) {
        path[character] = adapter_prefix[character];
    }

    size_t length = prefix_characters;

    if (index >= 10) {
        path[length++] = static_cast<wchar_t>(L'0' + index / 10);
    }

    path[length++] = static_cast<wchar_t>(L'0' + index % 10);
    UNICODE_STRING name{};
    RtlInitUnicodeString(&name, path);
    OBJECT_ATTRIBUTES attributes{};
    InitializeObjectAttributes(&attributes, &name, OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE, nullptr, nullptr);
    HANDLE key = nullptr;

    if (!NT_SUCCESS(ZwOpenKey(&key, KEY_QUERY_VALUE, &attributes))) {
        return false;
    }

    // KEY_VALUE_PARTIAL_INFORMATION followed by room for a short identifier;
    // longer identifiers fail with STATUS_BUFFER_OVERFLOW and are not ours.
    struct ValueBuffer final {
        KEY_VALUE_PARTIAL_INFORMATION header;
        UCHAR tail[64];
    } value{};
    wchar_t value_name_text[] = L"Identifier";
    UNICODE_STRING value_name{};
    RtlInitUnicodeString(&value_name, value_name_text);
    ULONG returned = 0;
    const NTSTATUS status = ZwQueryValueKey(key, &value_name, KeyValuePartialInformation, &value,
                                            sizeof(value), &returned);
    ZwClose(key);
    const size_t data_offset = FIELD_OFFSET(KEY_VALUE_PARTIAL_INFORMATION, Data);

    if (!NT_SUCCESS(status) || value.header.Type != REG_SZ ||
        value.header.DataLength > sizeof(value) - data_offset) {
        return true;
    }

    // Data starts inside header and continues into tail; DataLength is bounded
    // by the buffer above, and the view does not outlive value.
    const ByteView data(reinterpret_cast<const uint8_t*>(&value) + data_offset, value.header.DataLength);

    if (identifier_is(data, uefi_identifier, sizeof(uefi_identifier) / sizeof(wchar_t) - 1)) {
        firmware = BootFirmware::uefi;
    } else if (identifier_is(data, isa_identifier, sizeof(isa_identifier) / sizeof(wchar_t) - 1) &&
               firmware == BootFirmware::unknown) {
        firmware = BootFirmware::bios;
    }

    return true;
}

} // namespace

BootFirmware detect_boot_firmware() noexcept {
    BootFirmware firmware = BootFirmware::unknown;

    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return firmware;
    }

    for (ULONG index = 0; index < adapter_key_limit; ++index) {
        if (!read_adapter_identifier(index, firmware)) {
            break;
        }
    }

    return firmware;
}

VbeModeset set_vbe_linear_mode(const HardwareState& hardware, PVOID extension, const PreosScanoutNative& preos,
                               VbeModesetNative& state) noexcept {
    state.report = {};
    state.firmware = BootFirmware::unknown;
    state.result = VbeModeset::invalid_state;

    if (extension == nullptr || !hardware.acquired || hardware.vram_range_bytes == 0 ||
        hardware.vram_range.QuadPart <= 0 || preos.mapping != nullptr || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return state.result;
    }

    // Only a VGA engine scanning legacy memory is replaced; a linear GOP/VBE
    // surface, or any other refusal, is never touched.
    if (preos.result != PreosAdoption::decode_refused || preos.report.status != display::PreosStatus::legacy_vga) {
        state.result = VbeModeset::not_legacy_vga;
        return state.result;
    }

    // After a UEFI boot the real-mode vector table has no VBIOS behind INT 10h.
    state.firmware = detect_boot_firmware();

    if (state.firmware != BootFirmware::bios) {
        state.result = VbeModeset::firmware_not_bios;
        return state.result;
    }

    Int10Binding binding{};
    binding.services.Size = sizeof(binding.services);
    binding.services.Version = VIDEO_PORT_INT10_INTERFACE_VERSION_1;

    // VIDEO_PORT_INT10_INTERFACE begins with the INTERFACE header, as the
    // VideoPortQueryServices contract requires.
    if (VideoPortQueryServices(extension, VideoPortServicesInt10, reinterpret_cast<PINTERFACE>(&binding.services)) !=
        NO_ERROR) {
        state.result = VbeModeset::interface_unavailable;
        return state.result;
    }

    const auto& services = binding.services;
    auto release_interface = [&services]() noexcept {
        if (services.InterfaceDereference != nullptr) {
            services.InterfaceDereference(services.Context);
        }
    };

    if (services.Int10AllocateBuffer == nullptr || services.Int10FreeBuffer == nullptr ||
        services.Int10ReadMemory == nullptr || services.Int10WriteMemory == nullptr ||
        services.Int10CallBios == nullptr) {
        release_interface();
        state.result = VbeModeset::interface_unavailable;
        return state.result;
    }

    USHORT segment = 0;
    USHORT offset = 0;
    ULONG length = display::vbe_scratch_bytes;

    if (services.Int10AllocateBuffer(services.Context, &segment, &offset, &length) != NO_ERROR) {
        release_interface();
        state.result = VbeModeset::buffer_unavailable;
        return state.result;
    }

    const uint32_t linear = static_cast<uint32_t>(segment) * 16u + offset;
    RealModeAddress check{};
    VbeModeset result = VbeModeset::buffer_unavailable;

    if (length >= display::vbe_scratch_bytes && real_mode_range(linear, display::vbe_scratch_bytes, check)) {
        const display::VbeBios bios{&binding, linear, display::vbe_scratch_bytes, int10_call, int10_read,
                                    int10_write};
        const display::VbeAperture aperture{static_cast<uint64_t>(hardware.vram_range.QuadPart),
                                            hardware.vram_range_bytes};
        result = display::set_vbe_linear_mode(bios, aperture, state.workspace, state.report) ==
                         display::VbeStatus::mode_set
                     ? VbeModeset::mode_set
                     : VbeModeset::refused;
    }

    // ReactOS' V86 free can report failure for its fixed 0x20000 buffer; the
    // pages stay in the CSRSS real-mode area that videoprt reuses, so the
    // result does not change the outcome.
    services.Int10FreeBuffer(services.Context, segment, offset);
    release_interface();
    state.result = result;
    return state.result;
}

} // namespace rx7900::reactos
