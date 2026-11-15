/*
Module: rx7900.reactos.vbios
Purpose: Copy candidate VBIOS images from the card with VideoPort and validate them.
Created: 2026-10-22
Architecture: Thin NT/VideoPort owner of the legacy shadow mapping, the
expansion-ROM BAR enable/restore transaction and the SMUIO register adapter;
the Linux v6.12 algorithms and validity checks live in src/hardware/vbios_rom.

Linux amdgpu_get_bios_dgpu order and what this miniport can do with each source:
1. ATRM (ACPI method on a display device node): unavailable. ReactOS acpi.sys
   evaluates methods only for IRPs sent down a device stack (drivers/bus/acpi/
   main.c IOCTL_ACPI_EVAL_METHOD), and VideoPort gives a miniport no device
   object or stack to send one through.
2. ACPI VFCT table: unavailable. NtQuerySystemInformation class
   SystemFirmwareTableInformation returns STATUS_NOT_IMPLEMENTED for the 'ACPI'
   provider (ntoskrnl/ex/sysinfo.c), AuxKlibGetSystemFirmwareTable is a
   STATUS_NOT_IMPLEMENTED stub (sdk/lib/drivers/aux_klib/aux_klib.c), the HAL
   never sets HalDispatchTable->HalGetCachedAcpiTable (ntoskrnl/fstub/halstub.c)
   and acpi.sys publishes only DSDT/FACS/FADT/RSDT under HARDWARE\ACPI
   (drivers/bus/acpi/busmgr/utils.c). hardware::find_vfct_image is ready for a
   future table source.
3. VRAM BAR copy: skipped. Linux keeps it for SR-IOV virtual functions; on bare
   metal its amdgpu_device_need_post gate reads register dword 7 because the
   ATOM scratch offset is still zero before any BIOS is parsed.
4. Platform ROM (EFI-provided pdev->rom): unavailable; the ReactOS loader passes
   no PCI option-ROM images.
5. ROM BAR (pci_map_rom): for the boot VGA function Linux pci_fixup_video
   substitutes the legacy 0xC0000 128 KiB shadow, otherwise it enables the
   expansion ROM decode only for the copy. Both are implemented here; ReactOS
   pci.sys never assigns ROM BARs (drivers/bus/pci/pdo.c "FIXME: Check ROM
   address"), so only a firmware-programmed ROM BAR address is used, and
   VideoPortGetRomImage is not used (it ignores the device and truncates at 64 KiB).
6. SMUIO ROM_INDEX/ROM_DATA (amdgpu_read_bios_from_rom): implemented.
7. Disabled ROM BAR (soc21_read_disabled_bios): Linux returns false.
*/
#include "vbios.hpp"

namespace rx7900::reactos {
namespace {

constexpr ULONG vbios_pool_tag = 0x42565852u;
constexpr ULONG pci_command_offset = 0x04;
constexpr ULONG pci_rom_bar_offset = 0x30;
constexpr uint16_t pci_command_io = 0x0001;
constexpr uint16_t pci_command_memory = 0x0002;
constexpr uint32_t pci_rom_address_mask = 0xfffff800u;
constexpr uint32_t pci_rom_enable = 0x00000001u;
constexpr uint32_t minimum_rom_window_bytes = 0x800;
constexpr uint64_t legacy_region_end = 0x100000;
constexpr ULONG stall_limit_microseconds = 50;

size_t source_index(VbiosSource source) noexcept {
    return static_cast<size_t>(source);
}

void record(VbiosReport& report, VbiosSource source, VbiosAttempt attempt,
            hardware::VbiosStatus validation = hardware::VbiosStatus::success) noexcept {
    report.attempts[source_index(source)] = attempt;
    report.validation[source_index(source)] = validation;
}

// Scoped nonpaged scratch for one candidate; freed on every path.
class PoolBuffer final {
public:
    PoolBuffer() noexcept = default;
    PoolBuffer(const PoolBuffer&) = delete;
    PoolBuffer& operator=(const PoolBuffer&) = delete;
    ~PoolBuffer() noexcept {
        if (data_ != nullptr) {
            ExFreePoolWithTag(data_, vbios_pool_tag);
        }
    }

    bool allocate(ULONG bytes) noexcept {
        if (data_ != nullptr || bytes == 0) {
            return false;
        }

        data_ = static_cast<uint8_t*>(ExAllocatePoolWithTag(NonPagedPool, bytes, vbios_pool_tag));
        bytes_ = data_ != nullptr ? bytes : 0;
        return data_ != nullptr;
    }

    // Exclusive writable view of the owned bytes for the duration of one copy.
    sdma::CommandBuffer writable() noexcept { return sdma::CommandBuffer(data_, bytes_); }
    ByteView bytes(uint32_t count) const noexcept {
        ByteView view(nullptr, 0);
        ByteView(data_, bytes_).subview(0, count, view);
        return view;
    }

private:
    uint8_t* data_ = nullptr;
    ULONG bytes_ = 0;
};

// One VideoPort device-base mapping of a bus memory range, released on scope exit.
class MappedRange final {
public:
    MappedRange(PVOID extension, uint64_t bus_address, uint32_t bytes) noexcept
        : extension_(extension) {
        PHYSICAL_ADDRESS address{};
        address.QuadPart = static_cast<LONGLONG>(bus_address);
        // VideoPort translates the bus-relative range and maps it uncached.
        base_ = static_cast<PULONG>(VideoPortGetDeviceBase(extension, address, bytes,
                                                           VIDEO_MEMORY_SPACE_MEMORY));
        bytes_ = base_ != nullptr ? bytes : 0;
    }
    MappedRange(const MappedRange&) = delete;
    MappedRange& operator=(const MappedRange&) = delete;
    ~MappedRange() noexcept {
        if (base_ != nullptr) {
            VideoPortFreeDeviceBase(extension_, base_);
        }
    }

    bool mapped() const noexcept { return base_ != nullptr; }

    hardware::RomWindow window() const noexcept { return {this, bytes_, read}; }

private:
    // Only this checked boundary forms a pointer into the mapping: the offset is
    // dword aligned and the whole dword lies below the mapped length.
    static bool read(const void* context, uint32_t offset, uint32_t& value) noexcept {
        const auto& self = *static_cast<const MappedRange*>(context);

        if (self.base_ == nullptr || (offset & 3u) != 0 || offset > self.bytes_ ||
            sizeof(ULONG) > self.bytes_ - offset) {
            return false;
        }

        value = VideoPortReadRegisterUlong(self.base_ + offset / sizeof(ULONG));
        return true;
    }

    PVOID extension_;
    PULONG base_ = nullptr;
    uint32_t bytes_ = 0;
};

bool read_config32(const HardwareState& state, PVOID extension, ULONG offset,
                   uint32_t& value) noexcept {
    uint8_t bytes[4]{};

    if (VideoPortGetBusData(extension, PCIConfiguration, state.pci_slot, bytes, offset,
                            sizeof(bytes)) != sizeof(bytes)) {
        return false;
    }

    return ByteView(bytes, sizeof(bytes)).read_u32(0, value);
}

bool write_config32(const HardwareState& state, PVOID extension, ULONG offset,
                    uint32_t value) noexcept {
    uint8_t bytes[4]{};
    sdma::CommandBuffer(bytes, sizeof(bytes)).write_u32(0, value);
    return VideoPortSetBusData(extension, PCIConfiguration, state.pci_slot, bytes, offset,
                               sizeof(bytes)) == sizeof(bytes);
}

// Write then confirm by readback (Linux pci_write_config_dword has no readback).
bool write_config32_confirmed(const HardwareState& state, PVOID extension, ULONG offset,
                              uint32_t value, uint32_t expected_mask) noexcept {
    uint32_t readback = 0;
    return write_config32(state, extension, offset, value) &&
           read_config32(state, extension, offset, readback) &&
           (readback & expected_mask) == (value & expected_mask);
}

// Pool copy of [0, bytes) of a ROM window, rounded up to whole dwords inside it.
bool copy_window(const hardware::RomWindow& window, uint32_t bytes, PoolBuffer& output) noexcept {
    const uint32_t rounded = (bytes + 3u) & ~3u;

    if (bytes == 0 || rounded < bytes || rounded > window.bytes || !output.allocate(rounded)) {
        return false;
    }

    const sdma::CommandBuffer target = output.writable();

    for (uint32_t offset = 0; offset < rounded; offset += sizeof(uint32_t)) {
        uint32_t value = 0;

        if (!window.read32(window.context, offset, value) || !target.write_u32(offset, value)) {
            return false;
        }
    }

    return true;
}

// Linux pci_map_rom/pci_get_rom_size/memcpy_fromio over one mapped window:
// the copy holds exactly the walked image extent (dword rounded inside it).
VbiosAttempt copy_rom_images(const MappedRange& mapping, PoolBuffer& output,
                             uint32_t& bytes) noexcept {
    bytes = 0;

    if (!mapping.mapped()) {
        return VbiosAttempt::failed;
    }

    uint32_t extent = 0;

    if (!hardware::pci_rom_image_bytes(mapping.window(), extent)) {
        return VbiosAttempt::failed;
    }

    if (extent == 0) {
        return VbiosAttempt::absent;
    }

    // Linux maps the whole window; images beyond the shared 1 MiB validator
    // limit cannot be accepted, so the copy never exceeds it.
    if (extent > maximum_atom_file_bytes) {
        extent = maximum_atom_file_bytes;
    }

    if (!copy_window(mapping.window(), extent, output)) {
        return VbiosAttempt::failed;
    }

    bytes = extent;
    return VbiosAttempt::accepted;
}

// Publish an accepted candidate into the same storage acquire_atom_rom fills.
NTSTATUS publish(HardwareState& state, ByteView candidate, const AtomImage& image) noexcept {
    DriverBlob blob;
    const NTSTATUS status = blob.load_bytes(candidate);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (!blob.detach(state.rom)) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    state.rom_image_bytes = image.image_bytes;
    return STATUS_SUCCESS;
}

// Validation shared by every card source, then publication.
VbiosAttempt accept(HardwareState& state, VbiosSource source, ByteView candidate,
                    VbiosReport& report, NTSTATUS& status) noexcept {
    AtomImage image{};
    hardware::VbiosStatus validation = hardware::validate_vbios_image(
        candidate, state.adapter.vendor_id, state.adapter.device_id, image);

    if (validation == hardware::VbiosStatus::success && source == VbiosSource::rom_shadow) {
        validation = hardware::check_atom_subsystem(candidate, state.adapter.subsystem_vendor_id,
                                                    state.adapter.subsystem_device_id);
    }

    if (validation != hardware::VbiosStatus::success) {
        record(report, source, VbiosAttempt::rejected, validation);
        return VbiosAttempt::rejected;
    }

    status = publish(state, candidate, image);

    if (!NT_SUCCESS(status)) {
        record(report, source, VbiosAttempt::failed);
        return VbiosAttempt::failed;
    }

    record(report, source, VbiosAttempt::accepted);
    report.source = source;
    report.candidate_bytes = static_cast<uint32_t>(candidate.size());
    report.image_bytes = image.image_bytes;
    return VbiosAttempt::accepted;
}

// Linux pci_fixup_video gives the boot VGA function, when its COMMAND has I/O or
// memory decode, the 0xC0000 shadow instead of its ROM BAR. VGA routing through
// the upstream bridges is not visible to a miniport, so the shadow is accepted
// only when its PCIR identity and ATOM subsystem IDs both match this function.
VbiosAttempt try_rom_shadow(HardwareState& state, PVOID extension, uint16_t command,
                            VbiosReport& report, NTSTATUS& status) noexcept {
    if ((command & (pci_command_io | pci_command_memory)) == 0) {
        record(report, VbiosSource::rom_shadow, VbiosAttempt::skipped);
        return VbiosAttempt::skipped;
    }

    PoolBuffer copy;
    uint32_t bytes = 0;
    VbiosAttempt attempt = VbiosAttempt::failed;

    {
        const MappedRange shadow(extension, hardware::legacy_rom_shadow_address,
                                 hardware::legacy_rom_shadow_bytes);
        attempt = copy_rom_images(shadow, copy, bytes);
    }

    if (attempt != VbiosAttempt::accepted) {
        record(report, VbiosSource::rom_shadow, attempt);
        return attempt;
    }

    return accept(state, VbiosSource::rom_shadow, copy.bytes(bytes), report, status);
}

// Restores the exact original ROM BAR value when the enable transaction ends.
class RomBarRestore final {
public:
    RomBarRestore(const HardwareState& state, PVOID extension, uint32_t original,
                  VbiosReport& report) noexcept
        : state_(state), extension_(extension), original_(original), report_(report) {}
    RomBarRestore(const RomBarRestore&) = delete;
    RomBarRestore& operator=(const RomBarRestore&) = delete;
    ~RomBarRestore() noexcept {
        if (!write_config32_confirmed(state_, extension_, pci_rom_bar_offset, original_, UINT32_MAX)) {
            report_.rom_bar_restored = false;
        }
    }

private:
    const HardwareState& state_;
    PVOID extension_;
    uint32_t original_;
    VbiosReport& report_;
};

bool overlaps(uint64_t start, uint64_t bytes, uint64_t other, uint64_t other_bytes) noexcept {
    if (other == 0) {
        return false;
    }

    const uint64_t other_extent = other_bytes != 0 ? other_bytes : 1;
    return start < other + other_extent && other < start + bytes;
}

// The firmware-assigned ROM window must be naturally aligned, above the legacy
// region, below 4 GiB and disjoint from this function's own assigned BARs.
bool rom_window_usable(const HardwareState& state, uint32_t address, uint32_t window) noexcept {
    const uint64_t start = address;
    const uint64_t vram = state.vram_range.QuadPart > 0 ?
        static_cast<uint64_t>(state.vram_range.QuadPart) : 0;
    const uint64_t doorbell = state.doorbell_range.QuadPart > 0 ?
        static_cast<uint64_t>(state.doorbell_range.QuadPart) : 0;

    return (address & (window - 1)) == 0 && start >= legacy_region_end &&
           start + window <= (1ULL << 32) &&
           !overlaps(start, window, state.adapter.bars[5].bus_address, state.register_bytes) &&
           !overlaps(start, window, vram, state.vram_range_bytes) &&
           !overlaps(start, window, doorbell, state.doorbell_range_bytes);
}

// Linux pci_map_rom/pci_enable_rom/pci_unmap_rom for a non-shadowed function.
VbiosAttempt try_rom_bar(HardwareState& state, PVOID extension, uint16_t command,
                         VbiosReport& report, NTSTATUS& status) noexcept {
    uint32_t original = 0;

    if (!read_config32(state, extension, pci_rom_bar_offset, original)) {
        record(report, VbiosSource::rom_bar, VbiosAttempt::failed);
        return VbiosAttempt::failed;
    }

    report.rom_bar_original = original;
    const uint32_t address = original & pci_rom_address_mask;

    // ROM decode also needs COMMAND memory decode, which this owner never changes;
    // an unassigned ROM BAR is not given an address (Linux would assign one).
    if ((command & pci_command_memory) == 0 || address == 0) {
        record(report, VbiosSource::rom_bar,
               address == 0 ? VbiosAttempt::absent : VbiosAttempt::skipped);
        return report.attempts[source_index(VbiosSource::rom_bar)];
    }

    PoolBuffer copy;
    uint32_t bytes = 0;
    VbiosAttempt attempt = VbiosAttempt::failed;

    {
        const RomBarRestore restore(state, extension, original, report);
        uint32_t sized = 0;

        // Size with the decode bit clear: the device ignores the ROM address
        // while ROM decode is disabled, so COMMAND need not be touched.
        if (!write_config32(state, extension, pci_rom_bar_offset,
                            pci_rom_address_mask | (original & ~(pci_rom_address_mask | pci_rom_enable))) ||
            !read_config32(state, extension, pci_rom_bar_offset, sized) ||
            !write_config32_confirmed(state, extension, pci_rom_bar_offset, original, UINT32_MAX)) {
            record(report, VbiosSource::rom_bar, VbiosAttempt::failed);
            return VbiosAttempt::failed;
        }

        const uint32_t size_mask = sized & pci_rom_address_mask;
        const uint32_t window = size_mask != 0 ? ~size_mask + 1u : 0;
        report.rom_bar_window_bytes = window;

        if (window < minimum_rom_window_bytes) {
            record(report, VbiosSource::rom_bar, VbiosAttempt::absent);
            return VbiosAttempt::absent;
        }

        if (!rom_window_usable(state, address, window)) {
            record(report, VbiosSource::rom_bar, VbiosAttempt::skipped);
            return VbiosAttempt::skipped;
        }

        // pci_enable_rom: keep the low bits, set the assigned address and enable.
        if ((original & pci_rom_enable) == 0 &&
            !write_config32_confirmed(state, extension, pci_rom_bar_offset, original | pci_rom_enable,
                                      pci_rom_address_mask | pci_rom_enable)) {
            record(report, VbiosSource::rom_bar, VbiosAttempt::failed);
            return VbiosAttempt::failed;
        }

        const uint32_t mapped = window < maximum_atom_file_bytes ? window : maximum_atom_file_bytes;
        const MappedRange rom(extension, address, mapped);
        attempt = copy_rom_images(rom, copy, bytes);
        // The mapping is released before the restore, as pci_unmap_rom does.
    }

    if (!report.rom_bar_restored) {
        record(report, VbiosSource::rom_bar, VbiosAttempt::failed);
        return VbiosAttempt::failed;
    }

    if (attempt != VbiosAttempt::accepted) {
        record(report, VbiosSource::rom_bar, attempt);
        return attempt;
    }

    return accept(state, VbiosSource::rom_bar, copy.bytes(bytes), report, status);
}

bool mmio_read(void* context, uint32_t offset, uint32_t& value) noexcept {
    return read_register(*static_cast<HardwareState*>(context), offset, value);
}

bool mmio_write(void* context, uint32_t offset, uint32_t value) noexcept {
    return write_register(*static_cast<HardwareState*>(context), offset, value);
}

// PASSIVE_LEVEL only: short polls stall, the 1 ms IFWI waits yield.
void wait(void*, uint32_t microseconds) noexcept {
    if (microseconds <= stall_limit_microseconds) {
        KeStallExecutionProcessor(microseconds);
        return;
    }

    LARGE_INTEGER interval{};
    interval.QuadPart = -10LL * static_cast<LONGLONG>(microseconds);
    KeDelayExecutionThread(KernelMode, FALSE, &interval);
}

// Linux amdgpu_read_bios_from_rom after IP discovery (which precedes
// amdgpu_get_bios in amdgpu_device_ip_early_init).
VbiosAttempt try_smuio_rom(HardwareState& state, hardware::DiscoveryBinary& discovery,
                           VbiosReport& report, NTSTATUS& status) noexcept {
    const hardware::RegisterIo io{&state, state.register_bytes, mmio_read, mmio_write, wait};

    if (discovery.bytes().size() == 0) {
        hardware::VramDiscoveryReport discovery_report{};
        report.discovery = hardware::read_vram_discovery(io, discovery, discovery_report);
    } else {
        report.discovery = hardware::VramDiscoveryStatus::acquired;
    }

    hardware::NbioRegisters nbio_registers;
    hardware::SmuioRomRegisters smuio;
    hardware::Nbio nbio;

    if (report.discovery != hardware::VramDiscoveryStatus::acquired ||
        hardware::validate_nbio_discovery(discovery.bytes(), state.register_bytes, nbio_registers) !=
            hardware::DiscoveryStatus::validated ||
        hardware::validate_smuio_rom_discovery(discovery.bytes(), nbio_registers, smuio) !=
            hardware::DiscoveryStatus::validated ||
        nbio.initialize(nbio_registers, io) != hardware::NbioStatus::ready) {
        record(report, VbiosSource::smuio_rom, VbiosAttempt::skipped,
               hardware::VbiosStatus::unsupported_ip);
        return VbiosAttempt::skipped;
    }

    PoolBuffer copy;

    if (!copy.allocate(hardware::maximum_smuio_rom_bytes)) {
        record(report, VbiosSource::smuio_rom, VbiosAttempt::failed,
               hardware::VbiosStatus::insufficient_storage);
        return VbiosAttempt::failed;
    }

    uint32_t bytes = 0;
    report.smuio = hardware::read_smuio_rom(io, smuio, nbio, copy.writable(), bytes);

    if (report.smuio.status != hardware::VbiosStatus::success) {
        const bool failed = report.smuio.status == hardware::VbiosStatus::mmio_failure;
        record(report, VbiosSource::smuio_rom, failed ? VbiosAttempt::failed : VbiosAttempt::rejected,
               report.smuio.status);
        return failed ? VbiosAttempt::failed : VbiosAttempt::rejected;
    }

    return accept(state, VbiosSource::smuio_rom, copy.bytes(bytes), report, status);
}

VbiosAttempt try_administrator_file(HardwareState& state, VbiosReport& report) noexcept {
    DriverBlob candidate;
    report.file_status = candidate.load_atom_rom();

    if (!NT_SUCCESS(report.file_status)) {
        record(report, VbiosSource::administrator_file,
               report.file_status == STATUS_OBJECT_NAME_NOT_FOUND ? VbiosAttempt::absent :
                                                                    VbiosAttempt::failed);
        return report.attempts[source_index(VbiosSource::administrator_file)];
    }

    AtomImage image{};
    const hardware::VbiosStatus validation = hardware::validate_vbios_image(
        candidate.bytes(), state.adapter.vendor_id, state.adapter.device_id, image);

    if (validation != hardware::VbiosStatus::success) {
        report.file_status = STATUS_INVALID_IMAGE_FORMAT;
        record(report, VbiosSource::administrator_file, VbiosAttempt::rejected, validation);
        return VbiosAttempt::rejected;
    }

    const uint32_t candidate_bytes = static_cast<uint32_t>(candidate.bytes().size());

    if (!candidate.detach(state.rom)) {
        report.file_status = STATUS_INVALID_DEVICE_STATE;
        record(report, VbiosSource::administrator_file, VbiosAttempt::failed);
        return VbiosAttempt::failed;
    }

    state.rom_image_bytes = image.image_bytes;
    record(report, VbiosSource::administrator_file, VbiosAttempt::accepted);
    report.source = VbiosSource::administrator_file;
    report.candidate_bytes = candidate_bytes;
    report.image_bytes = image.image_bytes;
    return VbiosAttempt::accepted;
}

} // namespace

NTSTATUS acquire_vbios(HardwareState& state, PVOID extension, VbiosPrecedence precedence,
                       hardware::DiscoveryBinary& discovery, VbiosReport& report) noexcept {
    report = {};

    if (!state.acquired || extension == nullptr || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    if (state.rom.bytes().size() != 0) {
        return STATUS_DEVICE_BUSY;
    }

    if (precedence == VbiosPrecedence::administrator_file_only) {
        return try_administrator_file(state, report) == VbiosAttempt::accepted ?
            STATUS_SUCCESS : report.file_status;
    }

    record(report, VbiosSource::atrm, VbiosAttempt::unavailable);
    record(report, VbiosSource::vfct, VbiosAttempt::unavailable);
    record(report, VbiosSource::vram, VbiosAttempt::skipped);
    record(report, VbiosSource::platform, VbiosAttempt::unavailable);

    uint32_t command_dword = 0;
    NTSTATUS status = STATUS_NOT_FOUND;

    if (!read_config32(state, extension, pci_command_offset, command_dword)) {
        record(report, VbiosSource::rom_shadow, VbiosAttempt::failed);
        record(report, VbiosSource::rom_bar, VbiosAttempt::failed);
    } else {
        const uint16_t command = static_cast<uint16_t>(command_dword);

        if (try_rom_shadow(state, extension, command, report, status) == VbiosAttempt::accepted) {
            return status;
        }

        // Linux uses the shadow instead of the ROM BAR; a rejected shadow here
        // falls through to the function's own ROM BAR before the SMUIO read.
        const VbiosAttempt rom_bar = try_rom_bar(state, extension, command, report, status);

        if (rom_bar == VbiosAttempt::accepted) {
            return status;
        }

        if (!report.rom_bar_restored) {
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }
    }

    if (try_smuio_rom(state, discovery, report, status) == VbiosAttempt::accepted) {
        return status;
    }

    if (try_administrator_file(state, report) == VbiosAttempt::accepted) {
        return STATUS_SUCCESS;
    }

    return STATUS_NOT_FOUND;
}

} // namespace rx7900::reactos
