/*
Module: rx7900.reactos.driver_blob
Purpose: Share exact native file acquisition and nonpaged blob ownership.
Created: 2026-10-21
Architecture: Firmware loaders and the VideoPort ROM owner reuse this boundary.
*/
#pragma once
#include <ntddk.h>
#include "firmware.hpp"
#include "ucode.hpp"
#include "atom_bios.hpp"

namespace rx7900::reactos {
class DriverBlob;

// Device-extension ownership. Noncopyable and trivially destructible; the native
// owner must call DriverBlob::release after all borrowed views have retired.
class BlobStorage final {
public:
    BlobStorage() noexcept = default;
    BlobStorage(const BlobStorage&) = delete;
    BlobStorage& operator=(const BlobStorage&) = delete;
    ByteView bytes() const noexcept { return ByteView(data_, length_); }
private:
    friend class DriverBlob;
    uint8_t* data_ = nullptr;
    ULONG length_ = 0;
};

class DriverBlob final {
public:
    DriverBlob() noexcept = default;
    DriverBlob(const DriverBlob&) = delete;
    DriverBlob& operator=(const DriverBlob&) = delete;
    ~DriverBlob() noexcept { reset(); }
    // PASSIVE_LEVEL, special APCs enabled. Known names only, no caller paths.
    NTSTATUS load_firmware(hardware::FirmwareKind kind) noexcept;
    // Fixed Navi31 PSP payload names from hardware::ucode_filename and the PSP TA;
    // container validation belongs to UcodeCatalog, not to this file owner.
    NTSTATUS load_ucode(hardware::UcodeSource source, uint8_t pci_revision, bool fallback) noexcept;
    NTSTATUS load_psp_ta() noexcept;
    NTSTATUS load_atom_rom() noexcept;
    // PASSIVE_LEVEL. Copy a caller-validated in-memory ROM candidate of 1 to
    // maximum_atom_file_bytes into new exact nonpaged storage owned by this blob
    // (src/reactos/vbios card sources). STATUS_DEVICE_BUSY if it already owns bytes.
    NTSTATUS load_bytes(ByteView source) noexcept;
    ByteView bytes() const noexcept { return storage_.bytes(); }
    void reset() noexcept;
    // Transfer into empty native owner storage without extending view lifetime.
    bool detach(BlobStorage& destination) noexcept;
    static void release(BlobStorage& storage) noexcept;
private:
    NTSTATUS load_known(const char* name, ULONG minimum, ULONG maximum) noexcept;
    BlobStorage storage_{};
};
} // namespace rx7900::reactos
