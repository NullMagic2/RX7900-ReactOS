/*
Module: rx7900.reactos.firmware_file
Purpose: Validate native acquired firmware through the shared container parser.
Created: 2026-10-21
Architecture: DriverBlob owns file/pool lifetime; PSP authentication is separate.
*/
#include "firmware_file.hpp"

namespace rx7900::reactos {
void FirmwareBlob::reset() noexcept {
    blob_.reset();
    image_ = {};
}

NTSTATUS FirmwareBlob::load(hardware::FirmwareKind kind) noexcept {
    const NTSTATUS status = blob_.load_firmware(kind);

    if (!NT_SUCCESS(status)) { return status; }

    hardware::FirmwareImage candidate{};

    if (hardware::validate_firmware(blob_.bytes(), kind, candidate) != hardware::FirmwareStatus::success) {
        blob_.reset();
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    image_ = candidate;
    return STATUS_SUCCESS;
}
} // namespace rx7900::reactos
