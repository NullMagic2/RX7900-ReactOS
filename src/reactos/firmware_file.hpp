/*
Module: rx7900.reactos.firmware_file
Purpose: Acquire immutable firmware using native NT file APIs and scoped cleanup.
Created: 2026-10-21
Architecture: PASSIVE_LEVEL acquisition; PSP upload/authentication is separate.
*/
#pragma once

#include <ntddk.h>
#include "driver_blob.hpp"

namespace rx7900::reactos {

class FirmwareBlob final {
public:
    FirmwareBlob() noexcept = default;
    FirmwareBlob(const FirmwareBlob&) = delete;
    FirmwareBlob& operator=(const FirmwareBlob&) = delete;
    ~FirmwareBlob() noexcept { reset(); }

    // PASSIVE_LEVEL with special kernel APCs enabled; filename is selected from FirmwareKind under the fixed
    // SystemRoot firmware directory. Existing data is never replaced implicitly.
    // Loads an exact bounded file into nonpaged pool and validates its metadata.
    // This never maps the GPU, stages DMA or declares firmware authenticated.
    NTSTATUS load(hardware::FirmwareKind kind) noexcept;
    void reset() noexcept;
    ByteView bytes() const noexcept { return blob_.bytes(); }
    const hardware::FirmwareImage& image() const noexcept { return image_; }

private:
    // The object is the sole owner of this native pool allocation. Views cannot
    // outlive it; reset/destruction require all staging readers to have retired.
    DriverBlob blob_{};
    hardware::FirmwareImage image_{};
};

} // namespace rx7900::reactos
