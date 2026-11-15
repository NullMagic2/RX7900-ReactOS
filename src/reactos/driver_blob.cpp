/*
Module: rx7900.reactos.driver_blob
Purpose: Load fixed firmware and ROM filenames through bounded native file reads.
Created: 2026-10-21
Architecture: Shared native file/pool ownership; common parsers own decoding.
*/
#include "driver_blob.hpp"

namespace rx7900::reactos {
namespace {

constexpr ULONG blob_pool_tag = 0x46525852u;

class FileHandle final {
public:
    FileHandle() noexcept = default;
    FileHandle(const FileHandle&) = delete;
    FileHandle& operator=(const FileHandle&) = delete;
    ~FileHandle() noexcept {
        if (value != nullptr) {
            ZwClose(value);
        }
    }
    HANDLE value = nullptr;
};

bool native_filename(const char* filename, wchar_t (&output)[128]) noexcept {
    constexpr wchar_t prefix[] = L"\\SystemRoot\\System32\\drivers\\rx7900\\";
    constexpr size_t prefix_length = RTL_NUMBER_OF(prefix) - 1;

    if (filename == nullptr) {
        return false;
    }

    for (size_t index = 0; index < prefix_length; ++index) {
        output[index] = prefix[index];
    }

    for (size_t index = 0; index < RTL_NUMBER_OF(output) - prefix_length; ++index) {
        const unsigned char character = static_cast<unsigned char>(filename[index]);

        if (character > 0x7f) {
            return false;
        }

        output[prefix_length + index] = static_cast<wchar_t>(character);

        if (character == 0) {
            return true;
        }
    }

    return false;
}

} // namespace

void DriverBlob::reset() noexcept {
    release(storage_);
}

NTSTATUS DriverBlob::load_known(const char* basename, ULONG minimum, ULONG maximum) noexcept {
    if (KeGetCurrentIrql() != PASSIVE_LEVEL || KeAreAllApcsDisabled()) {
        return STATUS_INVALID_DEVICE_STATE;
    }
    if (storage_.data_ != nullptr) {
        return STATUS_DEVICE_BUSY;
    }

    wchar_t filename[128]{};

    if (!native_filename(basename, filename)) {
        return STATUS_INVALID_PARAMETER;
    }

    UNICODE_STRING name{};
    RtlInitUnicodeString(&name, filename);
    OBJECT_ATTRIBUTES attributes{};
    InitializeObjectAttributes(&attributes, &name,
        OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE, nullptr, nullptr);
    IO_STATUS_BLOCK io{};
    FileHandle file;
    NTSTATUS status = ZwOpenFile(&file.value, FILE_READ_DATA | FILE_READ_ATTRIBUTES | SYNCHRONIZE,
        &attributes, &io, FILE_SHARE_READ,
        FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    FILE_STANDARD_INFORMATION information{};
    status = ZwQueryInformationFile(file.value, &io, &information,
        sizeof(information), FileStandardInformation);

    if (!NT_SUCCESS(status)) {
        return status;
    }
    if (information.Directory || information.EndOfFile.QuadPart < minimum ||
        information.EndOfFile.QuadPart > maximum) {
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    DriverBlob candidate;
    candidate.storage_.length_ = static_cast<ULONG>(information.EndOfFile.QuadPart);
    candidate.storage_.data_ = static_cast<uint8_t*>(ExAllocatePoolWithTag(
        NonPagedPool, candidate.storage_.length_, blob_pool_tag));

    if (candidate.storage_.data_ == nullptr) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    LARGE_INTEGER offset{};
    // The candidate owns exactly candidate.storage_.length_ writable bytes. Synchronous kernel file
    // I/O neither retains this buffer nor accepts a user-specified address.
    status = ZwReadFile(file.value, nullptr, nullptr, nullptr, &io, candidate.storage_.data_,
        candidate.storage_.length_, &offset, nullptr);

    if (!NT_SUCCESS(status)) {
        return status;
    }
    if (io.Information != candidate.storage_.length_) {
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    return candidate.detach(storage_) ? STATUS_SUCCESS : STATUS_INVALID_DEVICE_STATE;
}

NTSTATUS DriverBlob::load_firmware(hardware::FirmwareKind kind) noexcept {
    constexpr ULONG minimum_container_bytes = 32;
    return load_known(hardware::firmware_filename(kind), minimum_container_bytes,
                      hardware::maximum_firmware_bytes);
}

NTSTATUS DriverBlob::load_ucode(hardware::UcodeSource source, uint8_t pci_revision,
                                bool fallback) noexcept {
    constexpr ULONG minimum_container_bytes = 32;
    return load_known(hardware::ucode_filename(source, pci_revision, fallback),
                      minimum_container_bytes, hardware::maximum_firmware_bytes);
}

NTSTATUS DriverBlob::load_psp_ta() noexcept {
    constexpr ULONG minimum_container_bytes = 32;
    return load_known(hardware::psp_ta_filename, minimum_container_bytes,
                      hardware::maximum_firmware_bytes);
}

NTSTATUS DriverBlob::load_atom_rom() noexcept {
    constexpr ULONG minimum_atom_bytes = 0x4a;
    return load_known("rx7900.rom", minimum_atom_bytes, maximum_atom_file_bytes);
}

NTSTATUS DriverBlob::load_bytes(ByteView source) noexcept {
    if (storage_.data_ != nullptr) {
        return STATUS_DEVICE_BUSY;
    }
    if (source.size() == 0 || source.size() > maximum_atom_file_bytes) {
        return STATUS_INVALID_PARAMETER;
    }

    DriverBlob candidate;
    candidate.storage_.length_ = static_cast<ULONG>(source.size());
    candidate.storage_.data_ = static_cast<uint8_t*>(ExAllocatePoolWithTag(
        NonPagedPool, candidate.storage_.length_, blob_pool_tag));

    if (candidate.storage_.data_ == nullptr) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    // The candidate owns exactly source.size() writable bytes; checked reads only.
    for (size_t index = 0; index < source.size(); ++index) {
        source.read_u8(index, candidate.storage_.data_[index]);
    }

    return candidate.detach(storage_) ? STATUS_SUCCESS : STATUS_INVALID_DEVICE_STATE;
}

bool DriverBlob::detach(BlobStorage& destination) noexcept {
    if (&destination == &storage_ || destination.data_ != nullptr || storage_.data_ == nullptr) {
        return false;
    }
    destination.data_ = storage_.data_;
    destination.length_ = storage_.length_;
    storage_.data_ = nullptr;
    storage_.length_ = 0;
    return true;
}

void DriverBlob::release(BlobStorage& storage) noexcept {
    if (storage.data_ != nullptr) {
        ExFreePoolWithTag(storage.data_, blob_pool_tag);
    }
    storage.data_ = nullptr;
    storage.length_ = 0;
}

} // namespace rx7900::reactos
