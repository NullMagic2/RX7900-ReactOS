/*
Module: rx7900.tools.inspect
Purpose: Inspect PCI/ROM/EDID files or Linux PCI headers read-only.
Created: 2026-10-21
Architecture: File ownership stays here; all binary interpretation is common code.
*/
#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#ifndef _WIN32
#include <dirent.h>
#endif

#include "atom_bios.hpp"
#include "pci.hpp"
#include "report.hpp"
#include "sink.hpp"
#include "file_support.hpp"
#include "firmware.hpp"

namespace {

constexpr size_t maximum_rom_bytes = 2 * 1024 * 1024;
constexpr size_t path_capacity = 512;

using rx7900::tools::File;
using rx7900::tools::read_bounded_file;

int inspect_pci(const char* path, rx7900::PciLocation location, bool quiet_mismatch) {
    File file(std::fopen(path, "rb"));

    if (!file) {
        if (!quiet_mismatch) {
            std::perror(path);
        }

        return 2;
    }

    std::array<uint8_t, rx7900::pci_header_bytes> bytes{};
    // fread receives exactly the writable capacity of this owned array.
    const size_t received = std::fread(bytes.data(), 1, bytes.size(), file.get());
    rx7900::Adapter adapter{};
    const auto status = rx7900::decode_adapter(
        rx7900::ByteView(bytes.data(), received), location, adapter);

    if (status != rx7900::ParseStatus::success) {
        if (!quiet_mismatch || status != rx7900::ParseStatus::unsupported_device) {
            std::fprintf(stderr, "%s: PCI decode error %u\n", path,
                         static_cast<unsigned>(status));
        }

        return status == rx7900::ParseStatus::unsupported_device ? 3 : 2;
    }

    rx7900::print_adapter(adapter);
    return 0;
}

int inspect_rom(const char* path) {
    // Fixed upper bound avoids trusting an on-disk length before allocation.
    std::vector<uint8_t> bytes(maximum_rom_bytes);

    if (!read_bounded_file(path, bytes)) {
        return 2;
    }

    rx7900::AtomHeader header{};
    const auto status = rx7900::validate_atom_bios(
        rx7900::ByteView(bytes.data(), bytes.size()), header);

    if (status != rx7900::ParseStatus::success) {
        std::fprintf(stderr, "ATOM signature validation error %u\n",
                     static_cast<unsigned>(status));
        return 2;
    }

    std::printf("ATOM signature recognized: header=0x%04x reversed=%u bytes=%llu\n",
        static_cast<unsigned>(header.offset), header.reversed_signature ? 1u : 0u,
        static_cast<unsigned long long>(bytes.size()));
    std::puts("Signature recognition does not validate tables or authorize execution.");
    return 0;
}

int inspect_firmware(const char* kind_name, const char* path) {
    using rx7900::hardware::FirmwareKind;
    FirmwareKind kind{};

    if (std::strcmp(kind_name, "sdma6") == 0) {
        kind = FirmwareKind::sdma_6_0;
    } else if (std::strcmp(kind_name, "psp13") == 0) {
        kind = FirmwareKind::psp_13_0;
    } else if (std::strcmp(kind_name, "dmcub32") == 0) {
        kind = FirmwareKind::dmcub_3_2;
    } else {
        std::fputs("Firmware kind must be sdma6, psp13 or dmcub32\n", stderr);
        return 2;
    }

    std::vector<uint8_t> bytes(rx7900::hardware::maximum_firmware_bytes);

    if (!read_bounded_file(path, bytes)) {
        return 2;
    }

    rx7900::hardware::FirmwareImage image{};
    const auto status = rx7900::hardware::validate_firmware(
        rx7900::ByteView(bytes.data(), bytes.size()), kind, image);

    if (status != rx7900::hardware::FirmwareStatus::success) {
        std::fprintf(stderr, "Firmware container validation error %u\n",
                     static_cast<unsigned>(status));
        return 2;
    }

    std::printf("Firmware %s version=0x%08x parts=%u payload=%u bytes\n",
        rx7900::hardware::firmware_filename(kind), static_cast<unsigned>(image.version),
        static_cast<unsigned>(image.part_count), static_cast<unsigned>(image.payload_bytes));
    std::puts("Container bounds validated; PSP authentication and GPU upload not performed.");
    return 0;
}

int inspect_edid(const char* path) {
    std::vector<uint8_t> bytes(rx7900::output::maximum_edid_blocks *
                               rx7900::output::edid_block_bytes);

    if (!read_bounded_file(path, bytes)) {
        return 2;
    }

    rx7900::output::SinkSnapshot sink{};
    const auto status = rx7900::output::parse_edid(
        rx7900::ByteView(bytes.data(), bytes.size()), sink);

    if (status != rx7900::output::Status::success) {
        std::fprintf(stderr, "EDID validation error %u\n", static_cast<unsigned>(status));
        return 2;
    }

    std::printf("EDID blocks=%u detailed candidates=%u rejected=%u unsupported extensions=%u\n",
        static_cast<unsigned>(sink.block_count),
        static_cast<unsigned>(sink.detailed_modes.count()),
        static_cast<unsigned>(sink.rejected_detailed_modes),
        static_cast<unsigned>(sink.unsupported_extensions));

    for (uint32_t index = 0; index < sink.detailed_modes.count(); ++index) {
        rx7900::display::Timing timing{};
        uint32_t millihz = 0;

        if (sink.detailed_modes.get(index, timing) != rx7900::display::Status::success ||
            rx7900::display::refresh_millihz(timing, millihz) !=
                rx7900::display::Status::success) {
            std::fputs("EDID candidate timing validation failed\n", stderr);
            return 2;
        }

        std::printf("%u: %ux%u %u.%03u Hz pixel clock=%u kHz%s\n",
            static_cast<unsigned>(index), static_cast<unsigned>(timing.width),
            static_cast<unsigned>(timing.height), static_cast<unsigned>(millihz / 1000),
            static_cast<unsigned>(millihz % 1000),
            static_cast<unsigned>(timing.pixel_clock_khz),
            index == sink.preferred_mode ? " preferred" : "");
    }

    std::puts("Sink candidates only; link verification and hardware output are unavailable.");
    return 0;
}

#ifndef _WIN32
struct DirectoryCloser final {
    void operator()(DIR* directory) const noexcept { closedir(directory); }
};

int inspect_live() {
    std::unique_ptr<DIR, DirectoryCloser> directory(opendir("/sys/bus/pci/devices"));

    if (!directory) {
        std::perror("PCI sysfs");
        return 2;
    }

    unsigned matched = 0;
    unsigned unreadable = 0;

    while (const dirent* entry = readdir(directory.get())) {
        unsigned domain = 0;
        unsigned bus = 0;
        unsigned slot = 0;
        unsigned function = 0;
        char trailing = '\0';

        if (std::sscanf(entry->d_name, "%4x:%2x:%2x.%1x%c", &domain, &bus,
                        &slot, &function, &trailing) != 4 ||
            bus > UINT8_MAX || slot >= 32 || function >= 8) {
            continue;
        }

        std::array<char, path_capacity> path{};
        const int length = std::snprintf(path.data(), path.size(),
            "/sys/bus/pci/devices/%s/config", entry->d_name);

        if (length < 0 || static_cast<size_t>(length) >= path.size()) {
            ++unreadable;
            continue;
        }

        const rx7900::PciLocation location{domain, static_cast<uint8_t>(bus),
            static_cast<uint8_t>(slot), static_cast<uint8_t>(function)};
        const int result = inspect_pci(path.data(), location, true);

        if (result == 0) {
            ++matched;
        } else if (result == 2) {
            ++unreadable;
        }
    }

    std::printf("Matching Navi 31 functions: %u; unreadable headers: %u\n",
                matched, unreadable);
    std::puts("Read-only PCI inspection; no ROM, BAR mapping, binding or reset performed.");
    return unreadable != 0 ? 2 : (matched != 0 ? 0 : 3);
}
#endif

} // namespace

int main(int argc, char** argv) {
#ifndef _WIN32
    if (argc == 2 && std::strcmp(argv[1], "--live") == 0) {
        return inspect_live();
    }
#endif

    if (argc == 3 && std::strcmp(argv[1], "--pci") == 0) {
        return inspect_pci(argv[2], {}, false);
    }
    if (argc == 3 && std::strcmp(argv[1], "--rom") == 0) {
        return inspect_rom(argv[2]);
    }
    if (argc == 3 && std::strcmp(argv[1], "--edid") == 0) {
        return inspect_edid(argv[2]);
    }
    if (argc == 4 && std::strcmp(argv[1], "--firmware") == 0) {
        return inspect_firmware(argv[2], argv[3]);
    }

    std::fprintf(stderr,
        "Usage: rx7900inspect --pci FILE | --rom FILE | --edid FILE | --live (Linux)\n"
        "       rx7900inspect --firmware sdma6|psp13|dmcub32 FILE\n");
    return 2;
}
