/*
Module: rx7900.tools.file_support
Purpose: Reuse scoped file ownership and bounded binary reads in host tools/tests.
Created: 2026-10-21
Architecture: User-mode I/O boundary; binary interpretation remains in core modules.
*/
#pragma once

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

namespace rx7900::tools {

struct FileCloser final {
    void operator()(FILE* file) const noexcept { std::fclose(file); }
};
using File = std::unique_ptr<FILE, FileCloser>;

// Caller owns the bounded writable vector. Read at most its size plus one byte
// to detect oversize input. Shrinking never reallocates; no view escapes here.
inline bool read_bounded_file(const char* path, std::vector<uint8_t>& bytes) {
    File file(std::fopen(path, "rb"));

    if (!file) {
        std::perror(path);
        return false;
    }

    const size_t received = std::fread(bytes.data(), 1, bytes.size(), file.get());
    const int extra = std::fgetc(file.get());

    if (std::ferror(file.get()) != 0 || extra != EOF) {
        std::fprintf(stderr, "Input read failed or exceeds the %llu-byte limit\n",
                     static_cast<unsigned long long>(bytes.size()));
        return false;
    }

    bytes.resize(received);
    return true;
}

} // namespace rx7900::tools
