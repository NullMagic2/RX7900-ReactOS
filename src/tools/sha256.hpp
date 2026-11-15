/*
Module: rx7900.tools.sha256
Purpose: Identify installed and packaged driver files by their SHA-256 digest.
Created: 2026-10-24
Architecture: Allocation-free FIPS 180-4 SHA-256 shared by the Radeon installer
engine, its Win32 binding and host tests. Identity only, not authentication.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900::tools {

constexpr size_t sha256_bytes = 32;
// Lowercase hexadecimal plus the terminating NUL.
constexpr size_t sha256_hex_characters = sha256_bytes * 2 + 1;

struct Sha256Digest final {
    uint8_t bytes[sha256_bytes]{};
};

inline bool operator==(const Sha256Digest& left, const Sha256Digest& right) noexcept {
    for (size_t index = 0; index < sha256_bytes; ++index) {
        if (left.bytes[index] != right.bytes[index]) {
            return false;
        }
    }

    return true;
}

inline bool operator!=(const Sha256Digest& left, const Sha256Digest& right) noexcept {
    return !(left == right);
}

// Streaming hasher. update accepts any number of caller-owned byte ranges; the
// pointer must reference count readable bytes for the call only. finish may be
// called once; later updates are ignored and the object must be reset first.
class Sha256 final {
public:
    Sha256() noexcept { reset(); }

    void reset() noexcept {
        constexpr uint32_t initial[8]{
            0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
            0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};

        for (size_t index = 0; index < 8; ++index) {
            state_[index] = initial[index];
        }

        total_bytes_ = 0;
        pending_ = 0;
        finished_ = false;
    }

    void update(const uint8_t* data, size_t count) noexcept {
        if (finished_ || data == nullptr) {
            return;
        }

        for (size_t index = 0; index < count; ++index) {
            block_[pending_] = data[index];
            ++pending_;

            if (pending_ == sizeof(block_)) {
                compress();
                pending_ = 0;
            }
        }

        total_bytes_ += count;
    }

    Sha256Digest finish() noexcept {
        Sha256Digest digest{};

        if (finished_) {
            return digest;
        }

        const uint64_t bit_count = total_bytes_ * 8u;
        block_[pending_] = 0x80;
        ++pending_;

        if (pending_ > 56) {
            while (pending_ < sizeof(block_)) {
                block_[pending_] = 0;
                ++pending_;
            }
            compress();
            pending_ = 0;
        }
        while (pending_ < 56) {
            block_[pending_] = 0;
            ++pending_;
        }
        for (size_t index = 0; index < 8; ++index) {
            block_[56 + index] = static_cast<uint8_t>(bit_count >> (56 - 8 * index));
        }

        compress();
        finished_ = true;

        for (size_t index = 0; index < 8; ++index) {
            digest.bytes[4 * index] = static_cast<uint8_t>(state_[index] >> 24);
            digest.bytes[4 * index + 1] = static_cast<uint8_t>(state_[index] >> 16);
            digest.bytes[4 * index + 2] = static_cast<uint8_t>(state_[index] >> 8);
            digest.bytes[4 * index + 3] = static_cast<uint8_t>(state_[index]);
        }

        return digest;
    }

private:
    static constexpr uint32_t rotate(uint32_t value, unsigned count) noexcept {
        return (value >> count) | (value << (32 - count));
    }

    void compress() noexcept {
        constexpr uint32_t round_constants[64]{
            0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u,
            0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
            0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u,
            0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
            0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
            0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
            0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
            0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
            0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au,
            0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
            0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};
        uint32_t schedule[64]{};

        for (size_t index = 0; index < 16; ++index) {
            schedule[index] = (static_cast<uint32_t>(block_[4 * index]) << 24) |
                              (static_cast<uint32_t>(block_[4 * index + 1]) << 16) |
                              (static_cast<uint32_t>(block_[4 * index + 2]) << 8) |
                              static_cast<uint32_t>(block_[4 * index + 3]);
        }
        for (size_t index = 16; index < 64; ++index) {
            const uint32_t low = schedule[index - 15];
            const uint32_t high = schedule[index - 2];
            const uint32_t sigma0 = rotate(low, 7) ^ rotate(low, 18) ^ (low >> 3);
            const uint32_t sigma1 = rotate(high, 17) ^ rotate(high, 19) ^ (high >> 10);
            schedule[index] = schedule[index - 16] + sigma0 + schedule[index - 7] + sigma1;
        }

        uint32_t a = state_[0];
        uint32_t b = state_[1];
        uint32_t c = state_[2];
        uint32_t d = state_[3];
        uint32_t e = state_[4];
        uint32_t f = state_[5];
        uint32_t g = state_[6];
        uint32_t h = state_[7];

        for (size_t index = 0; index < 64; ++index) {
            const uint32_t sum1 = rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25);
            const uint32_t choose = (e & f) ^ (~e & g);
            const uint32_t first = h + sum1 + choose + round_constants[index] + schedule[index];
            const uint32_t sum0 = rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22);
            const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t second = sum0 + majority;
            h = g;
            g = f;
            f = e;
            e = d + first;
            d = c;
            c = b;
            b = a;
            a = first + second;
        }

        state_[0] += a;
        state_[1] += b;
        state_[2] += c;
        state_[3] += d;
        state_[4] += e;
        state_[5] += f;
        state_[6] += g;
        state_[7] += h;
    }

    uint32_t state_[8]{};
    uint8_t block_[64]{};
    uint64_t total_bytes_ = 0;
    size_t pending_ = 0;
    bool finished_ = false;
};

// Writes exactly sha256_hex_characters including the NUL terminator.
template<typename Character>
void sha256_hex(const Sha256Digest& digest, Character (&output)[sha256_hex_characters]) noexcept {
    constexpr char digits[] = "0123456789abcdef";

    for (size_t index = 0; index < sha256_bytes; ++index) {
        output[2 * index] = static_cast<Character>(digits[digest.bytes[index] >> 4]);
        output[2 * index + 1] = static_cast<Character>(digits[digest.bytes[index] & 0x0f]);
    }

    output[sha256_hex_characters - 1] = 0;
}

// Accepts exactly 64 hexadecimal digits (either case) followed by a NUL within
// capacity. Output is unchanged on failure.
template<typename Character, size_t Capacity>
bool parse_sha256_hex(const Character (&text)[Capacity], Sha256Digest& output) noexcept {
    if (Capacity < sha256_hex_characters || text[sha256_hex_characters - 1] != 0) {
        return false;
    }

    Sha256Digest parsed{};

    for (size_t index = 0; index < sha256_bytes * 2; ++index) {
        const auto character = static_cast<uint32_t>(text[index]);
        uint8_t nibble = 0;

        if (character >= '0' && character <= '9') {
            nibble = static_cast<uint8_t>(character - '0');
        } else if (character >= 'a' && character <= 'f') {
            nibble = static_cast<uint8_t>(character - 'a' + 10);
        } else if (character >= 'A' && character <= 'F') {
            nibble = static_cast<uint8_t>(character - 'A' + 10);
        } else {
            return false;
        }

        parsed.bytes[index / 2] = static_cast<uint8_t>(parsed.bytes[index / 2] | (index % 2 == 0 ? nibble << 4 : nibble));
    }

    output = parsed;
    return true;
}

} // namespace rx7900::tools
