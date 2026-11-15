/*
Module: rx7900.reactos.lifetime
Purpose: Establish C++ lifetime in I/O-manager-owned extension storage.
Created: 2026-10-21
Architecture: Nonallocating construction only; no kernel C++ runtime dependency.
*/
#pragma once

#include <stddef.h>

namespace rx7900::reactos {
struct DeviceStorage final {
    void* address;
};
} // namespace rx7900::reactos

// A distinct placement tag avoids collision with any SDK standard overload.
inline void* operator new(size_t, rx7900::reactos::DeviceStorage storage) noexcept {
    return storage.address;
}

namespace rx7900::reactos {

// PASSIVE_LEVEL. Caller supplies nonnull, correctly aligned sizeof(T) storage,
// constructs exactly once before publishing it, and retains that storage until
// all users retire. Destruction is trivial; resources still need explicit release.
template<typename T>
T* construct_device_state(void* storage) noexcept {
#if defined(_MSC_VER)
    static_assert(__is_trivially_destructible(T),
#elif defined(__GNUC__)
    static_assert(__has_trivial_destructor(T),
#else
#error Native device storage requires an audited trivial-destructor compiler trait
#endif
                  "Device state needs an explicit resource-release owner");
    static_assert(noexcept(T{}), "Kernel construction must not throw");
    return new (DeviceStorage{storage}) T{};
}

} // namespace rx7900::reactos
