/*
Module: rx7900.reactos.video_helpers
Purpose: Reuse checked native extension alignment and VideoPort buffer access.
Created: 2026-10-21
Architecture: Native SDK types must be included before this shared header.
*/
#pragma once
#include <stdint.h>

namespace rx7900::reactos {
template<typename T>
constexpr size_t video_extension_bytes=sizeof(T)+alignof(T)-1U;

template<typename T>
T* aligned_video_extension(void* extension) noexcept {
    static_assert(video_extension_bytes<T><=MAXULONG,"VideoPort extension extent");
    const uintptr_t original=reinterpret_cast<uintptr_t>(extension);
    if (extension==nullptr || original>UINTPTR_MAX-(alignof(T)-1U)) return nullptr;
    const uintptr_t aligned=(original+alignof(T)-1U)&~(static_cast<uintptr_t>(alignof(T))-1U);
    if (aligned-original>video_extension_bytes<T>-sizeof(T)) return nullptr;
    return reinterpret_cast<T*>(aligned);
}

inline bool has_video_output(const VIDEO_REQUEST_PACKET& packet, ULONG bytes) noexcept {
    return packet.OutputBuffer!=nullptr && packet.OutputBufferLength>=bytes;
}

template<typename T>
VP_STATUS return_video_value(VIDEO_REQUEST_PACKET& packet, const T& value) noexcept {
    static_assert(sizeof(T)<=MAXULONG,"Native VideoPort output extent");
    if (!has_video_output(packet,sizeof(T))) return ERROR_INSUFFICIENT_BUFFER;
    VideoPortMoveMemory(packet.OutputBuffer,const_cast<T*>(&value),sizeof(T));
    packet.StatusBlock->Information=sizeof(T);
    return NO_ERROR;
}

template<typename T>
bool read_video_value(const VIDEO_REQUEST_PACKET& packet, T& value) noexcept {
    static_assert(sizeof(T)<=MAXULONG,"Native VideoPort input extent");
    if (packet.InputBuffer==nullptr || packet.InputBufferLength<sizeof(T)) return false;
    VideoPortMoveMemory(&value,packet.InputBuffer,sizeof(T));
    return true;
}
} // namespace rx7900::reactos
