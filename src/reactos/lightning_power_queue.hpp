/*
Module: lightning.reactos.power_queue
Purpose: Bound native pending power IRPs without per-request allocation.
Created: 2026-10-21
Architecture: Caller serializes access; the native owner holds each IRP remove lock.
*/
#pragma once

#include <stddef.h>

namespace lightning::reactos {

// Request is a nullable native request pointer. A successful push transfers its
// queue ownership; pop transfers that ownership to the single PASSIVE consumer.
// Stop rejects new requests while preserving the accepted FIFO for draining.
template<typename Request>
class PowerQueue final {
public:
    static constexpr size_t capacity = 8;

    bool push(Request request) noexcept {
        if (request == Request{} || stopping_ || count_ == capacity) {
            return false;
        }

        requests_[(head_ + count_) % capacity] = request;
        ++count_;
        return true;
    }

    bool pop(Request& request) noexcept {
        request = Request{};

        if (count_ == 0) {
            return false;
        }

        request = requests_[head_];
        requests_[head_] = Request{};
        head_ = (head_ + 1) % capacity;
        --count_;
        return true;
    }

    void stop() noexcept { stopping_ = true; }
    bool stopping() const noexcept { return stopping_; }
    size_t count() const noexcept { return count_; }

private:
    Request requests_[capacity]{};
    size_t head_ = 0;
    size_t count_ = 0;
    bool stopping_ = false;
};

} // namespace lightning::reactos
