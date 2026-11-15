/*
Module: rx7900.reactos.dp_port_native
Purpose: Bind one physical DP connector to an exclusively owned link lease.
Created: 2026-10-22
Architecture: Reuses ATOM topology, DP AUX discovery and SST training policy.
Actual Navi31 AUX/HPD/PHY, DCN clocks and scanout remain a separate backend.
*/
#pragma once

#include "atom_connectors.hpp"
#include "dp_training.hpp"

namespace rx7900::reactos {

enum class DpPortState : uint8_t {
    unbound,
    bound,
    probed,
    trained,
    quarantined
};

enum class DpPortResult : uint8_t {
    success,
    invalid_argument,
    unsupported_route,
    no_lease,
    not_quiescent,
    invalid_state,
    probe_failed,
    training_failed,
    state_unknown
};

// Native callbacks must be serialized at PASSIVE_LEVEL with ASIC reset,
// connector routing, mode changes and removal. They may not trust a caller
// supplied PCI/MMIO address. proof checks current hardware ownership and
// the exact ATOM path/encoder/HPD/AUX route for this epoch. quiescent proves
// no active stream owns the link before a probe. quiesce disables the source
// stream/PHY and confirms it before retiring the lease. After uncertainty,
// only an actual reset proof may release the quarantined object.
struct DpNativeOwner final {
    void* context = nullptr;
    bool (*proof)(void*, uint64_t epoch,
                  const output::Connector&) noexcept = nullptr;
    bool (*quiescent)(void*) noexcept = nullptr;
    bool (*quiesce)(void*) noexcept = nullptr;
    bool (*reset_proven)(void*, uint64_t old_epoch) noexcept = nullptr;
};

// Device-owned, not stack-local in a native miniport. No allocation, register
// access or mode publication happens here. The native owner must keep callbacks
// and connector routing resident until retire/recover completes; destruction
// alone does not quiesce hardware. Copy/move is forbidden: a lease is unique.
class DpNativePort final {
public:
    DpNativePort() noexcept = default;
    DpNativePort(const DpNativePort&) = delete;
    DpNativePort& operator=(const DpNativePort&) = delete;
    DpNativePort(DpNativePort&&) = delete;
    DpNativePort& operator=(DpNativePort&&) = delete;

    DpPortResult bind(const output::ConnectorCatalog& catalog,
                      uint8_t firmware_path_index, uint64_t epoch,
                      const output::DpSourceCapabilities& source,
                      output::AuxIo aux, output::DpPhyIo phy,
                      DpNativeOwner owner) noexcept;
    DpPortResult probe(output::DpProbeScratch& scratch) noexcept;
    DpPortResult train(const output::DpTrainingRequest& request) noexcept;
    DpPortResult retire() noexcept;
    DpPortResult recover_after_reset() noexcept;

    DpPortState state() const noexcept { return state_; }
    output::DpProbeStatus last_probe_status() const noexcept { return last_probe_; }
    output::DpTrainingStatus last_training_status() const noexcept {
        return last_training_;
    }
    // Borrowed while the caller holds the exclusive native lease. No stream
    // is enabled or mode advertised, even when state() == trained.
    const output::DpPortSnapshot* snapshot() const noexcept;

private:
    bool owner_current() const noexcept;
    void clear() noexcept;
    void quarantine() noexcept;

    output::Connector connector_{};
    output::DpSourceCapabilities source_{};
    output::AuxIo aux_{};
    output::DpPhyIo phy_{};
    DpNativeOwner owner_{};
    output::DpPortSnapshot snapshot_{};
    uint64_t epoch_ = 0;
    DpPortState state_ = DpPortState::unbound;
    output::DpProbeStatus last_probe_ = output::DpProbeStatus::invalid_argument;
    output::DpTrainingStatus last_training_ =
        output::DpTrainingStatus::invalid_argument;
};

} // namespace rx7900::reactos
