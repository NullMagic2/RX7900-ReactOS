/*
Module: rx7900.reactos.dp_port_native
Purpose: Fail-closed physical DisplayPort routing and link-lifetime bridge.
Created: 2026-10-22
Architecture: ATOM connector identity is checked before the owned AUX/PHY
callbacks are reused by dp_link/dp_training. No direct hardware writes here.
*/
#include "dp_port_native.hpp"

namespace rx7900::reactos {

bool DpNativePort::owner_current() const noexcept {
    return epoch_ != 0 && owner_.proof != nullptr &&
           owner_.proof(owner_.context, epoch_, connector_);
}

void DpNativePort::clear() noexcept {
    connector_ = {};
    source_ = {};
    aux_ = {};
    phy_ = {};
    owner_ = {};
    snapshot_ = {};
    epoch_ = 0;
    state_ = DpPortState::unbound;
    last_probe_ = output::DpProbeStatus::invalid_argument;
    last_training_ = output::DpTrainingStatus::invalid_argument;
}

void DpNativePort::quarantine() noexcept {
    snapshot_ = {};
    state_ = DpPortState::quarantined;
}

DpPortResult DpNativePort::bind(
    const output::ConnectorCatalog& catalog, uint8_t firmware_path_index,
    uint64_t epoch, const output::DpSourceCapabilities& source,
    output::AuxIo aux, output::DpPhyIo phy, DpNativeOwner owner) noexcept {
    if (state_ != DpPortState::unbound) {
        return DpPortResult::invalid_state;
    }

    if (epoch == 0 || catalog.count > output::connector_capacity ||
        !source.verified || !source.rgb8_verified ||
        aux.transfer == nullptr || aux.wait_microseconds == nullptr ||
        aux.connected == nullptr || phy.configure == nullptr ||
        phy.pattern == nullptr || phy.drive == nullptr || phy.finish == nullptr ||
        owner.proof == nullptr || owner.quiescent == nullptr ||
        owner.quiesce == nullptr || owner.reset_proven == nullptr) {
        return DpPortResult::invalid_argument;
    }

    output::LinkCapabilities budget{};
    if (output::dp_link_budget(source.maximum_rate_code,
                               source.maximum_lanes, true, budget) !=
        output::DpProbeStatus::success) {
        return DpPortResult::invalid_argument;
    }

    const output::Connector* selected = nullptr;
    for (uint32_t index = 0; index < catalog.count; ++index) {
        const output::Connector& candidate = catalog.connectors[index];
        if (candidate.firmware_path_index != firmware_path_index) {
            continue;
        }

        if (selected != nullptr) {
            return DpPortResult::unsupported_route;
        }
        selected = &candidate;
    }

    // eDP needs panel power sequencing; USB-C needs an owned mux/alt-mode
    // contract. They cannot inherit the external SST DisplayPort lease.
    if (selected == nullptr || selected->kind != output::ConnectorKind::display_port ||
        selected->encoder_object_id == 0 || !selected->has_i2c_record ||
        !selected->has_hpd_record) {
        return DpPortResult::unsupported_route;
    }

    if (!owner.proof(owner.context, epoch, *selected)) {
        return DpPortResult::no_lease;
    }

    if (!owner.quiescent(owner.context)) {
        return DpPortResult::not_quiescent;
    }

    connector_ = *selected;
    source_ = source;
    aux_ = aux;
    phy_ = phy;
    owner_ = owner;
    epoch_ = epoch;
    state_ = DpPortState::bound;
    return DpPortResult::success;
}

DpPortResult DpNativePort::probe(output::DpProbeScratch& scratch) noexcept {
    if (state_ != DpPortState::bound && state_ != DpPortState::probed) {
        return DpPortResult::invalid_state;
    }

    if (!owner_current()) {
        quarantine();
        return DpPortResult::state_unknown;
    }

    if (!owner_.quiescent(owner_.context)) {
        quarantine();
        return DpPortResult::state_unknown;
    }

    last_probe_ = output::probe_dp_port(aux_, source_, scratch, snapshot_);

    if (!owner_current()) {
        quarantine();
        return DpPortResult::state_unknown;
    }

    if (last_probe_ != output::DpProbeStatus::success) {
        snapshot_ = {};
        state_ = DpPortState::bound;
        return DpPortResult::probe_failed;
    }

    state_ = DpPortState::probed;
    return DpPortResult::success;
}

DpPortResult DpNativePort::train(
    const output::DpTrainingRequest& request) noexcept {
    if (state_ != DpPortState::probed) {
        return DpPortResult::invalid_state;
    }

    if (!owner_current()) {
        quarantine();
        return DpPortResult::state_unknown;
    }

    if (!owner_.quiescent(owner_.context)) {
        quarantine();
        return DpPortResult::state_unknown;
    }

    last_training_ = output::train_dp_sst(aux_, phy_, source_, snapshot_, request);

    if (!owner_current() ||
        last_training_ == output::DpTrainingStatus::state_unknown) {
        quarantine();
        return DpPortResult::state_unknown;
    }

    if (last_training_ == output::DpTrainingStatus::disconnected) {
        snapshot_ = {};
        state_ = DpPortState::bound;
    }

    if (last_training_ != output::DpTrainingStatus::trained) {
        return DpPortResult::training_failed;
    }

    state_ = DpPortState::trained;
    return DpPortResult::success;
}

DpPortResult DpNativePort::retire() noexcept {
    if (state_ == DpPortState::unbound) {
        return DpPortResult::success;
    }

    if (state_ == DpPortState::quarantined) {
        return DpPortResult::state_unknown;
    }

    if (!owner_current() || !owner_.quiesce(owner_.context)) {
        quarantine();
        return DpPortResult::state_unknown;
    }

    clear();
    return DpPortResult::success;
}

DpPortResult DpNativePort::recover_after_reset() noexcept {
    if (state_ != DpPortState::quarantined) {
        return DpPortResult::invalid_state;
    }

    if (!owner_.reset_proven(owner_.context, epoch_)) {
        return DpPortResult::state_unknown;
    }

    clear();
    return DpPortResult::success;
}

const output::DpPortSnapshot* DpNativePort::snapshot() const noexcept {
    if ((state_ != DpPortState::probed && state_ != DpPortState::trained) ||
        !owner_current() || !owner_.quiescent(owner_.context)) {
        return nullptr;
    }

    return &snapshot_;
}

} // namespace rx7900::reactos
