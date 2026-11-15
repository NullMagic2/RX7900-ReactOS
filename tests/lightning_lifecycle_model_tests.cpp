/*
Module: lightning.tests.lifecycle_model
Purpose: Drive full PnP/power IRP sequences through the production lifecycle state machine.
Created: 2026-10-25
Architecture: Host model over src/reactos/lightning_lifecycle.hpp. The DeviceLifecycle
policy and the run_* owner sequences are the production code; Owners here models the
transport, renderer, recognized display, control link and native containment of
lightning_driver.cpp, and FilterModel mirrors how dispatch_pnp/process_device_power use
them (lower results included). Every fallible owner step of every scenario is failed in
turn, and a bounded deterministic random walk feeds legal and illegal IRP orders. The
invariants: containment is sticky and never followed by register access or restart;
owners start and stop in the documented order; every acquired resource is released
exactly once or retained by containment; REMOVE always withdraws the control link and
chooses the matching disposition; an orderly STOP/REMOVE (and a device D1-D3 query) is
vetoed while renderer views are borrowed, so only an unrefusable surprise removal or SET
meets them (lightning-pnp-veto). The production NT dispatch itself is exercised by
tests/lightning_lifecycle_native_tests.cpp. Fixed-capacity storage only.
*/
#include "lightning_lifecycle.hpp"
#include <cstdint>
#include <cstdio>

namespace {
using namespace lightning::reactos;

unsigned checks = 0;
unsigned failures = 0;
const char* scenario_name = "";
unsigned scenario_fail_at = 0;

void expect(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        if (++failures <= 20) {
            std::fprintf(stderr, "FAIL [%s, fail_at %u]: %s\n", scenario_name, scenario_fail_at, description);
        }
    }
}

enum class Call : uint8_t {
    transport_start, transport_resume, transport_suspend, transport_stop, display_discover,
    display_resume, display_quiesce, display_abandon, observe_leases, renderer_start, renderer_stop,
    publish_control, withdraw_control, contain
};

enum class TransportState : uint8_t { none, active, suspended };

constexpr unsigned no_failure = 0xffffffffu;
constexpr unsigned log_capacity = 64;

// What lightning_driver.cpp owns below its lifecycle mutex, reduced to the
// facts the lifecycle depends on. Native semantics mirrored from
// lightning_transport.cpp (stop releases once, suspend retains, an uncertain
// stop/suspend quarantines the transport), lightning_renderer.cpp (live views
// keep shutdown busy) and the miniport's quiesce/resume/abandon requests.
struct Model final {
    DeviceLifecycle lifecycle{};
    TransportState transport = TransportState::none;
    bool transport_quarantined = false;
    unsigned transport_acquired = 0;
    unsigned transport_released = 0;
    bool renderer_active = false;
    bool renderer_quarantined = false;
    bool views = false;
    bool display_attached = true;
    // win32k holds an open of the top-of-stack device.
    bool foreign_open = false;
    bool recognized = false;
    // lightning_driver.cpp display_identity: latched by the first successful
    // discovery; a later failed discovery clears recognized only.
    bool identity = false;
    bool quiesced = true;
    bool lost = false;
    bool link = false;
    unsigned link_created = 0;
    unsigned link_deleted = 0;
    unsigned contain_calls = 0;
    unsigned containments = 0;
    unsigned observed = 0;
    unsigned hardware_after_containment = 0;
    // The lower function is in an owned D0 (started, not powered down,
    // stopped or surprise-removed): only then may an owner touch registers.
    bool lower_d0 = false;
    unsigned hardware_outside_d0 = 0;
    unsigned fallible = 0;
    unsigned fail_at = no_failure;
    bool start_quarantines = false;
    Call log[log_capacity]{};
    unsigned logged = 0;

    void note(Call call) noexcept {
        if (logged < log_capacity) {
            log[logged] = call;
        }

        ++logged;
    }

    bool fail() noexcept { return fallible++ == fail_at; }

    void hardware() noexcept {
        if (lifecycle.contained()) {
            ++hardware_after_containment;
        }

        if (!lower_d0) {
            ++hardware_outside_d0;
        }
    }
};

struct Owners final {
    Model& m;

    bool transport_start() noexcept {
        m.note(Call::transport_start);
        m.hardware();

        if (m.transport != TransportState::none || m.transport_quarantined) {
            return false;
        }

        if (m.fail()) {
            // An unusable adapter is retained by the transport itself.
            m.transport_quarantined = m.start_quarantines;
            return false;
        }

        m.transport = TransportState::active;
        ++m.transport_acquired;
        return true;
    }

    bool transport_resume() noexcept {
        m.note(Call::transport_resume);
        m.hardware();

        if (m.transport != TransportState::suspended || m.transport_quarantined || m.fail()) {
            return false;
        }

        m.transport = TransportState::active;
        return true;
    }

    bool transport_suspend() noexcept {
        m.note(Call::transport_suspend);

        if (m.transport == TransportState::active) {
            m.hardware();
        }

        if (m.transport_quarantined) {
            return false;
        }

        if (m.fail()) {
            m.transport_quarantined = true;
            return false;
        }

        if (m.transport == TransportState::active) {
            m.transport = TransportState::suspended;
        }

        return true;
    }

    bool transport_stop(bool hardware) noexcept {
        m.note(Call::transport_stop);

        if (hardware && m.transport != TransportState::none) {
            m.hardware();
        }

        if (m.transport_quarantined) {
            return false;
        }

        if (m.transport == TransportState::none) {
            return true;
        }

        if (m.fail()) {
            m.transport_quarantined = true;
            return false;
        }

        m.transport = TransportState::none;
        ++m.transport_released;
        return true;
    }

    bool transport_ready() const noexcept { return m.transport == TransportState::active; }
    bool transport_contained() const noexcept { return m.transport_quarantined; }

    bool display_discover() noexcept {
        m.note(Call::display_discover);
        m.hardware();
        m.recognized = false;
        m.lost = false;
        m.quiesced = true;

        if (m.fail()) {
            return false;
        }

        m.recognized = m.display_attached;
        m.identity = m.identity || m.recognized;
        return true;
    }

    bool display_resume() noexcept {
        m.note(Call::display_resume);

        if (m.lost) {
            return false;
        }

        if (!m.recognized) {
            return true;
        }

        m.hardware();

        if (m.fail()) {
            return false;
        }

        m.quiesced = false;
        return true;
    }

    bool display_quiesce() noexcept {
        m.note(Call::display_quiesce);

        if (!m.recognized || m.quiesced) {
            return true;
        }

        m.hardware();

        if (m.fail()) {
            return false;
        }

        m.quiesced = true;
        return true;
    }

    // Software-only containment request; legal while contained.
    bool display_abandon() noexcept {
        m.note(Call::display_abandon);

        if ((!m.recognized && !m.identity) || m.lost) {
            return true;
        }

        if (m.fail()) {
            return false;
        }

        m.lost = true;
        m.quiesced = true;
        return true;
    }

    bool display_recognized() const noexcept { return m.recognized || m.identity; }
    bool display_lost() const noexcept { return m.lost; }

    void observe_leases() noexcept {
        m.note(Call::observe_leases);
        ++m.observed;
    }

    void renderer_start() noexcept {
        m.note(Call::renderer_start);
        m.renderer_active = !m.renderer_quarantined && m.transport == TransportState::active;
    }

    bool renderer_stop(bool hardware) noexcept {
        m.note(Call::renderer_stop);

        if (hardware && m.renderer_active) {
            m.hardware();
        }

        m.renderer_active = false;

        if (m.views || m.fail()) {
            m.renderer_quarantined = true;
        }

        return !m.renderer_quarantined;
    }

    bool publish_control() noexcept {
        m.note(Call::publish_control);

        if (m.fail()) {
            return false;
        }

        m.link = true;
        ++m.link_created;
        return true;
    }

    void withdraw_control() noexcept {
        m.note(Call::withdraw_control);

        if (m.link) {
            m.link = false;
            ++m.link_deleted;
        }
    }

    // lightning_driver.cpp quarantine(): latch, then software-only retries.
    void contain(LifecycleStep) noexcept {
        m.note(Call::contain);
        ++m.contain_calls;

        if (m.lifecycle.contain()) {
            ++m.containments;
        }

        withdraw_control();
        display_abandon();

        if (renderer_stop(false)) {
            transport_stop(false);
        }
    }
};

enum class Event : uint8_t {
    start, start_lower_fails, query_stop, query_remove, query_vetoed_stop, query_vetoed_remove,
    cancel_stop, cancel_remove, stop, surprise, remove, d3, d3_refused, d0, d0_lower_fails,
    shutdown, sleep_query, map_views, unmap_views, external_containment, foreign_open,
    foreign_close, d3_query, count
};

enum class Outcome : uint8_t { ok, failed, refused };

// Mirrors dispatch_pnp, dispatch_power and process_device_power over the
// production policy. Lower results come from the event.
struct FilterModel final {
    Model m{};
    bool removed = false;
    RemoveDisposition disposition = RemoveDisposition::delete_device;

    Outcome deliver(Event event) noexcept {
        Owners owners{m};

        switch (event) {
        case Event::start:
        case Event::start_lower_fails: {
            if (event == Event::start_lower_fails) {
                return Outcome::failed;
            }

            const bool was_started = m.lifecycle.phase() == DevicePhase::started;
            m.lower_d0 = true;
            const StartResult result = run_start(m.lifecycle, owners);

            if (result == StartResult::failed && !m.lifecycle.contained()) {
                expect(m.transport == TransportState::none && !m.renderer_active && !m.link,
                    "a failed START releases everything it acquired");
            }

            expect(result != StartResult::refused || m.lifecycle.contained() || !was_started ||
                m.lifecycle.accepts_work(), "a refused duplicate START keeps the running device");
            return result == StartResult::started ? Outcome::ok :
                result == StartResult::refused ? Outcome::refused : Outcome::failed;
        }

        case Event::query_stop:
        case Event::query_remove:
        case Event::query_vetoed_stop:
        case Event::query_vetoed_remove: {
            const bool stop = event == Event::query_stop || event == Event::query_vetoed_stop;
            const DevicePhase phase = m.lifecycle.phase();
            const bool working = m.lifecycle.accepts_work();
            const unsigned calls = m.logged;
            // dispatch_pnp samples NativeRenderer::borrow_census under the guard.
            const QueryAdmission admission = m.lifecycle.admit_query(stop, m.views ? 1U : 0U);

            if (admission == QueryAdmission::borrowed) {
                expect(m.views && !m.lifecycle.contained(), "only a live borrow of an uncontained device vetoes");
                expect(m.lifecycle.phase() == phase && m.lifecycle.accepts_work() == working && m.logged == calls,
                    "a borrow veto changes no phase, keeps new work and touches no owner");
                return Outcome::refused;
            }

            if (admission == QueryAdmission::refused) {
                return Outcome::refused;
            }

            expect(!m.views, "a query is admitted only without borrowed views");

            if (event == Event::query_vetoed_stop || event == Event::query_vetoed_remove) {
                m.lifecycle.end_query(stop);
                return Outcome::failed;
            }

            return Outcome::ok;
        }

        case Event::cancel_stop:
        case Event::cancel_remove:
            m.lifecycle.end_query(event == Event::cancel_stop);
            return Outcome::ok;

        case Event::stop:
            expect(m.lifecycle.phase() != DevicePhase::stop_pending || m.lifecycle.contained() || !m.views,
                "a STOP after an admitted QUERY_STOP never meets borrowed views");

            if (run_stop(m.lifecycle, owners)) {
                m.lower_d0 = false;
                return Outcome::ok;
            }

            return Outcome::failed;

        case Event::surprise:
            m.lower_d0 = false;
            run_surprise(m.lifecycle, owners);
            return Outcome::ok;

        case Event::remove:
            // An admitted QUERY_REMOVE closed new work with no borrow live,
            // so the orderly REMOVE that follows never meets one.
            expect(m.lifecycle.phase() != DevicePhase::remove_pending || m.lifecycle.contained() || !m.views,
                "a REMOVE after an admitted QUERY_REMOVE never meets borrowed views");
            owners.withdraw_control();
            run_remove_owners(m.lifecycle, owners);
            disposition = m.lifecycle.remove_disposition(m.lost || m.foreign_open);
            removed = true;
            return Outcome::ok;

        case Event::d3:
        case Event::d3_refused:
        case Event::shutdown: {
            const bool system = event == Event::shutdown;
            const uint32_t target = system ? power_state::system_shutdown : power_state::device_d3;

            if (power_route(true, system, target) != PowerRoute::owned) {
                return Outcome::failed;
            }

            const bool pending = m.lifecycle.pending();
            const bool prepared = run_power_down(m.lifecycle, owners);

            if (event == Event::d3_refused) {
                if (prepared) {
                    run_power_up(m.lifecycle, owners);
                }
            } else {
                m.lower_d0 = false;
                expect(m.lifecycle.contained() || m.transport != TransportState::active,
                    "the lower device never enters D3 under an active transport");
            }

            expect(m.lifecycle.pending() == pending, "a power transition keeps a pending query");
            return event == Event::d3_refused ? Outcome::failed : Outcome::ok;
        }

        case Event::d0: {
            const bool pending = m.lifecycle.pending();
            m.lower_d0 = true;
            run_power_up(m.lifecycle, owners);
            expect(m.lifecycle.pending() == pending, "D0 keeps a pending query");
            return Outcome::ok;
        }

        case Event::d0_lower_fails:
            return Outcome::failed;

        case Event::sleep_query:
            return power_route(false, true, power_state::system_hibernate) == PowerRoute::veto ?
                Outcome::refused : Outcome::ok;

        case Event::map_views:
            // A view is mapped only by an exchange, which dispatch_io admits
            // only while the lifecycle accepts new work.
            m.views = m.views || (m.renderer_active && m.lifecycle.accepts_work());
            return m.views ? Outcome::ok : Outcome::refused;

        case Event::d3_query:
            // dispatch_power: a device D3 QUERY is forwarded unless a borrow vetoes it.
            return borrow_vetoes_power_query(false, power_state::device_d3, m.views ? 1U : 0U) ?
                Outcome::refused : Outcome::ok;

        case Event::unmap_views:
            m.views = false;
            return Outcome::ok;

        case Event::foreign_open:
            m.foreign_open = m.foreign_open || m.lifecycle.accepts_work();
            return m.foreign_open ? Outcome::ok : Outcome::refused;

        case Event::foreign_close:
            m.foreign_open = false;
            return Outcome::ok;

        case Event::external_containment:
            if (m.lifecycle.phase() != DevicePhase::added) {
                owners.contain(LifecycleStep{});
            }

            return Outcome::ok;

        case Event::count:
            break;
        }

        return Outcome::failed;
    }
};

// Invariants after every event.
void check_step(const FilterModel& f, bool was_contained) {
    const Model& m = f.m;
    expect(!was_contained || m.lifecycle.contained(), "containment is never cleared");
    expect(m.hardware_after_containment == 0, "no register access once contained");
    expect(m.containments <= 1, "per-image containment accounting runs once per FDO");
    expect(m.transport_released <= m.transport_acquired, "a transport release never repeats");
    expect(m.lifecycle.accepts_work() ==
        (!m.lifecycle.contained() && m.lifecycle.phase() == DevicePhase::started),
        "new work only in an uncontained started phase");
    expect(!m.lifecycle.accepts_work() || m.transport == TransportState::active,
        "accepted work always has an active transport");
    expect(!m.lifecycle.accepts_work() || !m.recognized || !m.quiesced,
        "a started recognized display is not left quiesced");
    expect(!m.lifecycle.accepts_work() || m.link, "a started device publishes its control link");
    expect(!m.link || m.lifecycle.accepts_work() || m.lifecycle.pending() ||
        m.lifecycle.settled() == DevicePhase::suspended,
        "the control link exists only for a started (or pending/suspended) device");
    expect(m.link_deleted <= m.link_created && m.link_created - m.link_deleted <= 1,
        "the control link is created and deleted exactly once each time");
    expect(!m.lifecycle.contained() || !m.link, "containment withdraws the control link");
    expect(m.hardware_outside_d0 == 0, "no register access outside an owned lower D0");
    expect(!m.transport_quarantined || m.lifecycle.contained(), "a quarantined transport always contains the device");
    expect(!m.renderer_quarantined || m.lifecycle.contained(), "a quarantined renderer always contains the device");
}

void check_removed(const FilterModel& f) {
    const Model& m = f.m;
    expect(m.lifecycle.phase() == DevicePhase::removed, "REMOVE ends in the removed phase");
    expect(!m.link && m.link_created == m.link_deleted, "REMOVE withdraws the control link exactly once");
    expect(!m.identity || m.lost || m.lifecycle.contained(),
        "REMOVE abandons an ever-recognized display unless containment retained it");

    if (m.lifecycle.contained()) {
        expect(f.disposition != RemoveDisposition::delete_device, "a contained FDO is never deleted");
        expect(!m.lifecycle.retirement_teardown_allowed(), "a contained channel never tears itself down");
    } else {
        expect(m.transport == TransportState::none && m.transport_acquired == m.transport_released,
            "an uncontained REMOVE released every transport acquisition exactly once");
        expect(!m.renderer_active, "an uncontained REMOVE closed the renderer");
        expect(f.disposition == (m.lost || m.foreign_open ? RemoveDisposition::retirement_channel :
            RemoveDisposition::delete_device), "REMOVE keeps a channel for leases or a foreign open, else deletes");
        expect(!m.identity || m.observed != 0, "a lost display's held leases were observed");
    }
}

unsigned run_events(const Event* events, unsigned count, unsigned fail_at, bool display, bool start_quarantines,
                    Outcome* outcomes = nullptr) {
    FilterModel f;
    f.m.fail_at = fail_at;
    f.m.display_attached = display;
    f.m.start_quarantines = start_quarantines;

    for (unsigned index = 0; index < count && !f.removed; ++index) {
        const bool was_contained = f.m.lifecycle.contained();
        const Outcome outcome = f.deliver(events[index]);

        if (outcomes != nullptr) {
            outcomes[index] = outcome;
        }

        check_step(f, was_contained);
    }

    if (f.removed) {
        check_removed(f);
    }

    return f.m.fallible;
}

// Every fallible owner step of the scenario fails once, in turn.
void exhaust(const char* name, const Event* events, unsigned count) {
    scenario_name = name;

    for (int display = 0; display < 2; ++display) {
        for (int quarantines = 0; quarantines < 2; ++quarantines) {
            scenario_fail_at = no_failure;
            const unsigned steps = run_events(events, count, no_failure, display != 0, quarantines != 0);

            for (unsigned fail_at = 0; fail_at <= steps; ++fail_at) {
                scenario_fail_at = fail_at;
                run_events(events, count, fail_at, display != 0, quarantines != 0);
            }
        }
    }
}

unsigned position(const Model& m, Call call, unsigned from = 0) {
    for (unsigned index = from; index < m.logged && index < log_capacity; ++index) {
        if (m.log[index] == call) {
            return index;
        }
    }

    return log_capacity;
}

// Clean runs: the documented order of each sequence.
void check_ordering() {
    scenario_name = "ordering";
    {
        FilterModel f;
        expect(f.deliver(Event::start) == Outcome::ok, "clean START succeeds");
        const Model& m = f.m;
        expect(position(m, Call::transport_start) < position(m, Call::display_discover) &&
            position(m, Call::display_discover) < position(m, Call::display_resume) &&
            position(m, Call::display_resume) < position(m, Call::renderer_start) &&
            position(m, Call::renderer_start) < position(m, Call::publish_control),
            "START: transport, display discovery, display resume, renderer, control link");
        expect(f.m.lifecycle.accepts_work() && f.m.renderer_active, "START leaves an owned D0");
        const unsigned from = f.m.logged;
        f.deliver(Event::d3);
        expect(position(m, Call::display_quiesce, from) < position(m, Call::renderer_stop, from) &&
            position(m, Call::renderer_stop, from) < position(m, Call::transport_suspend, from),
            "D3: display quiesce, renderer, transport suspend");
        expect(f.m.lifecycle.phase() == DevicePhase::suspended && !f.m.lifecycle.accepts_work(),
            "D3 suspends and closes new work");
        expect(f.m.transport == TransportState::suspended && f.m.transport_released == 0,
            "D3 retains the transport's DMA ownership");
        const unsigned up = f.m.logged;
        f.deliver(Event::d0);
        expect(position(m, Call::transport_resume, up) < position(m, Call::display_resume, up) &&
            position(m, Call::display_resume, up) < position(m, Call::renderer_start, up),
            "D0: transport resume, display resume, renderer");
        expect(f.m.lifecycle.accepts_work(), "D0 resumes an owned D0");
        f.deliver(Event::query_remove);
        expect(!f.m.lifecycle.accepts_work(), "a pending query closes new work");
        const unsigned rm = f.m.logged;
        f.deliver(Event::remove);
        expect(position(m, Call::withdraw_control, rm) < position(m, Call::display_quiesce, rm) &&
            position(m, Call::display_quiesce, rm) < position(m, Call::renderer_stop, rm) &&
            position(m, Call::renderer_stop, rm) < position(m, Call::transport_stop, rm) &&
            position(m, Call::transport_stop, rm) < position(m, Call::display_abandon, rm) &&
            position(m, Call::display_abandon, rm) < position(m, Call::observe_leases, rm),
            "REMOVE: link, quiesce, renderer, transport, abandon, lease observation");
        check_removed(f);
        expect(f.disposition == RemoveDisposition::retirement_channel,
            "a recognized display stays as the retirement channel");
    }
    {
        FilterModel f;
        f.deliver(Event::start);
        const unsigned from = f.m.logged;
        f.deliver(Event::surprise);
        const Model& m = f.m;
        expect(position(m, Call::withdraw_control, from) < position(m, Call::display_abandon, from) &&
            position(m, Call::display_abandon, from) < position(m, Call::renderer_stop, from) &&
            position(m, Call::renderer_stop, from) < position(m, Call::transport_stop, from),
            "SURPRISE: link, abandon, renderer, transport");
        expect(m.hardware_after_containment == 0 && !m.lifecycle.contained(), "SURPRISE is clean and register-free");
        f.deliver(Event::remove);
        check_removed(f);
    }
    {
        FilterModel f;
        f.m.display_attached = false;
        f.deliver(Event::start);
        f.deliver(Event::query_stop);
        f.deliver(Event::stop);
        expect(f.m.transport == TransportState::none && f.m.transport_released == 1, "STOP releases the transport");
        expect(f.deliver(Event::start) == Outcome::ok && f.m.transport_acquired == 2, "START after STOP restarts");
        f.deliver(Event::query_remove);
        f.deliver(Event::remove);
        check_removed(f);
        expect(f.disposition == RemoveDisposition::delete_device, "a standalone transport FDO is deleted");
    }
}

// Containment regressions: before the state machine each of these cleared
// the quarantine phase and let a START reuse retained ownership.
void check_sticky_containment() {
    scenario_name = "sticky";
    const Event stop_restart[] = {Event::start, Event::query_stop, Event::external_containment, Event::stop,
        Event::start};
    Outcome outcomes[5]{};
    run_events(stop_restart, 5, no_failure, true, false, outcomes);
    expect(outcomes[4] == Outcome::refused, "START after a contained STOP is refused");

    const Event vetoed[] = {Event::start, Event::query_remove, Event::external_containment};
    FilterModel f;

    for (const Event event : vetoed) {
        f.deliver(event);
    }

    f.m.lifecycle.end_query(false);
    expect(f.m.lifecycle.contained() && !f.m.lifecycle.accepts_work(), "a veto/cancel after containment keeps it");
    expect(f.deliver(Event::query_remove) == Outcome::refused, "QUERY_REMOVE of a contained device is refused");
    f.deliver(Event::surprise);
    expect(f.m.lifecycle.contained(), "SURPRISE keeps containment");
    f.deliver(Event::remove);
    expect(f.disposition != RemoveDisposition::delete_device, "a contained FDO survives REMOVE");

    FilterModel views;
    views.deliver(Event::start);
    views.deliver(Event::map_views);
    views.deliver(Event::d3);
    expect(views.m.lifecycle.contained(), "D3 with live borrowed views contains");
    views.deliver(Event::unmap_views);
    views.deliver(Event::d0);
    expect(!views.m.lifecycle.accepts_work() && views.m.hardware_after_containment == 0,
        "D0 never resumes a contained device");

    FilterModel refused;
    refused.deliver(Event::start);
    refused.deliver(Event::d3_refused);
    expect(refused.m.lifecycle.accepts_work(), "a refused D3 rolls back to an owned D0");
    refused.deliver(Event::query_stop);
    refused.deliver(Event::d3);
    refused.deliver(Event::stop);
    // A rollback after STOP moved on must not resume a stopped device.
    Owners owners{refused.m};
    run_power_up(refused.m.lifecycle, owners);
    expect(refused.m.lifecycle.phase() == DevicePhase::stopped && refused.m.transport == TransportState::none,
        "a late power-up never resumes a stopped device");

    FilterModel duplicate;
    duplicate.deliver(Event::start);
    const unsigned released = duplicate.m.transport_released;
    expect(duplicate.deliver(Event::start) == Outcome::refused && duplicate.m.lifecycle.accepts_work() &&
        duplicate.m.transport_released == released, "a duplicate START keeps the running device");
}

// lightning-pnp-veto: Device Manager disable (QUERY_REMOVE) or a rebalance
// (QUERY_STOP) while a GL process maps renderer views. Before the veto the
// query was admitted and the REMOVE/STOP contained the device until reboot.
void check_borrow_veto() {
    scenario_name = "borrow veto";
    FilterModel f;
    f.deliver(Event::start);
    expect(f.deliver(Event::map_views) == Outcome::ok && f.m.views, "a started renderer maps views");
    const unsigned calls = f.m.logged;
    expect(f.deliver(Event::query_remove) == Outcome::refused, "QUERY_REMOVE with live views is vetoed");
    expect(f.m.lifecycle.phase() == DevicePhase::started && f.m.lifecycle.accepts_work() &&
        !f.m.lifecycle.contained() && f.m.renderer_active && f.m.link && f.m.logged == calls,
        "the vetoed disable leaves the device started and serving");
    f.deliver(Event::cancel_remove);
    expect(f.m.lifecycle.accepts_work() && f.m.renderer_active, "CANCEL_REMOVE after the veto keeps full service");
    expect(f.deliver(Event::query_stop) == Outcome::refused, "QUERY_STOP with live views is vetoed");
    f.deliver(Event::cancel_stop);
    expect(f.m.lifecycle.accepts_work(), "CANCEL_STOP after the veto keeps full service");
    expect(f.deliver(Event::d3_query) == Outcome::refused, "a device D3 QUERY with live views is vetoed");
    expect(f.deliver(Event::sleep_query) == Outcome::refused, "sleep stays vetoed");
    f.deliver(Event::unmap_views);
    expect(f.deliver(Event::d3_query) == Outcome::ok, "a device D3 QUERY without views is forwarded");
    expect(f.deliver(Event::query_remove) == Outcome::ok, "QUERY_REMOVE after the borrow ended is admitted");
    expect(f.deliver(Event::map_views) == Outcome::refused && !f.m.views,
        "no view can be mapped once the query is admitted");
    f.deliver(Event::remove);
    check_removed(f);
    expect(!f.m.lifecycle.contained() && f.disposition == RemoveDisposition::retirement_channel,
        "the disable after the borrow ended is clean and keeps the lease channel");

    // The policy itself, every phase: a borrow only vetoes, never admits.
    for (int stop = 0; stop < 2; ++stop) {
        DeviceLifecycle lifecycle;
        lifecycle.started();
        expect(lifecycle.admit_query(stop != 0, 2) == QueryAdmission::borrowed &&
            lifecycle.phase() == DevicePhase::started, "borrowed sessions veto without a phase change");
        expect(lifecycle.admit_query(stop != 0, 0) == QueryAdmission::admitted && lifecycle.pending(),
            "no borrow admits");
        expect(lifecycle.admit_query(stop != 0, 1) == QueryAdmission::refused, "a pending query refuses first");
        lifecycle.contain();
        lifecycle.end_query(stop != 0);
        expect(lifecycle.admit_query(stop != 0, 1) == QueryAdmission::refused,
            "containment refuses (reboot-only) before the borrow census");
    }

    using namespace power_state;
    expect(!borrow_vetoes_power_query(false, device_d0, 1), "D0 is never vetoed");
    expect(borrow_vetoes_power_query(false, device_d3, 1) && borrow_vetoes_power_query(false, device_d0 + 1, 1),
        "D1-D3 queries are vetoed by a borrow");
    expect(!borrow_vetoes_power_query(false, device_d3, 0), "no borrow, no veto");
    expect(!borrow_vetoes_power_query(true, system_shutdown, 1), "the shutdown query is never vetoed by a borrow");

    // Unrefusable removals still contain.
    FilterModel surprise;
    surprise.deliver(Event::start);
    surprise.deliver(Event::map_views);
    surprise.deliver(Event::surprise);
    expect(surprise.m.lifecycle.contained(), "SURPRISE with live views contains (cannot be vetoed)");
    surprise.deliver(Event::remove);
    check_removed(surprise);

    FilterModel unqueried;
    unqueried.deliver(Event::start);
    unqueried.deliver(Event::map_views);
    unqueried.deliver(Event::remove);
    expect(unqueried.m.lifecycle.contained() && unqueried.disposition != RemoveDisposition::delete_device,
        "an unqueried REMOVE with live views still contains");
}

void check_power_routes() {
    scenario_name = "routes";
    using namespace power_state;
    expect(power_route(true, false, device_d0) == PowerRoute::owned, "device D0 SET is owned");
    expect(power_route(true, false, device_d3) == PowerRoute::owned, "device D3 SET is owned");
    expect(power_route(true, false, 0) == PowerRoute::invalid, "unspecified device SET is malformed");
    expect(power_route(true, false, device_d3 + 1) == PowerRoute::invalid, "beyond-D3 device SET is malformed");
    expect(power_route(false, false, device_d3) == PowerRoute::forward, "device QUERY is forwarded");
    expect(power_route(true, true, system_working) == PowerRoute::forward, "S0 SET is forwarded");
    expect(power_route(false, true, system_working) == PowerRoute::forward, "S0 QUERY is forwarded");
    expect(power_route(true, true, system_shutdown) == PowerRoute::owned, "shutdown SET is the final power-down");
    expect(power_route(false, true, system_shutdown) == PowerRoute::forward, "shutdown QUERY is admitted");

    for (uint32_t state = system_working + 1; state < system_shutdown; ++state) {
        expect(power_route(false, true, state) == PowerRoute::veto, "sleep/hibernate QUERY is vetoed");
        expect(power_route(true, true, state) == PowerRoute::forward, "unqueried sleep SET is forwarded");
    }

    FilterModel f;
    f.deliver(Event::start);
    f.deliver(Event::shutdown);
    expect(f.m.lifecycle.phase() == DevicePhase::suspended && f.m.transport == TransportState::suspended &&
        f.m.recognized && f.m.quiesced && !f.m.renderer_active,
        "shutdown quiesces display, renderer and transport and retains DMA until power-off");
}

uint32_t random_state = 0x2545f491u;

uint32_t next_random() {
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

// Bounded random walk over every event in any order (including orders the
// PnP manager never sends), with one random failure per walk.
void random_walks() {
    scenario_name = "random";
    constexpr unsigned walks = 20000;
    constexpr unsigned length = 14;

    for (unsigned walk = 0; walk < walks; ++walk) {
        Event events[length + 1]{};

        for (unsigned index = 0; index < length; ++index) {
            events[index] = static_cast<Event>(next_random() % static_cast<uint32_t>(Event::count));
        }

        events[length] = Event::remove;
        scenario_fail_at = next_random() % 24;
        run_events(events, length + 1, scenario_fail_at, (walk & 1) != 0, (walk & 2) != 0);
    }
}
} // namespace

int main() {
    check_ordering();
    check_sticky_containment();
    check_power_routes();
    check_borrow_veto();

    const Event lifecycle[] = {Event::start, Event::d3, Event::d0, Event::query_stop, Event::stop, Event::start,
        Event::query_remove, Event::cancel_remove, Event::d3_refused, Event::query_remove, Event::remove};
    exhaust("full lifecycle", lifecycle, sizeof(lifecycle) / sizeof(lifecycle[0]));
    const Event surprise[] = {Event::start, Event::map_views, Event::surprise, Event::remove};
    exhaust("surprise with views", surprise, sizeof(surprise) / sizeof(surprise[0]));
    const Event failed_start[] = {Event::start, Event::remove};
    exhaust("failed start", failed_start, sizeof(failed_start) / sizeof(failed_start[0]));
    const Event suspended_remove[] = {Event::start, Event::d3, Event::query_remove, Event::remove};
    exhaust("remove while suspended", suspended_remove, sizeof(suspended_remove) / sizeof(suspended_remove[0]));
    const Event pending_power[] = {Event::start, Event::query_stop, Event::d3, Event::cancel_stop, Event::d0,
        Event::query_stop, Event::d3, Event::d0, Event::stop, Event::start, Event::shutdown, Event::remove};
    exhaust("power during pending queries", pending_power, sizeof(pending_power) / sizeof(pending_power[0]));
    const Event contained_restart[] = {Event::start, Event::external_containment, Event::query_stop, Event::stop,
        Event::start, Event::d0, Event::surprise, Event::remove};
    exhaust("contained restart", contained_restart, sizeof(contained_restart) / sizeof(contained_restart[0]));
    const Event vetoes[] = {Event::start, Event::query_vetoed_stop, Event::query_vetoed_remove, Event::sleep_query,
        Event::d0_lower_fails, Event::start_lower_fails, Event::query_remove, Event::remove};
    exhaust("lower vetoes", vetoes, sizeof(vetoes) / sizeof(vetoes[0]));
    // A restart whose discovery fails must still abandon the latched display
    // and keep the channel for win32k's open (native use-after-free found by
    // tests/lightning_lifecycle_native_tests.cpp).
    const Event failed_restart[] = {Event::start, Event::foreign_open, Event::query_stop, Event::stop,
        Event::start, Event::query_remove, Event::remove};
    exhaust("failed restart with an open display", failed_restart, sizeof(failed_restart) / sizeof(failed_restart[0]));
    const Event vetoed_disable[] = {Event::start, Event::foreign_open, Event::map_views, Event::query_remove,
        Event::cancel_remove, Event::query_stop, Event::cancel_stop, Event::d3_query, Event::unmap_views,
        Event::query_remove, Event::remove};
    exhaust("disable vetoed by borrowed views", vetoed_disable, sizeof(vetoed_disable) / sizeof(vetoed_disable[0]));
    random_walks();

    std::printf("Lightning lifecycle model: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
