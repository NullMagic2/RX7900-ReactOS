/*
Module: rx7900.tests.acpi_hotplug_model
Purpose: Model QEMU PIIX4 ACPI PCI hot-unplug against reactos-acpi-hotplug.patch.
Created: 2026-10-24
Architecture: Host-only. Includes the patch's production decision header
(sdk/include/reactos/drivers/acpi/pcihp.h: Rx7900AcpiHpBegin/Advance, Rx7900PciHpTrackPnp,
Rx7900PciHpFunctionGone); build with -I <patched ReactOS>/sdk/include/reactos. The step loop
mirrors AcpiHotplugProcess (drivers/bus/acpi/hotplug.c); QEMU follows hw/acpi/pcihp.c
(up/down bitmaps, PCNT -> DVNT(PCIU,1)/DVNT(PCID,3), PCEJ -> B0EJ -> acpi_pcihp_eject_slot);
the kernel follows the pinned IoRequestDeviceEject and PnP enumeration in
ntoskrnl/io/pnpmgr/devaction.c (QUERY_REMOVE only for the target; vanished children get
SURPRISE_REMOVAL then REMOVE_DEVICE).
*/
#include <drivers/acpi/pcihp.h>

#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

namespace {

unsigned checks = 0;
unsigned failures = 0;

void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL %s\n", message);
    }
}

// --- QEMU hw/acpi/pcihp.c, bsel 0 (i440fx root bus), bridge support on (PCIU clears on read)
struct QemuSlot {
    bool populated = false;
    bool hotpluggable = true;
    std::uint16_t vendor = 0, device = 0;
};

struct Qemu {
    QemuSlot slots[32];
    std::uint32_t up = 0, down = 0;
    bool gpe_sts = false;
    bool gpe_enabled = false;   // only after AcpiUpdateAllGpes
    std::vector<std::string> deleted;   // QMP DEVICE_DELETED events

    void device_add(int slot, std::uint16_t v, std::uint16_t d) {
        slots[slot].populated = true;
        slots[slot].vendor = v;
        slots[slot].device = d;
        up |= 1u << slot;
        gpe_sts = true;
    }
    void device_del(int slot) {        // acpi_pcihp_device_unplug_request_cb
        down |= 1u << slot;
        gpe_sts = true;
    }
    bool sci() const { return gpe_sts && gpe_enabled; }
    // \_GPE._E01 -> \_SB.PCI0.PCNT: (object devfn, notify) in DVNT order
    std::vector<std::pair<int, unsigned>> e01() {
        std::vector<std::pair<int, unsigned>> out;
        gpe_sts = false;
        std::uint32_t pciu = up;
        up = 0;
        for (int s = 0; s < 32; ++s)
            if (pciu & (1u << s)) out.push_back({s << 3, RX7900_ACPI_NOTIFY_DEVICE_CHECK});
        for (int s = 0; s < 32; ++s)
            if (down & (1u << s)) out.push_back({s << 3, RX7900_ACPI_NOTIFY_EJECT_REQUEST});
        return out;
    }
    // Sxx._EJ0 -> PCEJ(BSEL, _SUN): BNUM = 0, B0EJ = 1 << _SUN
    void b0ej(std::uint32_t value) {
        int slot = __builtin_ctz(value);
        down &= ~(1u << slot);
        up &= ~(1u << slot);
        if (slots[slot].populated && slots[slot].hotpluggable) {
            slots[slot].populated = false;
            deleted.push_back("slot" + std::to_string(slot));
        }
    }
    // HalGetBusDataByOffset(bus 0, slot, 0, 4)
    unsigned long read_ids(int slot, std::uint16_t* v, std::uint16_t* d) const {
        if (!slots[slot].populated) { *v = 0xFFFF; *d = 0xFFFF; return 2; }
        *v = slots[slot].vendor; *d = slots[slot].device;
        return 4;
    }
};

// --- guest: pci.sys list + PDO tracking, function driver, pinned PnP manager
struct Driver {
    bool veto_query = false;
    bool has_children = false;
    std::vector<std::string> irps;   // IRPs the function driver saw
};

struct Pdo {
    int slot = 0;
    std::uint16_t vendor = 0, device = 0;
    bool listed = true;              // in pci.sys DeviceListHead
    bool devnode = true;             // DO_BUS_ENUMERATED_DEVICE
    bool devnode_present = true;     // PnP still has it in the tree
    unsigned long remove_state = RX7900_PCIHP_STATE_ACTIVE;
    unsigned long remove_events = 0;
    Driver driver;
    int child_removes = 0;           // REMOVE_DEVICE sent to its child devnodes
};

struct Guest {
    Qemu* qemu = nullptr;
    std::vector<Pdo> pdos;
    bool pci_registered = true;
    bool ost_defined = false;        // QEMU defines no _OST on PCI slots
    int async_after_polls = -1;      // >= 0: a kernel that delivers QUERY_REMOVE later
    int pending_async = -1;
    std::vector<std::string> trace;  // ACPI-visible steps
    std::vector<unsigned long> ost;

    Pdo* find(int slot) {
        for (auto& p : pdos)
            if (p.listed && p.slot == slot) return &p;
        return nullptr;
    }
    void pdo_irp(Pdo& p, unsigned char minor, const char* name) {
        p.driver.irps.push_back(name);
        Rx7900PciHpTrackPnp(minor, &p.remove_state, &p.remove_events);
    }
    // QUERY_REMOVE through the stack: a vetoing driver completes it without passing it
    // down, then the PnP manager sends CANCEL_REMOVE, which every driver passes down.
    void query_remove(Pdo& p) {
        if (p.driver.veto_query) {
            p.driver.irps.push_back("QUERY_REMOVE(veto)");
            pdo_irp(p, RX7900_PCIHP_MN_CANCEL_REMOVE, "CANCEL_REMOVE");
        } else {
            pdo_irp(p, RX7900_PCIHP_MN_QUERY_REMOVE_DEVICE, "QUERY_REMOVE");
        }
    }
    // devaction.c IoRequestDeviceEject: children twice, target QUERY_REMOVE, no REMOVE
    void io_request_device_eject(Pdo& p) {
        trace.push_back("IoRequestDeviceEject");
        if (p.driver.has_children) p.child_removes += 2;
        if (async_after_polls >= 0) { pending_async = async_after_polls; return; }
        query_remove(p);
    }
    void poll_tick() {
        if (pending_async > 0) --pending_async;
        else if (pending_async == 0) { pending_async = -1; for (auto& p : pdos) if (p.listed) { query_remove(p); break; } }
    }
    // IoInvalidateDeviceRelations(PCI0) -> pci.sys FdoEnumerateDevices (+ prune) -> PiEnumerateDevice
    void rescan() {
        trace.push_back("RESCAN");
        for (auto& p : pdos) {
            if (!p.listed) continue;
            std::uint16_t v, d;
            unsigned long n = qemu->read_ids(p.slot, &v, &d);
            if (Rx7900PciHpFunctionGone(n, v, d, p.vendor, p.device)) p.listed = false;
        }
        for (int s = 0; s < 32; ++s) {
            if (!qemu->slots[s].populated || find(s)) continue;
            Pdo p;
            p.slot = s;
            p.vendor = qemu->slots[s].vendor;
            p.device = qemu->slots[s].device;
            pdos.push_back(p);
            pdo_irp(pdos.back(), RX7900_PCIHP_MN_START_DEVICE, "START");
        }
        for (auto& p : pdos) {
            if (p.listed || !p.devnode_present) continue;
            p.devnode_present = false;               // DNF_DEVICE_GONE -> IopRemoveDevice(Force)
            if (p.driver.has_children) p.child_removes += 1;
            pdo_irp(p, RX7900_PCIHP_MN_SURPRISE_REMOVAL, "SURPRISE_REMOVAL");
            pdo_irp(p, RX7900_PCIHP_MN_REMOVE_DEVICE, "REMOVE");
        }
    }
    bool may_have_children(const Pdo& p) const { return p.driver.has_children; }

    // Mirror of AcpiHotplugProcess for a notify on object devfn (slot = devfn >> 3)
    unsigned long notify(int devfn, unsigned long value, unsigned long role) {
        int slot = devfn >> 3;
        RX7900_ACPI_HP_MACHINE m;
        Pdo* pdo = nullptr;
        unsigned guard = 0;
        unsigned long step = Rx7900AcpiHpBegin(&m, value, role);
        while (step != RX7900_ACPI_HP_STEP_DONE && ++guard < 200) {
            switch (step) {
            case RX7900_ACPI_HP_STEP_OST:
                if (ost_defined) { ost.push_back(m.OstStatus); trace.push_back("OST"); }
                break;
            case RX7900_ACPI_HP_STEP_FIND_PDO:
                trace.push_back("FIND");
                pdo = pci_registered ? find(slot) : nullptr;
                m.Answered = pci_registered;
                m.HavePdo = pdo && pdo->devnode;
                m.State = pdo ? pdo->remove_state : 0;
                m.Events = pdo ? pdo->remove_events : 0;
                break;
            case RX7900_ACPI_HP_STEP_CHECK_CHILDREN:
                m.MayHaveChildren = may_have_children(*pdo);
                break;
            case RX7900_ACPI_HP_STEP_REQUEST_EJECT:
                io_request_device_eject(*pdo);
                break;
            case RX7900_ACPI_HP_STEP_WAIT_REMOVAL: {
                if (m.Polls) poll_tick();
                Pdo* now = find(slot);
                m.PdoGone = now != pdo;
                m.StateNow = now ? now->remove_state : 0;
                m.EventsNow = now ? now->remove_events : 0;
                break;
            }
            case RX7900_ACPI_HP_STEP_EVALUATE_EJ0:
                trace.push_back("EJ0");
                qemu->b0ej(1u << slot);
                m.Ej0Ok = 1;
                break;
            case RX7900_ACPI_HP_STEP_RESCAN_BUS:
                rescan();
                break;
            default:
                check(false, "unknown step");
            }
            step = Rx7900AcpiHpAdvance(&m);
        }
        check(guard < 200, "machine terminates");
        return m.Result;
    }
    // SCI -> _E01 -> Notify handler (eject requests already being handled are coalesced)
    std::vector<unsigned long> gpe() {
        std::vector<unsigned long> results;
        while (qemu->sci())
            for (auto& n : qemu->e01())
                results.push_back(notify(n.first, n.second, RX7900_ACPI_HP_ROLE_ROOT_SLOT));
        return results;
    }
};

bool same(const std::vector<std::string>& a, std::initializer_list<const char*> b) {
    if (a.size() != b.size()) return false;
    size_t i = 0;
    for (const char* s : b) if (a[i++] != s) return false;
    return true;
}

Guest boot(Qemu& q, int slot, bool veto = false, bool children = false) {
    q.slots[slot] = QemuSlot{true, true, 0x1af4, 0x1000};
    Guest g;
    g.qemu = &q;
    g.rescan();
    g.trace.clear();
    q.gpe_enabled = true;
    Pdo* p = g.find(slot);
    p->driver.veto_query = veto;
    p->driver.has_children = children;
    p->driver.irps.clear();
    return g;
}

void test_stock_gpe_masked() {
    Qemu q;
    Guest g = boot(q, 5);
    q.gpe_enabled = false;                     // stock ReactOS: no AcpiUpdateAllGpes
    q.device_del(5);
    check(g.gpe().empty(), "stock: masked GPE 1 never reaches \\_GPE._E01");
    check(q.down == (1u << 5) && q.slots[5].populated, "stock: device_del stays pending");
}

void test_unplug_accepted() {
    Qemu q;
    Guest g = boot(q, 5);
    q.device_del(5);
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_EJECTED, "accepted: one eject request, ejected");
    check(same(g.trace, {"FIND", "IoRequestDeviceEject", "EJ0", "RESCAN"}),
          "accepted: request eject, then _EJ0, then re-enumerate");
    check(same(g.pdos[0].driver.irps, {"QUERY_REMOVE", "SURPRISE_REMOVAL", "REMOVE"}),
          "accepted: driver sees QUERY_REMOVE, SURPRISE_REMOVAL, REMOVE");
    check(q.deleted.size() == 1 && q.down == 0 && !q.slots[5].populated, "accepted: QEMU unplugged slot 5");
    check(!g.pdos[0].listed && !g.pdos[0].devnode_present, "accepted: pci.sys dropped it, PnP removed it");
    check(g.gpe().empty(), "accepted: no further eject request");
}

void test_unplug_vetoed_then_retried() {
    Qemu q;
    Guest g = boot(q, 3, true);
    q.device_del(3);
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_VETOED, "veto: result vetoed");
    check(same(g.trace, {"FIND", "IoRequestDeviceEject"}), "veto: no _EJ0, no re-enumeration");
    check(same(g.pdos[0].driver.irps, {"QUERY_REMOVE(veto)", "CANCEL_REMOVE"}), "veto: query vetoed, cancelled");
    check(q.slots[3].populated && q.down == (1u << 3) && q.deleted.empty(), "veto: device stays, PCID bit stays");
    // device_del again (QEMU allows a retry once pending_deleted_expires_ms passed)
    g.pdos[0].driver.veto_query = false;
    g.trace.clear();
    q.device_del(3);
    r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_EJECTED, "veto: retry ejects");
    check(q.deleted.size() == 1, "veto: retry completes device_del");
}

void test_unplug_with_children() {
    Qemu q;
    Guest g = boot(q, 4, false, true);
    q.device_del(4);
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_EJECTED, "children: ejected");
    check(same(g.trace, {"FIND", "EJ0", "RESCAN"}), "children: IoRequestDeviceEject not used");
    check(g.pdos[0].child_removes == 1, "children: each child removed once");
    check(same(g.pdos[0].driver.irps, {"SURPRISE_REMOVAL", "REMOVE"}), "children: surprise removal path");
}

void test_ost_where_defined() {
    Qemu q;
    Guest g = boot(q, 6);
    g.ost_defined = true;
    q.device_del(6);
    g.gpe();
    check(g.ost.size() == 2 && g.ost[0] == RX7900_ACPI_OST_EJECT_IN_PROGRESS && g.ost[1] == RX7900_ACPI_OST_SUCCESS,
          "_OST: 0x84 then success");
    check(same(g.trace, {"OST", "FIND", "IoRequestDeviceEject", "EJ0", "OST", "RESCAN"}),
          "_OST: success reported after _EJ0, before re-enumeration");

    Qemu q2;
    Guest v = boot(q2, 6, true);
    v.ost_defined = true;
    q2.device_del(6);
    v.gpe();
    check(v.ost.size() == 2 && v.ost[1] == RX7900_ACPI_OST_EJECT_DEVICE_IN_USE, "_OST: veto reports 0x81");
}

void test_empty_and_unenumerated_slots() {
    Qemu q;
    Guest g;
    g.qemu = &q;
    q.gpe_enabled = true;
    q.device_del(7);                            // nothing in slot 7
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_EJECTED && same(g.trace, {"FIND", "EJ0", "RESCAN"}),
          "empty slot: _EJ0 directly");

    Qemu q2;
    Guest h = boot(q2, 2);
    h.pdos[0].devnode = false;                  // PDO without a devnode: no driver ever loaded
    h.pdos[0].devnode_present = false;
    q2.device_del(2);
    r = h.gpe();
    check(same(h.trace, {"FIND", "EJ0", "RESCAN"}) && h.pdos[0].driver.irps.empty(),
          "no devnode: _EJ0 directly, no IRPs to a driver");
}

void test_pci_not_listening() {
    Qemu q;
    Guest g = boot(q, 5);
    g.pci_registered = false;
    g.ost_defined = true;
    q.device_del(5);
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_NO_INTERFACE, "no pci.sys answer: refused");
    check(q.slots[5].populated && g.ost.back() == RX7900_ACPI_OST_FAILURE, "no pci.sys answer: no _EJ0, _OST failure");
}

void test_already_removed_stack() {
    Qemu q;
    Guest g = boot(q, 5);
    g.pdo_irp(g.pdos[0], RX7900_PCIHP_MN_QUERY_REMOVE_DEVICE, "QUERY_REMOVE");   // e.g. Device Manager disable
    g.pdo_irp(g.pdos[0], RX7900_PCIHP_MN_REMOVE_DEVICE, "REMOVE");
    g.pdos[0].devnode_present = false;
    q.device_del(5);
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_EJECTED && same(g.trace, {"FIND", "EJ0", "RESCAN"}),
          "removed stack: _EJ0 without a new request");
}

void test_async_kernel_and_refusal() {
    Qemu q;
    Guest g = boot(q, 5);
    g.async_after_polls = 3;
    q.device_del(5);
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_EJECTED, "async: accepted after polling");

    Qemu q2;
    Guest h = boot(q2, 5);
    h.async_after_polls = 1000;                 // never reaches the PDO (e.g. not disableable)
    q2.device_del(5);
    r = h.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_VETOED && q2.slots[5].populated,
          "refused before QUERY_REMOVE: treated as veto after the poll budget");
}

void test_hot_add() {
    Qemu q;
    Guest g;
    g.qemu = &q;
    q.gpe_enabled = true;
    q.device_add(9, 0x8086, 0x100e);
    auto r = g.gpe();
    check(r.size() == 1 && r[0] == RX7900_ACPI_HP_RESULT_RESCANNED, "device_add: Device Check rescans");
    check(g.find(9) && same(g.find(9)->driver.irps, {"START"}), "device_add: new function enumerated and started");
    check(q.up == 0, "device_add: PCIU cleared on read");
}

void test_mapping_table() {
    struct Row { unsigned long notify, role, first; };
    const Row rows[] = {
        {RX7900_ACPI_NOTIFY_BUS_CHECK, RX7900_ACPI_HP_ROLE_ROOT_BRIDGE, RX7900_ACPI_HP_STEP_RESCAN_BUS},
        {RX7900_ACPI_NOTIFY_BUS_CHECK, RX7900_ACPI_HP_ROLE_ROOT_SLOT, RX7900_ACPI_HP_STEP_RESCAN_BUS},
        {RX7900_ACPI_NOTIFY_DEVICE_CHECK, RX7900_ACPI_HP_ROLE_ROOT_BRIDGE, RX7900_ACPI_HP_STEP_RESCAN_BUS},
        {RX7900_ACPI_NOTIFY_DEVICE_CHECK, RX7900_ACPI_HP_ROLE_ROOT_SLOT, RX7900_ACPI_HP_STEP_RESCAN_BUS},
        {RX7900_ACPI_NOTIFY_EJECT_REQUEST, RX7900_ACPI_HP_ROLE_ROOT_SLOT, RX7900_ACPI_HP_STEP_OST},
        {RX7900_ACPI_NOTIFY_EJECT_REQUEST, RX7900_ACPI_HP_ROLE_ROOT_BRIDGE, RX7900_ACPI_HP_STEP_OST},
        {0x02, RX7900_ACPI_HP_ROLE_ROOT_SLOT, RX7900_ACPI_HP_STEP_DONE},     // Device Wake
        {0x80, RX7900_ACPI_HP_ROLE_ROOT_SLOT, RX7900_ACPI_HP_STEP_DONE},     // device specific
        {RX7900_ACPI_NOTIFY_EJECT_REQUEST, RX7900_ACPI_HP_ROLE_NONE, RX7900_ACPI_HP_STEP_DONE},
        {RX7900_ACPI_NOTIFY_DEVICE_CHECK, RX7900_ACPI_HP_ROLE_NONE, RX7900_ACPI_HP_STEP_DONE},
    };
    for (const Row& row : rows) {
        RX7900_ACPI_HP_MACHINE m;
        check(Rx7900AcpiHpBegin(&m, row.notify, row.role) == row.first, "mapping: first step");
    }
    RX7900_ACPI_HP_MACHINE m;
    Rx7900AcpiHpBegin(&m, RX7900_ACPI_NOTIFY_EJECT_REQUEST, RX7900_ACPI_HP_ROLE_ROOT_BRIDGE);
    check(m.OstStatus == RX7900_ACPI_OST_EJECT_NOT_SUPPORTED && Rx7900AcpiHpAdvance(&m) == RX7900_ACPI_HP_STEP_DONE &&
          m.Result == RX7900_ACPI_HP_RESULT_UNSUPPORTED, "mapping: eject on the host bridge is refused with 0x80");
    Rx7900AcpiHpBegin(&m, RX7900_ACPI_NOTIFY_DEVICE_CHECK, RX7900_ACPI_HP_ROLE_ROOT_SLOT);
    check(Rx7900AcpiHpAdvance(&m) == RX7900_ACPI_HP_STEP_OST && m.OstStatus == RX7900_ACPI_OST_SUCCESS &&
          Rx7900AcpiHpAdvance(&m) == RX7900_ACPI_HP_STEP_DONE && m.Result == RX7900_ACPI_HP_RESULT_RESCANNED,
          "mapping: device check reports success after the rescan");
}

void test_ej0_failure() {
    RX7900_ACPI_HP_MACHINE m;
    unsigned long s = Rx7900AcpiHpBegin(&m, RX7900_ACPI_NOTIFY_EJECT_REQUEST, RX7900_ACPI_HP_ROLE_ROOT_SLOT);
    s = Rx7900AcpiHpAdvance(&m);
    m.Answered = 1;
    s = Rx7900AcpiHpAdvance(&m);
    check(s == RX7900_ACPI_HP_STEP_EVALUATE_EJ0, "_EJ0 failure: empty slot goes to _EJ0");
    m.Ej0Ok = 0;
    s = Rx7900AcpiHpAdvance(&m);
    check(s == RX7900_ACPI_HP_STEP_OST && m.OstStatus == RX7900_ACPI_OST_FAILURE, "_EJ0 failure: _OST failure");
    check(Rx7900AcpiHpAdvance(&m) == RX7900_ACPI_HP_STEP_DONE && m.Result == RX7900_ACPI_HP_RESULT_EJ0_FAILED,
          "_EJ0 failure: no re-enumeration");
}

// Every input combination: _EJ0 never runs for a live, childless stack unless the PDO saw the
// removal (QUERY_REMOVE or REMOVE after the request), and the machine always terminates.
void test_safety_exhaustive() {
    unsigned cases = 0, violations = 0, unbounded = 0;
    for (int answered = 0; answered < 2; ++answered)
    for (int have = 0; have < 2; ++have)
    for (unsigned long state0 = 0; state0 < 4; ++state0)
    for (int children = 0; children < 2; ++children)
    for (int gone = 0; gone < 2; ++gone)
    for (unsigned long state1 = 0; state1 < 4; ++state1)
    for (int changed = 0; changed < 2; ++changed)
    for (int ej0 = 0; ej0 < 2; ++ej0) {
        ++cases;
        RX7900_ACPI_HP_MACHINE m;
        bool requested = false, ejected = false, rescanned = false, accepted_seen = false;
        unsigned steps = 0;
        unsigned long s = Rx7900AcpiHpBegin(&m, RX7900_ACPI_NOTIFY_EJECT_REQUEST, RX7900_ACPI_HP_ROLE_ROOT_SLOT);
        while (s != RX7900_ACPI_HP_STEP_DONE && ++steps < 200) {
            if (s == RX7900_ACPI_HP_STEP_FIND_PDO) { m.Answered = answered; m.HavePdo = have; m.State = state0; m.Events = 5; }
            if (s == RX7900_ACPI_HP_STEP_CHECK_CHILDREN) m.MayHaveChildren = children;
            if (s == RX7900_ACPI_HP_STEP_REQUEST_EJECT) requested = true;
            if (s == RX7900_ACPI_HP_STEP_WAIT_REMOVAL) {
                m.PdoGone = gone; m.StateNow = state1; m.EventsNow = changed ? 6 : 5;
                if (gone || (changed && (state1 == RX7900_PCIHP_STATE_QUERY_REMOVED || state1 == RX7900_PCIHP_STATE_REMOVED)))
                    accepted_seen = true;
            }
            if (s == RX7900_ACPI_HP_STEP_EVALUATE_EJ0) {
                ejected = true; m.Ej0Ok = ej0;
                bool live = have && state0 != RX7900_PCIHP_STATE_REMOVED && !children;
                if (!answered || (live && !(requested && accepted_seen))) ++violations;
            }
            if (s == RX7900_ACPI_HP_STEP_RESCAN_BUS) { rescanned = true; if (!ejected || !ej0) ++violations; }
            s = Rx7900AcpiHpAdvance(&m);
        }
        if (steps >= 200) ++unbounded;
        if ((m.Result == RX7900_ACPI_HP_RESULT_EJECTED) != (ejected && ej0 && rescanned)) ++violations;
        if (m.Result == RX7900_ACPI_HP_RESULT_VETOED && ejected) ++violations;
        if (children && have && state0 != RX7900_PCIHP_STATE_REMOVED && requested) ++violations;
    }
    check(cases == 1024, "exhaustive: all combinations");
    check(violations == 0, "exhaustive: no _EJ0 before an accepted removal, no rescan without _EJ0");
    check(unbounded == 0, "exhaustive: bounded");
}

void test_pci_tracking_and_presence() {
    unsigned long state = 0, events = 0;
    Rx7900PciHpTrackPnp(RX7900_PCIHP_MN_QUERY_REMOVE_DEVICE, &state, &events);
    check(state == RX7900_PCIHP_STATE_QUERY_REMOVED && events == 1, "track: query remove");
    Rx7900PciHpTrackPnp(RX7900_PCIHP_MN_CANCEL_REMOVE, &state, &events);
    check(state == RX7900_PCIHP_STATE_REMOVE_CANCELLED && events == 2, "track: cancel");
    Rx7900PciHpTrackPnp(0x07 /* QUERY_DEVICE_RELATIONS */, &state, &events);
    Rx7900PciHpTrackPnp(0x14 /* QUERY_PNP_DEVICE_STATE */, &state, &events);
    check(state == RX7900_PCIHP_STATE_REMOVE_CANCELLED && events == 2, "track: other IRPs ignored");
    Rx7900PciHpTrackPnp(RX7900_PCIHP_MN_SURPRISE_REMOVAL, &state, &events);
    check(state == RX7900_PCIHP_STATE_REMOVED && events == 3, "track: surprise removal");
    Rx7900PciHpTrackPnp(RX7900_PCIHP_MN_START_DEVICE, &state, &events);
    check(state == RX7900_PCIHP_STATE_ACTIVE && events == 4, "track: restart");

    check(!Rx7900PciHpFunctionGone(4, 0x1af4, 0x1000, 0x1af4, 0x1000), "present: same ids");
    check(Rx7900PciHpFunctionGone(4, 0xFFFF, 0xFFFF, 0x1af4, 0x1000), "gone: 0xFFFF");
    check(Rx7900PciHpFunctionGone(2, 0xFFFF, 0xFFFF, 0x1af4, 0x1000), "gone: short HAL read");
    check(Rx7900PciHpFunctionGone(4, 0x8086, 0x100e, 0x1af4, 0x1000), "gone: another device in the slot");
    check(Rx7900PciHpFunctionGone(4, 0x0000, 0x0000, 0x1af4, 0x1000), "gone: vendor 0");
    check(Rx7900PciHpFunctionGone(2, 0x1af4, 0x1000, 0x1af4, 0x1000), "gone: a short read wins over stale ids");
    check(Rx7900PciHpFunctionGone(4, 0x1af4, 0x1041, 0x1af4, 0x1000), "gone: same vendor, other device id");
}

// Stock FdoQueryBusRelations sizes the buffer with DeviceListCount (present functions) but
// writes every list entry; the pruned list keeps count and entries equal.
void test_stock_relations_overrun() {
    struct Entry { int slot; bool present; };
    std::vector<Entry> list = {{1, true}, {3, true}, {5, false}, {7, true}};
    unsigned present = 0;
    for (auto& e : list) present += e.present;
    unsigned written = static_cast<unsigned>(list.size());
    check(written == present + 1, "stock: one pointer written past the relations buffer");
    check(list[present].slot == 7 && list[2].slot == 5, "stock: relations Count omits slot 7, reports vanished slot 5");
    std::vector<Entry> pruned;
    for (auto& e : list) if (e.present) pruned.push_back(e);
    check(pruned.size() == present, "pruned: entries equal DeviceListCount");
}

}  // namespace

int main() {
    test_stock_gpe_masked();
    test_unplug_accepted();
    test_unplug_vetoed_then_retried();
    test_unplug_with_children();
    test_ost_where_defined();
    test_empty_and_unenumerated_slots();
    test_pci_not_listening();
    test_already_removed_stack();
    test_async_kernel_and_refusal();
    test_hot_add();
    test_mapping_table();
    test_ej0_failure();
    test_safety_exhaustive();
    test_pci_tracking_and_presence();
    test_stock_relations_overrun();

    std::printf("acpi_hotplug_model: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
