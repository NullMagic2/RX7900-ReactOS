/*
Module: rx7900.tests.pnp_query_remove_model
Purpose: Model the ntoskrnl query-and-remove sequence of reactos-pnp-query-remove.patch.
Created: 2026-10-24
Architecture: Host-only model of PiControlQueryRemoveDevice, PiQueryAndRemoveDevice,
PiQueryRemoveDeviceTree and PiPrepareDisabledDeviceForStart (ntoskrnl io/pnpmgr,
RX7900_PNP_QUERY_REMOVE). Each model function follows the patched C function step by
step; device stacks are fixed-capacity fakes that log IRPs and count PDO references.
*/
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace {

// Values from sdk/include/psdk/ntstatus.h, cfg.h and PNP_VETO_TYPE in the pinned trees.
constexpr std::uint32_t status_success = 0x00000000;
constexpr std::uint32_t status_plugplay_query_vetoed = 0x80000028;
constexpr std::uint32_t status_unsuccessful = 0xC0000001;
constexpr std::uint32_t status_not_supported = 0xC00000BB;
constexpr std::uint32_t status_invalid_parameter = 0xC000000D;
constexpr std::uint32_t status_no_such_device = 0xC000000E;
constexpr std::uint32_t status_invalid_device_request = 0xC0000010;
constexpr std::uint32_t status_invalid_device_state = 0xC0000184;
constexpr std::uint32_t cm_prob_failed_start = 0x0000000A;
constexpr std::uint32_t cm_prob_disabled = 0x00000016;

enum class VetoType : int {
    unknown = 0,
    legacy_device = 1,
    device = 6,
    non_disableable = 10,
};

enum class NodeState {
    initialized,
    drivers_added,
    started,
    query_removed,
    awaiting_queued_removal,
    removed,
};

enum class IrpKind {
    query_remove,
    cancel_remove,
    remove,
};

constexpr int max_nodes = 16;
constexpr int max_links = 4;
constexpr int max_log = 64;
constexpr int no_node = -1;

bool nt_success(std::uint32_t status) {
    return static_cast<std::int32_t>(status) >= 0;
}

struct Node {
    const char* instance = "";
    int parent = no_node;
    int children[max_links]{};
    int child_count = 0;
    int relations[max_links]{};
    int relation_count = 0;
    int level = 0;
    NodeState state = NodeState::started;
    NodeState previous_state = NodeState::started;
    bool has_problem = false;
    std::uint32_t problem = 0;
    bool legacy_driver = false;
    bool not_disableable = false;
    bool device_gone = false;
    // Driver behaviour of the fake stack.
    std::uint32_t query_remove_status = status_success;
    bool bus_deletes_pdo_on_remove = false;
    bool fdo_detaches_on_remove = true;
    bool fdo_deletes_children_on_remove = true;
    // Object state.
    bool fdo_attached = true;
    bool delete_pending = false;
    bool freed = false;
    int references = 2;  // IoCreateDevice reference plus the device node's reference
    bool has_resources = true;
};

struct LogEntry {
    IrpKind kind = IrpKind::query_remove;
    int node = no_node;
};

struct Tree {
    Node nodes[max_nodes]{};
    int count = 0;
    LogEntry log[max_log]{};
    int log_count = 0;
    unsigned free_violations = 0;
};

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", description);
    }
}

int add_node(Tree& tree, const char* instance, int parent) {
    const int index = tree.count++;
    Node& node = tree.nodes[index];
    node.instance = instance;
    node.parent = parent;
    if (parent != no_node) {
        Node& parent_node = tree.nodes[parent];
        parent_node.children[parent_node.child_count++] = index;
        node.level = parent_node.level + 1;
    }
    return index;
}

void add_relation(Tree& tree, int node, int relation) {
    Node& owner = tree.nodes[node];
    owner.relations[owner.relation_count++] = relation;
}

void log_irp(Tree& tree, IrpKind kind, int node) {
    if (tree.log_count < max_log) {
        tree.log[tree.log_count++] = LogEntry{kind, node};
    }
}

bool log_is(const Tree& tree, std::initializer_list<LogEntry> expected) {
    if (static_cast<int>(expected.size()) != tree.log_count) {
        return false;
    }
    int index = 0;
    for (const LogEntry& entry : expected) {
        if (tree.log[index].kind != entry.kind || tree.log[index].node != entry.node) {
            return false;
        }
        ++index;
    }
    return true;
}

// IopFreeDeviceNode: reached from IopDeleteDevice when the last PDO reference goes.
void unlink_from_parent(Tree& tree, int index) {
    Node& node = tree.nodes[index];
    if (node.parent == no_node) {
        return;
    }
    Node& parent = tree.nodes[node.parent];
    int kept = 0;
    for (int i = 0; i < parent.child_count; ++i) {
        if (parent.children[i] != index) {
            parent.children[kept++] = parent.children[i];
        }
    }
    parent.child_count = kept;
}

void dereference(Tree& tree, int index) {
    Node& node = tree.nodes[index];
    if (node.freed || node.references <= 0) {
        ++tree.free_violations;  // over-dereference
        return;
    }
    if (--node.references == 0) {
        // ASSERTs of IopFreeDeviceNode
        if (node.state != NodeState::removed || node.child_count != 0) {
            ++tree.free_violations;
        }
        node.freed = true;
        unlink_from_parent(tree, index);
    }
}

void delete_pdo(Tree& tree, int index) {
    Node& node = tree.nodes[index];
    if (!node.delete_pending) {
        node.delete_pending = true;
        dereference(tree, index);  // IoDeleteDevice drops the IoCreateDevice reference
    }
}

// Fake stack response to IRP_MN_QUERY_REMOVE_DEVICE (IopQueryRemoveDevice).
std::uint32_t send_query_remove(Tree& tree, int index) {
    log_irp(tree, IrpKind::query_remove, index);
    return tree.nodes[index].query_remove_status;
}

// IopCancelRemoveDevice
void send_cancel_remove(Tree& tree, int index) {
    log_irp(tree, IrpKind::cancel_remove, index);
}

// IopSendRemoveDevice: IRP_MN_REMOVE_DEVICE, DeviceNodeRemoved, drop the node's reference.
void send_remove_device(Tree& tree, int index) {
    Node& node = tree.nodes[index];
    if (node.state != NodeState::awaiting_queued_removal) {
        ++tree.free_violations;  // ASSERT in IopSendRemoveDevice
    }
    log_irp(tree, IrpKind::remove, index);
    if (node.fdo_attached && node.fdo_detaches_on_remove) {
        node.fdo_attached = false;
    }
    if (node.fdo_deletes_children_on_remove) {
        const int child_count = node.child_count;
        int children[max_links]{};
        for (int i = 0; i < child_count; ++i) {
            children[i] = node.children[i];
        }
        for (int i = 0; i < child_count; ++i) {
            delete_pdo(tree, children[i]);
        }
    }
    if (node.bus_deletes_pdo_on_remove) {
        delete_pdo(tree, index);
    }
    node.has_resources = false;  // resource map hack: Count = 0
    node.state = NodeState::removed;
    dereference(tree, index);
}

void set_state(Node& node, NodeState state) {
    if (node.state != state) {
        node.previous_state = node.state;
        node.state = state;
    }
}

void set_problem(Node& node, std::uint32_t problem) {
    node.has_problem = true;
    node.problem = problem;
}

// The query-remove list: newest first, each entry holds a PDO reference.
struct QueryList {
    int entries[max_nodes]{};
    int count = 0;
};

bool in_list(const QueryList& list, int index) {
    for (int i = 0; i < list.count; ++i) {
        if (list.entries[i] == index) {
            return true;
        }
    }
    return false;
}

void insert_head(Tree& tree, QueryList& list, int index) {
    for (int i = list.count; i > 0; --i) {
        list.entries[i] = list.entries[i - 1];
    }
    list.entries[0] = index;
    ++list.count;
    ++tree.nodes[index].references;  // ObReferenceObject
}

void remove_at(QueryList& list, int position) {
    for (int i = position; i + 1 < list.count; ++i) {
        list.entries[i] = list.entries[i + 1];
    }
    --list.count;
}

struct QueryResult {
    VetoType veto_type = VetoType::unknown;
    int veto_device = no_node;
};

std::uint32_t query_remove_veto(Tree& tree, int index, VetoType type, QueryResult& result) {
    ++tree.nodes[index].references;  // VetoDevice reference
    result.veto_type = type;
    result.veto_device = index;
    return status_plugplay_query_vetoed;
}

// PiQueryRemoveDeviceTree
std::uint32_t query_remove_device_tree(Tree& tree, int index, QueryList& list, QueryResult& result) {
    Node& node = tree.nodes[index];

    if (node.legacy_driver) {
        return query_remove_veto(tree, index, VetoType::legacy_device, result);
    }
    if (node.not_disableable) {
        return query_remove_veto(tree, index, VetoType::non_disableable, result);
    }

    insert_head(tree, list, index);
    if (!nt_success(send_query_remove(tree, index))) {
        return query_remove_veto(tree, index, VetoType::device, result);
    }
    set_state(node, NodeState::query_removed);

    std::uint32_t status = status_success;
    for (int i = 0; i < node.relation_count; ++i) {
        const int relation = node.relations[i];
        if (nt_success(status) && relation != 0 &&
            tree.nodes[relation].state != NodeState::removed && !in_list(list, relation)) {
            status = query_remove_device_tree(tree, relation, list, result);
        }
    }
    if (!nt_success(status)) {
        return status;
    }

    for (int i = 0; i < node.child_count; ++i) {
        const int child = node.children[i];
        if (tree.nodes[child].state != NodeState::removed && !in_list(list, child)) {
            status = query_remove_device_tree(tree, child, list, result);
            if (!nt_success(status)) {
                return status;
            }
        }
    }
    return status_success;
}

// PiQueryAndRemoveDevice (node 0 is the root device node)
std::uint32_t query_and_remove_device(Tree& tree, int index, QueryResult& result) {
    Node& node = tree.nodes[index];

    result = QueryResult{};
    if (index == 0) {
        return status_invalid_device_request;
    }
    if (node.state == NodeState::removed || node.state == NodeState::initialized) {
        if (node.device_gone) {
            return status_no_such_device;
        }
        set_problem(node, cm_prob_disabled);
        return status_success;
    }
    if (node.state != NodeState::started) {
        return status_invalid_device_state;
    }

    QueryList list;
    const std::uint32_t status = query_remove_device_tree(tree, index, list, result);
    if (!nt_success(status)) {
        while (list.count != 0) {
            const int entry = list.entries[0];
            remove_at(list, 0);
            send_cancel_remove(tree, entry);
            Node& entry_node = tree.nodes[entry];
            if (entry_node.state == NodeState::query_removed) {
                set_state(entry_node, entry_node.previous_state);
            }
            dereference(tree, entry);
        }
        return status;
    }

    set_problem(node, cm_prob_disabled);
    while (list.count != 0) {
        int deepest = 0;
        for (int i = 1; i < list.count; ++i) {
            if (tree.nodes[list.entries[i]].level > tree.nodes[list.entries[deepest]].level) {
                deepest = i;
            }
        }
        const int entry = list.entries[deepest];
        remove_at(list, deepest);
        Node& entry_node = tree.nodes[entry];

        if (entry_node.parent != no_node && in_list(list, entry_node.parent)) {
            set_state(entry_node, NodeState::awaiting_queued_removal);
            send_remove_device(tree, entry);
        } else {
            ++entry_node.references;
            set_state(entry_node, NodeState::awaiting_queued_removal);
            send_remove_device(tree, entry);
            if (entry_node.delete_pending) {
                dereference(tree, entry);
            }
        }
        dereference(tree, entry);
    }
    return status_success;
}

// PiPrepareDisabledDeviceForStart followed by the unchanged PiActionStartDevice gate.
bool start_disabled_device(Tree& tree, int index) {
    Node& node = tree.nodes[index];

    if (node.has_problem && node.problem == cm_prob_disabled) {
        if (node.state == NodeState::removed) {
            if (node.device_gone || node.delete_pending || node.fdo_attached || node.child_count != 0) {
                return false;
            }
            node.has_resources = false;
            set_state(node, NodeState::initialized);
        } else if (node.state != NodeState::initialized) {
            return false;
        }
        node.has_problem = false;
        node.problem = 0;
    }
    if (node.state != NodeState::initialized || node.has_problem) {
        return false;
    }
    // PiDevNodeStateMachine: AddDevice attaches a new FDO, then start.
    node.fdo_attached = true;
    node.has_resources = true;
    set_state(node, NodeState::started);
    return true;
}

// PiControlQueryRemoveDevice: flags, veto type and the WCHAR veto name buffer.
struct ControlData {
    std::uint32_t flags = 0;
    VetoType veto_type = VetoType::unknown;
    char16_t* veto_name = nullptr;
    std::uint32_t name_length = 0;  // in WCHARs
};

std::uint32_t control_query_remove(Tree& tree, int index, ControlData& data) {
    if (data.flags != 0) {
        return status_invalid_parameter;
    }
    if (index == no_node) {
        return status_no_such_device;
    }
    ++tree.nodes[index].references;  // IopGetDeviceObjectFromDeviceInstance

    QueryResult result;
    const std::uint32_t status = query_and_remove_device(tree, index, result);
    data.veto_type = result.veto_type;
    if (result.veto_device != no_node && data.veto_name != nullptr && data.name_length != 0) {
        const char* instance = tree.nodes[result.veto_device].instance;
        std::uint32_t length = static_cast<std::uint32_t>(std::strlen(instance));
        if (length > data.name_length - 1) {
            length = data.name_length - 1;
        }
        for (std::uint32_t i = 0; i < length; ++i) {
            data.veto_name[i] = static_cast<char16_t>(instance[i]);
        }
        data.veto_name[length] = u'\0';
    }
    if (result.veto_device != no_node) {
        dereference(tree, result.veto_device);
    }
    dereference(tree, index);
    return status;
}

bool name_is(const char16_t* name, const char* expected) {
    std::size_t i = 0;
    for (; expected[i] != '\0'; ++i) {
        if (name[i] != static_cast<char16_t>(expected[i])) {
            return false;
        }
    }
    return name[i] == u'\0';
}

// Root, a PCI bus, the display under test and an unrelated sibling.
struct DisplayTree {
    Tree tree;
    int root = 0;
    int bus = 0;
    int display = 0;
    int sibling = 0;
};

DisplayTree make_display_tree() {
    DisplayTree t;
    t.root = add_node(t.tree, "HTREE\\ROOT\\0", no_node);
    t.bus = add_node(t.tree, "ACPI\\PNP0A03\\0", t.root);
    t.display = add_node(t.tree, "PCI\\VEN_1002&DEV_744C\\3&0&10", t.bus);
    t.sibling = add_node(t.tree, "PCI\\VEN_8086&DEV_100E\\3&0&18", t.bus);
    return t;
}

void test_single_device_disable_and_enable() {
    DisplayTree t = make_display_tree();
    Tree& tree = t.tree;
    ControlData data;

    expect(control_query_remove(tree, t.display, data) == status_success, "disable succeeds");
    expect(log_is(tree, {{IrpKind::query_remove, t.display}, {IrpKind::remove, t.display}}),
        "a leaf device gets exactly QUERY_REMOVE then REMOVE");
    const Node& display = tree.nodes[t.display];
    expect(display.state == NodeState::removed && display.has_problem &&
        display.problem == cm_prob_disabled, "the device is removed with CM_PROB_DISABLED");
    expect(display.references == 2 && !display.freed,
        "the parent bus keeps the PDO, so the device node keeps its reference");
    expect(data.veto_type == VetoType::unknown, "no veto type on success");
    expect(tree.nodes[t.sibling].state == NodeState::started && tree.nodes[t.bus].state ==
        NodeState::started, "parent and siblings stay started");
    expect(tree.free_violations == 0, "no reference or free violation");

    expect(start_disabled_device(tree, t.display), "enable restarts the disabled device");
    expect(!display.has_problem && display.state == NodeState::started && display.fdo_attached,
        "enable clears CM_PROB_DISABLED and rebuilds the stack");

    tree.log_count = 0;
    expect(control_query_remove(tree, t.display, data) == status_success &&
        log_is(tree, {{IrpKind::query_remove, t.display}, {IrpKind::remove, t.display}}),
        "a second disable/enable cycle sends the same IRPs");
    expect(start_disabled_device(tree, t.display) && display.references == 2,
        "repeated cycles keep the reference count balanced");
}

void test_children_removed_before_parent() {
    DisplayTree t = make_display_tree();
    Tree& tree = t.tree;
    const int monitor0 = add_node(tree, "DISPLAY\\DEFAULT_MONITOR\\4&0&0", t.display);
    const int monitor1 = add_node(tree, "DISPLAY\\DEFAULT_MONITOR\\4&0&1", t.display);
    const int child_of_monitor = add_node(tree, "SW\\MONITOR_FILTER\\5&0&0", monitor0);
    ControlData data;

    expect(control_query_remove(tree, t.display, data) == status_success, "subtree disable succeeds");
    expect(log_is(tree, {
            {IrpKind::query_remove, t.display}, {IrpKind::query_remove, monitor0},
            {IrpKind::query_remove, child_of_monitor}, {IrpKind::query_remove, monitor1},
            {IrpKind::remove, child_of_monitor}, {IrpKind::remove, monitor1},
            {IrpKind::remove, monitor0}, {IrpKind::remove, t.display}}),
        "query top-down in tree order, remove deepest first (newest first among equals)");
    expect(tree.nodes[monitor0].freed && tree.nodes[monitor1].freed &&
        tree.nodes[child_of_monitor].freed, "children deleted by their parent are freed");
    expect(tree.nodes[t.display].child_count == 0, "freed children are unlinked");
    expect(tree.free_violations == 0, "children free only after they are removed and childless");
    expect(start_disabled_device(tree, t.display), "the torn-down parent can restart");
}

void test_child_veto_cancels_everything_queried() {
    DisplayTree t = make_display_tree();
    Tree& tree = t.tree;
    const int monitor0 = add_node(tree, "DISPLAY\\DEFAULT_MONITOR\\4&0&0", t.display);
    const int monitor1 = add_node(tree, "DISPLAY\\DEFAULT_MONITOR\\4&0&1", t.display);
    const int monitor2 = add_node(tree, "DISPLAY\\DEFAULT_MONITOR\\4&0&2", t.display);
    tree.nodes[monitor1].query_remove_status = status_unsuccessful;
    char16_t name[64]{};
    ControlData data;
    data.veto_name = name;
    data.name_length = 64;

    expect(control_query_remove(tree, t.display, data) == status_plugplay_query_vetoed,
        "a failing QUERY_REMOVE vetoes the disable");
    expect(log_is(tree, {
            {IrpKind::query_remove, t.display}, {IrpKind::query_remove, monitor0},
            {IrpKind::query_remove, monitor1}, {IrpKind::cancel_remove, monitor1},
            {IrpKind::cancel_remove, monitor0}, {IrpKind::cancel_remove, t.display}}),
        "the vetoing device and every queried device get CANCEL_REMOVE, newest first");
    expect(data.veto_type == VetoType::device && name_is(name, "DISPLAY\\DEFAULT_MONITOR\\4&0&1"),
        "PNP_VetoDevice names the device whose driver failed the query");
    for (int node : {t.display, monitor0, monitor1, monitor2}) {
        expect(tree.nodes[node].state == NodeState::started && !tree.nodes[node].has_problem &&
            tree.nodes[node].references == 2, "every device is back to started with its references");
    }
    expect(tree.free_violations == 0, "veto path releases exactly its references");
}

void test_target_veto_and_policy_vetoes() {
    {
        DisplayTree t = make_display_tree();
        t.tree.nodes[t.display].query_remove_status = status_not_supported;
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_plugplay_query_vetoed &&
            data.veto_type == VetoType::device, "an unhandled QUERY_REMOVE (STATUS_NOT_SUPPORTED) vetoes");
        expect(log_is(t.tree, {{IrpKind::query_remove, t.display}, {IrpKind::cancel_remove, t.display}}),
            "the target that failed the query also gets CANCEL_REMOVE");
    }
    {
        DisplayTree t = make_display_tree();
        const int child = add_node(t.tree, "HID\\VID_0627&PID_0001\\5&0&0", t.display);
        t.tree.nodes[child].not_disableable = true;
        char16_t name[8]{};
        ControlData data;
        data.veto_name = name;
        data.name_length = 8;
        expect(control_query_remove(t.tree, t.display, data) == status_plugplay_query_vetoed &&
            data.veto_type == VetoType::non_disableable, "a non-disableable child vetoes");
        expect(log_is(t.tree, {{IrpKind::query_remove, t.display}, {IrpKind::cancel_remove, t.display}}),
            "the non-disableable device itself is never queried");
        expect(name_is(name, "HID\\VID") && name[7] == u'\0',
            "the veto name is truncated to NameLength - 1 WCHARs and terminated");
    }
    {
        DisplayTree t = make_display_tree();
        t.tree.nodes[t.display].legacy_driver = true;
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_plugplay_query_vetoed &&
            data.veto_type == VetoType::legacy_device && t.tree.log_count == 0,
            "a legacy device is vetoed without any IRP");
        expect(t.tree.nodes[t.display].references == 2, "the veto reference is released");
    }
}

void test_removal_relations() {
    DisplayTree t = make_display_tree();
    Tree& tree = t.tree;
    const int audio = add_node(tree, "HDAUDIO\\FUNC_01&VEN_1002\\4&0&1", t.sibling);
    const int audio_child = add_node(tree, "SW\\KMIXER\\5&0&0", audio);
    add_relation(tree, t.display, audio);
    add_relation(tree, t.display, audio);       // duplicate relation
    add_relation(tree, t.display, t.root);      // the root is never a relation target
    add_relation(tree, audio, t.display);       // cycle back to the target
    ControlData data;

    expect(control_query_remove(tree, t.display, data) == status_success, "relations disable succeeds");
    expect(log_is(tree, {
            {IrpKind::query_remove, t.display}, {IrpKind::query_remove, audio},
            {IrpKind::query_remove, audio_child}, {IrpKind::remove, audio_child},
            {IrpKind::remove, audio}, {IrpKind::remove, t.display}}),
        "relations are queried once, before children, and removed deepest first");
    expect(tree.nodes[audio].state == NodeState::removed && tree.nodes[audio].references == 2 &&
        !tree.nodes[audio].has_problem, "a relation on another bus keeps its PDO reference and no problem");
    expect(tree.nodes[audio_child].freed, "the relation's child is deleted by the relation's FDO");
    expect(tree.nodes[t.sibling].state == NodeState::started, "the relation's parent is untouched");
    expect(tree.free_violations == 0, "relations keep reference accounting balanced");
}

void test_relation_veto_cancels_target() {
    DisplayTree t = make_display_tree();
    Tree& tree = t.tree;
    const int child = add_node(tree, "DISPLAY\\DEFAULT_MONITOR\\4&0&0", t.display);
    const int audio = add_node(tree, "HDAUDIO\\FUNC_01&VEN_1002\\4&0&1", t.sibling);
    tree.nodes[audio].query_remove_status = status_unsuccessful;
    add_relation(tree, t.display, audio);
    ControlData data;

    expect(control_query_remove(tree, t.display, data) == status_plugplay_query_vetoed,
        "a vetoing relation vetoes the disable");
    expect(log_is(tree, {
            {IrpKind::query_remove, t.display}, {IrpKind::query_remove, audio},
            {IrpKind::cancel_remove, audio}, {IrpKind::cancel_remove, t.display}}),
        "children after a vetoed relation are never queried; the target is cancelled");
    expect(tree.nodes[child].state == NodeState::started && tree.nodes[t.display].state ==
        NodeState::started, "nothing is left query-removed");
}

void test_bus_deleting_its_pdo() {
    DisplayTree t = make_display_tree();
    Tree& tree = t.tree;
    tree.nodes[t.display].bus_deletes_pdo_on_remove = true;  // PnpRoot-style PDO
    ControlData data;

    expect(control_query_remove(tree, t.display, data) == status_success, "disable succeeds");
    expect(tree.nodes[t.display].freed && tree.free_violations == 0,
        "a PDO deleted on REMOVE releases the device node exactly once");
}

void test_restart_refusals() {
    {
        DisplayTree t = make_display_tree();
        t.tree.nodes[t.display].fdo_detaches_on_remove = false;
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_success,
            "disable of a driver that stays attached still removes");
        expect(!start_disabled_device(t.tree, t.display) && t.tree.nodes[t.display].problem ==
            cm_prob_disabled, "no second AddDevice on a stack whose FDO did not detach");
    }
    {
        DisplayTree t = make_display_tree();
        const int child = add_node(t.tree, "DISPLAY\\DEFAULT_MONITOR\\4&0&0", t.display);
        t.tree.nodes[t.display].fdo_deletes_children_on_remove = false;
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_success &&
            t.tree.nodes[child].state == NodeState::removed, "child removed but not deleted");
        expect(!start_disabled_device(t.tree, t.display), "restart refused while stale children remain");
    }
    {
        DisplayTree t = make_display_tree();
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_success, "disable succeeds");
        t.tree.nodes[t.display].device_gone = true;
        expect(!start_disabled_device(t.tree, t.display), "a device reported missing is not restarted");
    }
    {
        DisplayTree t = make_display_tree();
        set_problem(t.tree.nodes[t.sibling], cm_prob_failed_start);
        t.tree.nodes[t.sibling].state = NodeState::removed;
        t.tree.nodes[t.sibling].fdo_attached = false;
        expect(!start_disabled_device(t.tree, t.sibling),
            "enable leaves devices removed for other problems alone");
    }
}

void test_states_without_running_stack() {
    {
        DisplayTree t = make_display_tree();
        Node& sibling = t.tree.nodes[t.sibling];
        set_problem(sibling, cm_prob_failed_start);
        sibling.state = NodeState::removed;
        sibling.fdo_attached = false;
        ControlData data;
        expect(control_query_remove(t.tree, t.sibling, data) == status_success &&
            t.tree.log_count == 0 && sibling.problem == cm_prob_disabled,
            "a removed device is only marked disabled");
        expect(start_disabled_device(t.tree, t.sibling), "and can then be enabled");
    }
    {
        DisplayTree t = make_display_tree();
        t.tree.nodes[t.display].state = NodeState::initialized;
        t.tree.nodes[t.display].fdo_attached = false;
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_success &&
            t.tree.log_count == 0, "a device without drivers is only marked disabled");
        expect(start_disabled_device(t.tree, t.display), "an initialized disabled device starts");
    }
    {
        DisplayTree t = make_display_tree();
        t.tree.nodes[t.display].state = NodeState::drivers_added;
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_invalid_device_state &&
            t.tree.log_count == 0 && !t.tree.nodes[t.display].has_problem,
            "transient states are refused without side effects");
    }
    {
        DisplayTree t = make_display_tree();
        t.tree.nodes[t.display].state = NodeState::removed;
        t.tree.nodes[t.display].device_gone = true;
        ControlData data;
        expect(control_query_remove(t.tree, t.display, data) == status_no_such_device,
            "a surprise-removed device is gone");
    }
}

void test_control_contract() {
    DisplayTree t = make_display_tree();
    ControlData data;
    data.flags = 1;
    expect(control_query_remove(t.tree, t.display, data) == status_invalid_parameter &&
        t.tree.log_count == 0, "unknown flags are rejected before any IRP");
    data.flags = 0;
    expect(control_query_remove(t.tree, t.root, data) == status_invalid_device_request &&
        t.tree.log_count == 0, "the root device node cannot be removed");
    expect(control_query_remove(t.tree, no_node, data) == status_no_such_device,
        "an unknown instance is STATUS_NO_SUCH_DEVICE");

    t.tree.nodes[t.display].query_remove_status = status_unsuccessful;
    char16_t untouched[4]{u'x', u'y', u'z', u'w'};
    data.veto_name = untouched;
    data.name_length = 0;
    expect(control_query_remove(t.tree, t.display, data) == status_plugplay_query_vetoed &&
        untouched[0] == u'x', "a zero NameLength buffer is never written");
    data.veto_name = nullptr;
    data.name_length = 4;
    expect(control_query_remove(t.tree, t.display, data) == status_plugplay_query_vetoed &&
        data.veto_type == VetoType::device, "a NULL veto name still reports the veto type");
    data.veto_name = untouched;
    data.name_length = 1;
    expect(control_query_remove(t.tree, t.display, data) == status_plugplay_query_vetoed &&
        untouched[0] == u'\0' && untouched[1] == u'y', "a one-WCHAR buffer receives only the terminator");
    expect(t.tree.nodes[t.display].references == 2 && t.tree.free_violations == 0,
        "repeated vetoes leak no reference");
}

}  // namespace

int main() {
    test_single_device_disable_and_enable();
    test_children_removed_before_parent();
    test_child_veto_cancels_everything_queried();
    test_target_veto_and_policy_vetoes();
    test_removal_relations();
    test_relation_veto_cancels_target();
    test_bus_deleting_its_pdo();
    test_restart_refusals();
    test_states_without_running_stack();
    test_control_contract();

    std::printf("pnp_query_remove_model: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
