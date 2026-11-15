/*
Module: rx7900.tests.radeon_installer_fake_system
Purpose: In-memory registry, files, SCM, PnP and display behind the radeonctl seam.
Created: 2026-10-24
Architecture: Test-only SystemOps binding with deterministic fault injection
(crash before the Nth mutation, optionally tearing a copy), locked files, a
restart model for delayed renames and a PnP model of SetupAPI, DIF_REMOVE and
ReactOS re-detection. reactos_setupapi models the ReactOS 0.4.16 behaviour the
installer rig observed: DIF_REMOVE fails with ERROR_CALL_NOT_IMPLEMENTED,
SetupUninstallOEMInf without SUOI_FORCEDELETE succeeds and deletes nothing, and
VideoPort device-map paths name services in upper case. Never touches the host
registry, files or devices.
*/
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "radeon_installer.hpp"

namespace rx7900_test::radeon_fake {

namespace radeon = rx7900::tools::radeon;
namespace error = rx7900::tools::radeon::error;
using Bytes = std::vector<uint8_t>;

inline std::wstring fold(const wchar_t* text) {
    std::wstring folded;
    for (size_t index = 0; text != nullptr && text[index] != 0; ++index) {
        const wchar_t value = text[index];
        folded.push_back(value >= L'A' && value <= L'Z' ? static_cast<wchar_t>(value + 32) : value);
    }
    return folded;
}

inline std::wstring parent_of(const std::wstring& path) {
    const size_t slash = path.rfind(L'\\');
    return slash == std::wstring::npos ? std::wstring() : path.substr(0, slash);
}

struct Value final {
    uint32_t type = 0;
    Bytes data;
    bool operator==(const Value& other) const { return type == other.type && data == other.data; }
};

struct Device final {
    radeon::DeviceRecord record{};
    bool present = true;
    std::wstring inf_path;  // driver key InfPath while bound to us
    bool unbound = false;   // Service deleted in place; next boot installs a driver
    // Enum values the in-place unbinding (pnp_unbind.hpp) works on.
    std::wstring driver_key;     // Driver: {class GUID}\NNNN below Control\Class
    bool class_values = false;   // Class and ClassGUID
    uint32_t config_flags = 0;   // ConfigFlags
};

struct Service final {
    bool exists = false;
    uint32_t type = 1;
    uint32_t start = 3;
    uint32_t error_control = 0;
    bool running = false;
    std::wstring image;
};

struct Snapshot final {
    std::map<std::wstring, std::map<std::wstring, Value>> registry;
    std::set<std::wstring> keys;
    std::map<std::wstring, Bytes> files;
    Service service;
    std::vector<std::pair<std::wstring, Bytes>> filters;  // instance -> upper|lower serialized
    std::vector<std::wstring> bindings;

    bool operator==(const Snapshot& other) const {
        return registry == other.registry && keys == other.keys && files == other.files &&
               service.exists == other.service.exists && service.start == other.service.start &&
               service.image == other.service.image && filters == other.filters && bindings == other.bindings;
    }
};

class FakeSystem final {
public:
    // Paths mirror a ReactOS layout; the package sits on removable media.
    std::wstring system = L"C:\\ReactOS\\system32";
    std::wstring inf = L"C:\\ReactOS\\inf";
    std::wstring package = L"D:\\radeon";

    std::map<std::wstring, std::map<std::wstring, Value>> registry;
    std::set<std::wstring> keys;
    std::map<std::wstring, Bytes> files;
    std::set<std::wstring> directories;
    std::set<std::wstring> locked;  // loaded images: rename/delete refused
    std::vector<std::pair<std::wstring, std::wstring>> pending_renames;
    std::vector<Device> devices;
    Service service;
    bool lightning_present = false;
    std::wstring active_opengl;
    std::wstring active_service;

    // Behaviour switches.
    bool install_clears_filters = false;  // Windows-style filter reset on install
    bool install_fails = false;
    bool install_binds_without_files = false;
    std::wstring redetect_service = L"vbemp";
    bool reactos_setupapi = false;  // see the module comment
    uint64_t boot = 1;  // restart() starts a new boot
    uint16_t machine = radeon::machine_i386;  // radeonctl architecture (SystemOps::machine)

    // Fault injection: the Nth mutating operation (1-based) and every later
    // operation fail without effect, as if the process died there.
    size_t crash_at = 0;
    bool torn_copy = false;
    std::wstring corrupt_copy;  // destination suffix whose copy flips one byte
    bool crashed = false;
    size_t mutations = 0;

    std::vector<std::string> output;
    bool echo = false;

    FakeSystem() {
        for (const auto* path : {L"c:", L"c:\\reactos", L"c:\\reactos\\system32", L"c:\\reactos\\system32\\drivers",
                                 L"c:\\reactos\\inf", L"d:", L"d:\\radeon"}) {
            directories.insert(path);
        }
    }

    radeon::SystemOps ops() {
        radeon::SystemOps value{};
        value.context = this;
        value.read_value = [](void* c, const wchar_t* k, const wchar_t* n, uint32_t& t, void* d, uint32_t cap,
                              uint32_t& b) { return self(c).read_value(k, n, t, d, cap, b); };
        value.write_value = [](void* c, const wchar_t* k, const wchar_t* n, uint32_t t, const void* d, uint32_t b) {
            return self(c).write_value(k, n, t, d, b);
        };
        value.delete_value = [](void* c, const wchar_t* k, const wchar_t* n) { return self(c).delete_value(k, n); };
        value.key_exists = [](void* c, const wchar_t* k, bool& e) { return self(c).key_exists(k, e); };
        value.delete_key = [](void* c, const wchar_t* k) { return self(c).delete_key(k); };
        value.file_size = [](void* c, const wchar_t* p, bool& e, uint64_t& b) { return self(c).file_size(p, e, b); };
        value.read_file = [](void* c, const wchar_t* p, uint64_t o, void* d, uint32_t cap, uint32_t& r) {
            return self(c).read_file(p, o, d, cap, r);
        };
        value.copy_file = [](void* c, const wchar_t* f, const wchar_t* t) { return self(c).copy_file(f, t); };
        value.move_file = [](void* c, const wchar_t* f, const wchar_t* t) { return self(c).move_file(f, t); };
        value.replace_on_restart = [](void* c, const wchar_t* f, const wchar_t* t) {
            return self(c).replace_on_restart(f, t);
        };
        value.delete_file = [](void* c, const wchar_t* p) { return self(c).delete_file(p); };
        value.create_directory = [](void* c, const wchar_t* p) { return self(c).create_directory(p); };
        value.query_service = [](void* c, radeon::ServiceInfo& s) { return self(c).query_service(s); };
        value.create_service = [](void* c) { return self(c).create_service(); };
        value.set_demand_start = [](void* c) { return self(c).set_demand_start(); };
        value.delete_service = [](void* c) { return self(c).delete_service(); };
        value.enumerate_devices = [](void* c, radeon::DeviceInventory& i) { return self(c).enumerate(i); };
        value.install_driver = [](void* c, const wchar_t* d, const wchar_t* h, const wchar_t* i, bool& r) {
            return self(c).install_driver(d, h, i, r);
        };
        value.remove_device = [](void* c, const wchar_t* i) { return self(c).remove_device(i); };
        value.unbind_device = [](void* c, const wchar_t* i, radeon::UnbindStep& f) {
            return self(c).unbind_device(i, f);
        };
        value.write_filters = [](void* c, const wchar_t* i, bool u, const wchar_t* l, size_t n) {
            return self(c).write_filters(i, u, l, n);
        };
        value.driver_value = [](void* c, const wchar_t* i, const wchar_t* n, wchar_t* o, size_t cap) {
            return self(c).driver_value(i, n, o, cap);
        };
        value.uninstall_oem_inf = [](void* c, const wchar_t* n, bool f) { return self(c).uninstall_oem_inf(n, f); };
        value.active_display = [](void* c, radeon::ActiveDisplay& a) { return self(c).active_display(a); };
        value.boot_id = [](void* c, uint64_t& b) {
            b = self(c).boot;
            return self(c).crashed ? error::gen_failure : error::success;
        };
        value.print = [](void* c, const char* line) { self(c).print(line); };
        value.system_directory = system.c_str();
        value.inf_directory = inf.c_str();
        value.package_directory = package.c_str();
        value.machine = machine;
        return value;
    }

    // ---- Helpers for tests ----
    std::wstring drivers() const { return system + L"\\drivers"; }
    std::wstring live(size_t component) const {
        const auto& file = radeon::component_files[component];
        return (file.destination == radeon::Destination::drivers ? drivers() : system) + L"\\" + file.filename;
    }
    std::wstring packaged(const wchar_t* name) const { return package + L"\\" + name; }
    std::wstring firmware(const char* name) const {
        std::wstring wide = drivers() + L"\\rx7900\\";
        for (size_t index = 0; name[index] != 0; ++index) {
            wide.push_back(static_cast<wchar_t>(name[index]));
        }
        return wide;
    }
    void put_file(const std::wstring& path, Bytes bytes) {
        files[fold(path.c_str())] = std::move(bytes);
        for (std::wstring directory = parent_of(fold(path.c_str())); !directory.empty();
             directory = parent_of(directory)) {
            directories.insert(directory);
        }
    }
    bool has_file(const std::wstring& path) const { return files.count(fold(path.c_str())) != 0; }
    const Bytes* file(const std::wstring& path) const {
        const auto found = files.find(fold(path.c_str()));
        return found == files.end() ? nullptr : &found->second;
    }
    const Value* value(const wchar_t* key, const wchar_t* name) const {
        const auto found = registry.find(fold(key));
        if (found == registry.end()) {
            return nullptr;
        }
        const auto item = found->second.find(fold(name));
        return item == found->second.end() ? nullptr : &item->second;
    }
    bool dword(const wchar_t* key, const wchar_t* name, uint32_t& output) const {
        const Value* found = value(key, name);
        if (found == nullptr || found->type != radeon::value_type::dword || found->data.size() != 4) {
            return false;
        }
        std::memcpy(&output, found->data.data(), 4);
        return true;
    }
    std::wstring string(const wchar_t* key, const wchar_t* name) const {
        const Value* found = value(key, name);
        if (found == nullptr || found->data.size() < sizeof(wchar_t)) {
            return L"<absent>";
        }
        std::wstring text(found->data.size() / sizeof(wchar_t), 0);
        std::memcpy(&text[0], found->data.data(), text.size() * sizeof(wchar_t));
        while (!text.empty() && text.back() == 0) {
            text.pop_back();
        }
        return text;
    }
    void set_dword(const wchar_t* key, const wchar_t* name, uint32_t number) {
        Value item{radeon::value_type::dword, Bytes(4)};
        std::memcpy(item.data.data(), &number, 4);
        keys.insert(fold(key));
        registry[fold(key)][fold(name)] = item;
    }
    void set_string(const wchar_t* key, const wchar_t* name, const std::wstring& text) {
        Value item{radeon::value_type::string, Bytes((text.size() + 1) * sizeof(wchar_t))};
        std::memcpy(item.data.data(), text.c_str(), item.data.size());
        keys.insert(fold(key));
        registry[fold(key)][fold(name)] = item;
    }
    Device& add_device(uint16_t device_id, const std::wstring& service_name = L"", bool started = false,
                       const std::wstring& location = L"4&1A2B3C4D&0&0018") {
        Device device{};
        wchar_t id[64]{};
        const wchar_t digits[] = L"0123456789ABCDEF";
        std::wstring hardware = L"PCI\\VEN_1002&DEV_";
        for (int shift = 12; shift >= 0; shift -= 4) {
            hardware.push_back(digits[(device_id >> shift) & 0xf]);
        }
        const std::wstring specific = hardware + L"&SUBSYS_471E1DA2&REV_C8";
        const std::wstring instance = specific + L"\\" + location;
        (void)id;
        radeon::text_copy(device.record.instance_id, instance.c_str());
        radeon::text_copy(device.record.hardware_id, specific.c_str());
        radeon::text_copy(device.record.service, service_name.c_str());
        device.record.device_id = device_id;
        device.record.revision = 0xc8;
        device.record.started = started;
        devices.push_back(device);
        return devices.back();
    }
    static void set_filters(radeon::DeviceRecord& record, bool upper, const std::vector<std::wstring>& names) {
        wchar_t* list = upper ? record.upper_filters : record.lower_filters;
        size_t& characters = upper ? record.upper_characters : record.lower_characters;
        size_t position = 0;
        for (const auto& name : names) {
            for (wchar_t value : name) {
                list[position++] = value;
            }
            list[position++] = 0;
        }
        if (names.empty()) {
            characters = 0;
            return;
        }
        list[position++] = 0;
        characters = position;
    }
    static std::vector<std::wstring> filters(const radeon::DeviceRecord& record, bool upper) {
        const wchar_t* list = upper ? record.upper_filters : record.lower_filters;
        const size_t characters = upper ? record.upper_characters : record.lower_characters;
        std::vector<std::wstring> names;
        size_t position = 0;
        while (position < characters && list[position] != 0) {
            std::wstring name;
            while (position < characters && list[position] != 0) {
                name.push_back(list[position++]);
            }
            names.push_back(name);
            ++position;
        }
        return names;
    }
    // ReactOS restart: delayed renames run, locks drop, services stop.
    void restart() {
        ++boot;
        for (const auto& rename : pending_renames) {
            const auto source = files.find(rename.first);
            if (rename.second.empty()) {
                if (source != files.end()) {
                    files.erase(source);
                }
                continue;
            }
            if (source != files.end()) {
                files[rename.second] = source->second;
                files.erase(rename.first);
            }
        }
        pending_renames.clear();
        locked.clear();
        service.running = false;
        if (service.exists && service.start == 0xffffffffu) {
            erase_service();
        }
    }
    // ReactOS re-detects devices removed by DIF_REMOVE with its default driver.
    // A device unbound in place keeps its devnode and filters; umpnpmgr installs
    // the best driver at boot, which is this package's INF again while a staged
    // copy of it remains (the binding then has no files and fails to load).
    void redetect() {
        const bool staged_inf_left = has_file(inf + L"\\oem7.inf");
        for (auto& device : devices) {
            if (!device.present) {
                device.present = true;
                radeon::text_copy(device.record.service, redetect_service.c_str());
                device.record.started = true;
                device.record.upper_characters = 0;
                device.record.lower_characters = 0;
            } else if (device.unbound && !device.class_values &&
                       (device.config_flags & rx7900::pnp_unbind::configflag_failed_install) == 0) {
                // umpnpmgr installs only devices without Class and FAILEDINSTALL.
                device.unbound = false;
                radeon::text_copy(device.record.service,
                                  staged_inf_left ? radeon::service_name : redetect_service.c_str());
                device.record.started = !staged_inf_left;
                device.inf_path = staged_inf_left ? L"oem7.inf" : L"";
                device.class_values = true;
            }
        }
    }
    Snapshot snapshot() const {
        Snapshot value{registry, keys, files, service, {}, {}};
        for (const auto& device : devices) {
            Bytes serialized;
            for (const auto& name : filters(device.record, true)) {
                for (wchar_t c : name) serialized.push_back(static_cast<uint8_t>(c));
                serialized.push_back(0);
            }
            serialized.push_back(1);
            for (const auto& name : filters(device.record, false)) {
                for (wchar_t c : name) serialized.push_back(static_cast<uint8_t>(c));
                serialized.push_back(0);
            }
            value.filters.emplace_back(device.record.instance_id, serialized);
            value.bindings.push_back(std::wstring(device.record.service) + (device.present ? L"+" : L"-"));
        }
        return value;
    }
    bool output_contains(const char* text) const {
        for (const auto& line : output) {
            if (line.find(text) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

private:
    static FakeSystem& self(void* context) { return *static_cast<FakeSystem*>(context); }

    bool fail_read() const { return crashed; }
    // True when this mutation must fail (crash point reached or passed).
    bool mutate() {
        if (crashed) {
            return true;
        }
        ++mutations;
        if (crash_at != 0 && mutations >= crash_at) {
            crashed = true;
            return true;
        }
        return false;
    }

    uint32_t read_value(const wchar_t* key, const wchar_t* name, uint32_t& type, void* data, uint32_t capacity,
                        uint32_t& bytes) {
        if (fail_read()) {
            return error::gen_failure;
        }
        if (keys.count(fold(key)) == 0) {
            return error::file_not_found;
        }
        const Value* found = value(key, name);
        if (found == nullptr) {
            return error::file_not_found;
        }
        type = found->type;
        bytes = static_cast<uint32_t>(found->data.size());
        if (found->data.size() > capacity) {
            return error::more_data;
        }
        if (!found->data.empty()) {
            std::memcpy(data, found->data.data(), found->data.size());
        }
        return error::success;
    }
    uint32_t write_value(const wchar_t* key, const wchar_t* name, uint32_t type, const void* data, uint32_t bytes) {
        if (mutate()) {
            return error::gen_failure;
        }
        Value item{type, Bytes(bytes)};
        if (bytes != 0) {
            std::memcpy(item.data.data(), data, bytes);
        }
        for (std::wstring path = fold(key); !path.empty(); path = parent_of(path)) {
            keys.insert(path);
        }
        registry[fold(key)][fold(name)] = item;
        return error::success;
    }
    uint32_t delete_value(const wchar_t* key, const wchar_t* name) {
        if (mutate()) {
            return error::gen_failure;
        }
        auto found = registry.find(fold(key));
        if (keys.count(fold(key)) == 0 || found == registry.end() || found->second.erase(fold(name)) == 0) {
            return error::file_not_found;
        }
        return error::success;
    }
    uint32_t key_exists(const wchar_t* key, bool& exists) {
        if (fail_read()) {
            return error::gen_failure;
        }
        exists = keys.count(fold(key)) != 0;
        return error::success;
    }
    uint32_t delete_key(const wchar_t* key) {
        if (mutate()) {
            return error::gen_failure;
        }
        const std::wstring folded = fold(key);
        if (keys.count(folded) == 0) {
            return error::file_not_found;
        }
        for (const auto& other : keys) {
            if (other.size() > folded.size() && other.compare(0, folded.size(), folded) == 0 &&
                other[folded.size()] == L'\\') {
                return error::access_denied;  // subkeys present
            }
        }
        keys.erase(folded);
        registry.erase(folded);
        return error::success;
    }
    uint32_t file_size(const wchar_t* path, bool& exists, uint64_t& bytes) {
        if (fail_read()) {
            return error::gen_failure;
        }
        const auto found = files.find(fold(path));
        exists = found != files.end();
        bytes = exists ? found->second.size() : 0;
        return error::success;
    }
    uint32_t read_file(const wchar_t* path, uint64_t offset, void* data, uint32_t capacity, uint32_t& received) {
        received = 0;
        if (fail_read()) {
            return error::gen_failure;
        }
        const auto found = files.find(fold(path));
        if (found == files.end()) {
            return error::file_not_found;
        }
        if (offset >= found->second.size()) {
            return error::success;
        }
        const size_t available = found->second.size() - static_cast<size_t>(offset);
        received = static_cast<uint32_t>(available < capacity ? available : capacity);
        std::memcpy(data, found->second.data() + offset, received);
        return error::success;
    }
    bool parent_exists(const std::wstring& path) const { return directories.count(parent_of(path)) != 0; }
    uint32_t copy_file(const wchar_t* from, const wchar_t* to) {
        const std::wstring source = fold(from);
        const std::wstring target = fold(to);
        const auto found = files.find(source);
        if (crashed) {
            return error::gen_failure;
        }
        if (found == files.end()) {
            return error::file_not_found;
        }
        if (!parent_exists(target)) {
            return error::path_not_found;
        }
        if (locked.count(target) != 0) {
            return error::sharing_violation;
        }
        const Bytes bytes = found->second;
        if (mutate()) {
            if (torn_copy) {
                files[target] = Bytes(bytes.begin(), bytes.begin() + static_cast<long>(bytes.size() / 2));
            }
            return error::gen_failure;
        }
        files[target] = bytes;
        const std::wstring suffix = fold(corrupt_copy.c_str());
        if (!suffix.empty() && target.size() >= suffix.size() &&
            target.compare(target.size() - suffix.size(), suffix.size(), suffix) == 0 && !bytes.empty()) {
            files[target][bytes.size() / 2] ^= 0x5a;
        }
        return error::success;
    }
    uint32_t move_file(const wchar_t* from, const wchar_t* to) {
        const std::wstring source = fold(from);
        const std::wstring target = fold(to);
        if (crashed) {
            return error::gen_failure;
        }
        if (files.count(source) == 0) {
            return error::file_not_found;
        }
        if (locked.count(source) != 0 || locked.count(target) != 0) {
            return error::sharing_violation;
        }
        if (!parent_exists(target)) {
            return error::path_not_found;
        }
        if (mutate()) {
            return error::gen_failure;
        }
        files[target] = files[source];
        files.erase(source);
        return error::success;
    }
    uint32_t replace_on_restart(const wchar_t* from, const wchar_t* to) {
        if (mutate()) {
            return error::gen_failure;
        }
        pending_renames.emplace_back(fold(from), to != nullptr ? fold(to) : std::wstring());
        return error::success;
    }
    uint32_t delete_file(const wchar_t* path) {
        const std::wstring target = fold(path);
        if (crashed) {
            return error::gen_failure;
        }
        if (files.count(target) == 0) {
            return error::file_not_found;
        }
        if (locked.count(target) != 0) {
            return error::access_denied;
        }
        if (mutate()) {
            return error::gen_failure;
        }
        files.erase(target);
        return error::success;
    }
    uint32_t create_directory(const wchar_t* path) {
        const std::wstring target = fold(path);
        if (directories.count(target) != 0) {
            return crashed ? error::gen_failure : error::success;
        }
        if (!parent_exists(target)) {
            return error::path_not_found;
        }
        if (mutate()) {
            return error::gen_failure;
        }
        directories.insert(target);
        return error::success;
    }
    uint32_t query_service(radeon::ServiceInfo& output) {
        output = radeon::ServiceInfo{};
        if (fail_read()) {
            return error::gen_failure;
        }
        if (!service.exists) {
            return error::success;
        }
        output.exists = true;
        output.type = service.type;
        output.start = service.start;
        output.error_control = service.error_control;
        output.running = service.running;
        radeon::text_copy(output.image_path, service.image.c_str());
        return error::success;
    }
    uint32_t create_service() {
        if (service.exists) {
            return crashed ? error::gen_failure : error::service_exists;
        }
        if (mutate()) {
            return error::gen_failure;
        }
        service = Service{true, 1, 3, 0, false, radeon::service_image_systemroot};
        return error::success;
    }
    uint32_t set_demand_start() {
        if (!service.exists) {
            return crashed ? error::gen_failure : error::service_does_not_exist;
        }
        if (mutate()) {
            return error::gen_failure;
        }
        service.start = 3;
        return error::success;
    }
    uint32_t delete_service() {
        if (!service.exists) {
            return crashed ? error::gen_failure : error::service_does_not_exist;
        }
        if (mutate()) {
            return error::gen_failure;
        }
        // A running service is only marked; it disappears at restart.
        if (service.running) {
            service.start = 0xffffffffu;
            return error::success;
        }
        erase_service();
        return error::success;
    }
    void erase_service() {
        service = Service{};
        for (const auto* key : {radeon::device0_key, radeon::video_subkey, radeon::service_key}) {
            registry.erase(fold(key));
            keys.erase(fold(key));
        }
    }
    uint32_t enumerate(radeon::DeviceInventory& output) {
        output = radeon::DeviceInventory{};
        if (fail_read()) {
            return error::gen_failure;
        }
        output.lightning_present = lightning_present;
        for (const auto& device : devices) {
            if (!device.present) {
                continue;
            }
            if (output.count == radeon::device_capacity) {
                ++output.omitted;
                continue;
            }
            output.devices[output.count++] = device.record;
        }
        return error::success;
    }
    void write_inf_values() {
        // radeondisplay.inf [ServiceSettings]: every opt-in 0.
        constexpr wchar_t drivers_list[] = L"radeondisp\0";
        Value list{radeon::value_type::multi_string, Bytes(sizeof(drivers_list))};
        std::memcpy(list.data.data(), drivers_list, sizeof(drivers_list));
        for (std::wstring path = fold(radeon::device0_key); !path.empty(); path = parent_of(path)) {
            keys.insert(path);
        }
        registry[fold(radeon::device0_key)][L"installeddisplaydrivers"] = list;
        set_dword(radeon::device0_key, L"VgaCompatible", 0);
        for (const auto& rule : radeon::feature_rules) {
            if (rule.registry_value != nullptr) {
                set_dword(radeon::device0_key, rule.registry_value, 0);
            }
        }
        set_string(radeon::video_subkey, L"Service", radeon::service_name);
    }
    uint32_t install_driver(const wchar_t* instance_id, const wchar_t* hardware_id, const wchar_t* inf_source,
                            bool& restart) {
        restart = false;
        if (mutate()) {
            return error::gen_failure;
        }
        if (install_fails || files.count(fold(inf_source)) == 0) {
            return error::file_not_found;
        }
        bool matched = false;
        for (auto& device : devices) {
            if (!device.present || !radeon::text_equal_insensitive(device.record.hardware_id, hardware_id) ||
                !radeon::text_equal_insensitive(device.record.instance_id, instance_id)) {
                continue;
            }
            matched = true;
            if (!install_binds_without_files) {
                files[fold(live(0).c_str())] = files[fold(packaged(radeon::component_files[0].filename).c_str())];
                files[fold(live(1).c_str())] = files[fold(packaged(radeon::component_files[1].filename).c_str())];
            }
            if (!service.exists) {
                service = Service{true, 1, 3, 0, false, L"System32\\drivers\\rx7900video.sys"};
            }
            write_inf_values();
            radeon::text_copy(device.record.service, radeon::service_name);
            device.record.started = false;
            device.unbound = false;
            // Driver key with the Settings subkey of the display class installer;
            // ReactOS leaves the failed-install marks of an earlier attempt.
            device.driver_key = L"{4D36E968-E325-11CE-BFC1-08002BE10318}\\0007";
            device.class_values = true;
            device.config_flags = reactos_setupapi ? rx7900::pnp_unbind::configflag_failed_install |
                                                         rx7900::pnp_unbind::configflag_reinstall : 0;
            const std::wstring software = std::wstring(rx7900::pnp_unbind::class_root) + device.driver_key;
            set_string(software.c_str(), L"InfPath", L"oem7.inf");
            keys.insert(fold((software + L"\\Settings").c_str()));
            if (install_clears_filters) {
                device.record.upper_characters = 0;
                device.record.lower_characters = 0;
            }
            device.inf_path = L"oem7.inf";
            files[fold((inf + L"\\oem7.inf").c_str())] = files[fold(inf_source)];
            restart = true;
        }
        return matched ? error::success : error::no_more_items;
    }
    Device* find(const wchar_t* instance) {
        for (auto& device : devices) {
            if (device.present && radeon::text_equal_insensitive(device.record.instance_id, instance)) {
                return &device;
            }
        }
        return nullptr;
    }
    uint32_t remove_device(const wchar_t* instance) {
        Device* device = find(instance);
        if (crashed) {
            return error::gen_failure;
        }
        if (device == nullptr) {
            return error::no_more_items;
        }
        if (!radeon::text_equal_insensitive(device->record.service, radeon::service_name)) {
            return error::access_denied;
        }
        if (reactos_setupapi) {
            return error::call_not_implemented;  // SetupDiRemoveDevice stub, nothing changed
        }
        if (mutate()) {
            return error::gen_failure;
        }
        // DIF_REMOVE deletes the devnode: binding, driver key and filters.
        if (!device->driver_key.empty()) {
            const std::wstring software = fold((std::wstring(rx7900::pnp_unbind::class_root) + device->driver_key).c_str());
            for (const auto& key : {software + L"\\settings", software}) {
                keys.erase(key);
                registry.erase(key);
            }
        }
        device->driver_key.clear();
        device->class_values = false;
        device->present = false;
        device->record.service[0] = 0;
        device->record.upper_characters = 0;
        device->record.lower_characters = 0;
        device->inf_path.clear();
        return error::success;
    }
    // The shared pnp_unbind algorithm over this model: Enum values live in
    // Device, the driver key in the fake registry. Every write is a mutation.
    struct EnumRegistry final {
        FakeSystem& fake;
        Device* device(const wchar_t* key) {
            const std::wstring folded = fold(key);
            const std::wstring root = fold(rx7900::pnp_unbind::enum_root);
            if (folded.compare(0, root.size(), root) != 0) {
                return nullptr;
            }
            for (auto& candidate : fake.devices) {
                if (candidate.present && fold(candidate.record.instance_id) == folded.substr(root.size())) {
                    return &candidate;
                }
            }
            return nullptr;
        }
        uint32_t read_string(const wchar_t* key, const wchar_t* name, wchar_t* output, size_t capacity) {
            Device* target = device(key);
            output[0] = 0;
            if (fake.crashed) {
                return error::gen_failure;
            }
            const std::wstring folded = fold(name);
            const std::wstring text = target == nullptr ? std::wstring() :
                                      folded == L"service" ? std::wstring(target->record.service) :
                                      folded == L"driver" ? target->driver_key : std::wstring();
            if (text.empty()) {
                return error::file_not_found;
            }
            if (text.size() + 1 > capacity) {
                return error::invalid_data;
            }
            std::memcpy(output, text.c_str(), (text.size() + 1) * sizeof(wchar_t));
            return error::success;
        }
        uint32_t read_dword(const wchar_t* key, const wchar_t* name, uint32_t& value) {
            Device* target = device(key);
            value = 0;
            if (fake.crashed) {
                return error::gen_failure;
            }
            if (target == nullptr || fold(name) != L"configflags") {
                return error::file_not_found;
            }
            value = target->config_flags;
            return error::success;
        }
        uint32_t write_dword(const wchar_t* key, const wchar_t* name, uint32_t value) {
            Device* target = device(key);
            if (fake.mutate()) {
                return error::gen_failure;
            }
            if (target == nullptr || fold(name) != L"configflags") {
                return error::access_denied;
            }
            target->config_flags = value;
            return error::success;
        }
        uint32_t delete_value(const wchar_t* key, const wchar_t* name) {
            Device* target = device(key);
            if (fake.mutate()) {
                return error::gen_failure;
            }
            const std::wstring folded = fold(name);
            bool existed = false;
            if (target != nullptr && folded == L"service") {
                existed = target->record.service[0] != 0;
                target->record.service[0] = 0;
                target->unbound = target->unbound || existed;
            } else if (target != nullptr && folded == L"driver") {
                existed = !target->driver_key.empty();
                target->driver_key.clear();
                target->inf_path.clear();
            } else if (target != nullptr && (folded == L"class" || folded == L"classguid")) {
                existed = target->class_values;
                target->class_values = false;
            } else if (target != nullptr && (folded == L"upperfilters" || folded == L"lowerfilters")) {
                size_t& characters = folded == L"upperfilters" ? target->record.upper_characters
                                                               : target->record.lower_characters;
                existed = characters != 0;
                characters = 0;
            }
            return existed ? error::success : error::file_not_found;
        }
        uint32_t delete_key(const wchar_t* key) {
            const uint32_t result = fake.delete_key(key);
            for (auto& candidate : fake.devices) {
                if (result == error::success && !candidate.driver_key.empty() &&
                    fold(key) == fold((std::wstring(rx7900::pnp_unbind::class_root) + candidate.driver_key).c_str())) {
                    candidate.inf_path.clear();  // InfPath lived in the driver key
                }
            }
            return result;
        }
        uint32_t flush(const wchar_t*) { return fake.crashed ? error::gen_failure : error::success; }
    };
    uint32_t unbind_device(const wchar_t* instance, radeon::UnbindStep& failed) {
        failed = radeon::UnbindStep::open_device;
        if (crashed) {
            return error::gen_failure;
        }
        if (find(instance) == nullptr) {
            return error::no_more_items;
        }
        EnumRegistry enum_registry{*this};
        return rx7900::unbind_in_place(enum_registry, instance, radeon::service_name, false, failed);
    }
    uint32_t write_filters(const wchar_t* instance, bool upper, const wchar_t* list, size_t characters) {
        Device* device = find(instance);
        if (crashed) {
            return error::gen_failure;
        }
        if (device == nullptr) {
            return error::no_more_items;
        }
        if (mutate()) {
            return error::gen_failure;
        }
        wchar_t* target = upper ? device->record.upper_filters : device->record.lower_filters;
        for (size_t index = 0; index < characters; ++index) {
            target[index] = list[index];
        }
        (upper ? device->record.upper_characters : device->record.lower_characters) = characters;
        return error::success;
    }
    uint32_t driver_value(const wchar_t* instance, const wchar_t* name, wchar_t* output, size_t capacity) {
        Device* device = find(instance);
        output[0] = 0;
        if (fail_read()) {
            return error::gen_failure;
        }
        if (device == nullptr || device->inf_path.empty() || !radeon::text_equal_insensitive(name, L"InfPath") ||
            device->inf_path.size() + 1 > capacity) {
            return error::file_not_found;
        }
        std::memcpy(output, device->inf_path.c_str(), (device->inf_path.size() + 1) * sizeof(wchar_t));
        return error::success;
    }
    uint32_t uninstall_oem_inf(const wchar_t* name, bool force) {
        if (mutate()) {
            return error::gen_failure;
        }
        if (reactos_setupapi && !force) {
            return error::success;  // FIXME "not deleting" in ReactOS setupapi misc.c
        }
        constexpr uint32_t inf_in_use_by_devices = 0xe000023du;  // ERROR_INF_IN_USE_BY_DEVICES
        for (const auto& device : devices) {
            if (!force && device.present && radeon::text_equal_insensitive(device.inf_path.c_str(), name)) {
                return inf_in_use_by_devices;
            }
        }
        return files.erase(fold((inf + L"\\" + name).c_str())) != 0 ? error::success : error::file_not_found;
    }
    uint32_t active_display(radeon::ActiveDisplay& output) {
        output = radeon::ActiveDisplay{};
        if (fail_read()) {
            return error::gen_failure;
        }
        radeon::text_copy(output.opengl_driver, active_opengl.c_str());
        radeon::text_copy(output.primary_service, active_service.c_str());
        if (reactos_setupapi) {
            // \REGISTRY\Machine\System\CONTROLSET001\SERVICES\<NAME>\Device0
            for (size_t index = 0; output.primary_service[index] != 0; ++index) {
                const wchar_t value = output.primary_service[index];
                output.primary_service[index] =
                    value >= L'a' && value <= L'z' ? static_cast<wchar_t>(value - 32) : value;
            }
        }
        return error::success;
    }
    void print(const char* line) {
        output.emplace_back(line);
        if (echo) {
            std::printf("  | %s\n", line);
        }
    }
};

} // namespace rx7900_test::radeon_fake
