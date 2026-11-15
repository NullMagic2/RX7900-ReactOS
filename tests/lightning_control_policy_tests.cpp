/*
Module: rx7900.tests.lightning_control_policy
Purpose: Check lightningnt5ctl's capability, primary-display and frontend decisions.
Created: 2026-10-25
Architecture: Portable; the exact production predicates without Win32 observations.
*/
#include "lightning_abi.hpp"
#include "lightning_control_policy.hpp"
#include <cstdio>
#include <cwchar>

namespace {
using namespace lightning::reactos;
using namespace lightning::tools;
unsigned checks=0;
unsigned failures=0;

void expect(bool condition,const char* description) {
    ++checks;
    if (!condition) {++failures; std::printf("FAIL %s\n",description);}
}

LightningCapabilities reply(uint32_t flags) {
    LightningCapabilities caps{};
    caps.size=sizeof(caps);
    caps.version=capabilities_version;
    caps.flags=flags;
    return caps;
}
} // namespace

int main() {
    // The filter's reply with a ready renderer (lightning_driver.cpp:
    // transport capabilities_observed | renderer exchange | exchange batch).
    const uint32_t ready=capabilities_observed|capability_renderer_exchange|capability_exchange_batch;
    expect(valid_capability_reply(reply(ready),sizeof(LightningCapabilities)),
        "the filter's ready-renderer reply (with exchange batching) validates");
    expect(valid_capability_reply(reply(capabilities_observed),sizeof(LightningCapabilities)),
        "a transport-only reply validates");
    expect(!valid_capability_reply(reply(ready|8U),sizeof(LightningCapabilities)),
        "an unknown capability bit is still a revision mismatch");
    expect(!valid_capability_reply(reply(capability_renderer_exchange),sizeof(LightningCapabilities)),
        "a reply without an observed host exchange is rejected");
    auto sized=reply(ready);
    sized.size=sizeof(sized)-4;
    expect(!valid_capability_reply(sized,sizeof(LightningCapabilities)),"a wrong self-described size is rejected");
    auto versioned=reply(ready);
    versioned.version=capabilities_version+1;
    expect(!valid_capability_reply(versioned,sizeof(LightningCapabilities)),"a newer reply version is rejected");
    expect(!valid_capability_reply(reply(ready),sizeof(LightningCapabilities)-1),"a short transfer is rejected");

    wchar_t service[64]{};
    expect(video_key_owner(L"\\REGISTRY\\Machine\\System\\CurrentControlSet\\Services\\BochsMP\\Device0",
        service,64)==VideoKeyForm::service && std::wcscmp(service,L"BochsMP")==0,
        "the default DEVICEMAP path names the owning miniport service");
    expect(video_key_owner(L"\\registry\\machine\\system\\currentcontrolset\\services\\LightningVideo\\device12",
        service,64)==VideoKeyForm::service && std::wcscmp(service,L"LightningVideo")==0,
        "service form parsing is case-insensitive and accepts any device number");
    expect(video_key_owner(L"\\Registry\\Machine\\System\\CurrentControlSet\\Control\\Video\\"
        L"{0A4E2B8C-2C1D-4F51-9D6B-3A0D4D3D4E11}\\0000",service,64)==VideoKeyForm::video_id && service[0]==0,
        "the UseNewKey form is recognized and names no service");
    expect(video_key_owner(L"\\Registry\\Machine\\System\\CurrentControlSet\\Services\\BochsMP",service,64)==
        VideoKeyForm::unknown && service[0]==0,"a path without a device suffix is unknown");
    expect(video_key_owner(L"\\Registry\\Machine\\System\\CurrentControlSet\\Services\\\\Device0",service,64)==
        VideoKeyForm::unknown,"an empty service name is unknown");
    expect(video_key_owner(L"\\Registry\\Machine\\System\\CurrentControlSet\\Services\\BochsMP\\Device0x",service,64)==
        VideoKeyForm::unknown,"a malformed device suffix is unknown");
    expect(video_key_owner(L"\\Registry\\Machine\\System\\CurrentControlSet\\Services\\BochsMP\\Device0",service,7)==
        VideoKeyForm::unknown && service[0]==0,"a service name that does not fit is refused");
    expect(video_key_owner(nullptr,service,64)==VideoKeyForm::unknown,"no path is unknown");

    expect(classify_primary(true,VideoKeyForm::service,L"lightningvideo",L"LightningVideo")==PrimaryOwner::lightning,
        "Lightning owns \\Device\\Video0");
    expect(classify_primary(true,VideoKeyForm::service,L"BochsMP",L"LightningVideo")==PrimaryOwner::other,
        "a different miniport owns \\Device\\Video0 (run 1: BochsMP)");
    expect(classify_primary(true,VideoKeyForm::video_id,L"",L"LightningVideo")==PrimaryOwner::unknown,
        "the new-key form cannot prove ownership");
    expect(classify_primary(false,VideoKeyForm::unknown,L"",L"LightningVideo")==PrimaryOwner::absent,
        "no \\Device\\Video0 entry is reported as absent");

    expect(competing_display_disable_admissible(L"BochsMP",L"LightningVideo",true,service_kernel_driver,3),
        "an explicitly named present display miniport may be disabled");
    expect(!competing_display_disable_admissible(L"lightningvideo",L"LightningVideo",true,service_kernel_driver,3),
        "the Lightning miniport itself is never disabled");
    expect(!competing_display_disable_admissible(L"Serial",L"LightningVideo",false,service_kernel_driver,3),
        "a service no present display device uses is refused");
    expect(!competing_display_disable_admissible(L"BochsMP",L"LightningVideo",true,service_kernel_driver,4),
        "an already disabled service is not recorded again");
    expect(!competing_display_disable_admissible(L"BochsMP",L"LightningVideo",true,0x10U,3),
        "a user-mode service is refused");
    expect(!competing_display_disable_admissible(L"",L"LightningVideo",true,service_kernel_driver,3) &&
        !competing_display_disable_admissible(nullptr,L"LightningVideo",true,service_kernel_driver,3),
        "an unnamed service is refused");
    expect(display_rollback_admissible(3,service_disabled) && display_rollback_admissible(1,service_disabled),
        "rollback restores the recorded start type of a still-disabled service");
    expect(!display_rollback_admissible(3,2),"rollback leaves a service someone re-enabled since untouched");
    expect(!display_rollback_admissible(service_disabled,service_disabled),"a corrupt record restores nothing");

    expect(frontend_order[0]==FrontendSource::registered && frontend_order[1]==FrontendSource::package &&
        frontend_order[2]==FrontendSource::sibling,
        "frontend-check prefers the registered ICD, then the package LightningGL copy");
    expect(std::wcscmp(frontend_package_relative,L"LightningGL\\QemuLightningOpenGL.dll")==0,
        "the package-relative frontend matches the display-install layout");

    std::printf("Lightning control policy: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}
