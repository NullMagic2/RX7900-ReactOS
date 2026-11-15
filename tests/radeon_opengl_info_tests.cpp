/*
Module: rx7900.tests.radeon_opengl_info
Purpose: Check the OPENGL_GETINFO reply radeondisp gives ReactOS opengl32.
Created: 2026-10-23
Architecture: Pure header checks against the icdload.c Drv_Opengl_Info contract.
*/
#include <stdio.h>
#include <string.h>

#include "radeon_opengl_info.hpp"

namespace {

namespace gl = rx7900::reactos::radeon_opengl;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", contract);
    }
}

} // namespace

int main() {
    const uint32_t getinfo = gl::opengl_getinfo;
    const uint32_t other = 4352;
    expect(gl::supports(&getinfo, sizeof(getinfo)), "QUERYESCSUPPORT admits OPENGL_GETINFO");
    expect(!gl::supports(&other, sizeof(other)) && !gl::supports(&getinfo, 2) && !gl::supports(nullptr, 4),
           "QUERYESCSUPPORT refuses other codes, short input and null");

    gl::OpenglInfo reply{};
    memset(&reply, 0xcc, sizeof(reply));
    const uint32_t selector = gl::getinfo_driver_name;
    expect(gl::encode_info(&selector, sizeof(selector), &reply, sizeof(reply)) == sizeof(reply) &&
               reply.version == 1 && reply.driver_version == 1,
           "OPENGL_GETINFO returns versions equal to the OpenGLDrivers registry values");
    const char name[] = "RadeonGL";
    bool same = true;
    for (size_t index = 0; index < sizeof(name); ++index) {
        same = same && reply.driver_name[index] == static_cast<uint16_t>(static_cast<unsigned char>(name[index]));
    }
    expect(same && reply.driver_name[gl::max_path] == 0, "driver name is NUL-terminated UTF-16 RadeonGL");

    gl::OpenglInfo untouched{};
    memset(&untouched, 0x5a, sizeof(untouched));
    const uint32_t wrong = 1;
    expect(gl::encode_info(&wrong, sizeof(wrong), &untouched, sizeof(untouched)) == 0 &&
               gl::encode_info(&selector, sizeof(selector), &untouched, sizeof(untouched) - 1) == 0 &&
               gl::encode_info(&selector, 2, &untouched, sizeof(untouched)) == 0 &&
               untouched.version == 0x5a5a5a5aU,
           "unknown selector, short output or short input write nothing");
    expect(sizeof(gl::OpenglInfo) == 532, "Drv_Opengl_Info size on i386/AMD64");

    printf("radeon_opengl_info: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
