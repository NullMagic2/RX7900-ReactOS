# Module: rx7900.build.test_references
# Purpose: Provide the pinned upstream register references six host tests read.
# Created: 2026-11-15
# Architecture: Standalone test builds only. The files are AMD's GC 11.0.0 and
# DCN 3.2 register headers from Linux v6.12 and Mesa 24.2.0's register JSON,
# identified by the SHA-256 values recorded in licenses/OPENGL.txt and
# src/display/dcn32_stream_registers.hpp. They are downloaded once into
# Binaries/upstream (not tracked) and never compiled into a driver.

set(rx7900_linux "https://raw.githubusercontent.com/torvalds/linux/v6.12/drivers/gpu/drm/amd/include/asic_reg")
set(rx7900_mesa "https://gitlab.freedesktop.org/mesa/mesa/-/raw/mesa-24.2.0/src/amd/registers")
set(rx7900_references
    "gpuinit|gc_11_0_0_offset.h|${rx7900_linux}/gc/gc_11_0_0_offset.h|a2394677509a3415a9e6307d876cae9cd654a99275ab40b3027d983da26801f2"
    "gpuinit|gc_11_0_0_sh_mask.h|${rx7900_linux}/gc/gc_11_0_0_sh_mask.h|83dd0060886da2927e3f722b992af208e4d41999b0740123a770a6561f060c94"
    "gpuinit|mesa-24.2.0/gfx11.json|${rx7900_mesa}/gfx11.json|304b57f399534febbd2b055067b81c2652c700eaea88526a7d10c6bb889bf2fb"
    "gpuinit|mesa-24.2.0/pkt3.json|${rx7900_mesa}/pkt3.json|36f5c2eba891df4ba1f85807241d536e8e4cc6787e34328971f071f4cddf0242"
    "dcn-stream|dcn_3_2_0_offset.h|${rx7900_linux}/dcn/dcn_3_2_0_offset.h|1e693846c3c95226fb7daaa7b80822edb240da6513b331d8622656f56f5a425e"
    "dcn-stream|dcn_3_2_0_sh_mask.h|${rx7900_linux}/dcn/dcn_3_2_0_sh_mask.h|f8554c0061f1b413a293cc15b2ea83e4e905b93cb093280411c0db38744b7ab6")

# Fills root/gpuinit and root/dcn-stream, each with the SHA256SUMS the tests read.
function(rx7900_fetch_test_references root)
    set(sums_gpuinit "")
    set(sums_dcn-stream "")
    foreach(entry IN LISTS rx7900_references)
        string(REPLACE "|" ";" fields "${entry}")
        list(GET fields 0 group)
        list(GET fields 1 name)
        list(GET fields 2 url)
        list(GET fields 3 hash)
        set(path "${root}/${group}/${name}")
        set(actual "")
        if(EXISTS "${path}")
            file(SHA256 "${path}" actual)
        endif()
        if(NOT actual STREQUAL hash)
            message(STATUS "Fetching test reference ${name}")
            file(DOWNLOAD "${url}" "${path}" EXPECTED_HASH "SHA256=${hash}" STATUS result TLS_VERIFY ON)
            list(GET result 0 code)
            if(NOT code EQUAL 0)
                file(REMOVE "${path}")
                message(FATAL_ERROR "Could not fetch ${url}: ${result}. "
                    "Set RX7900_FETCH_TEST_REFERENCES=OFF to build without it; six tests then fail.")
            endif()
        endif()
        string(APPEND sums_${group} "${hash}  ${name}\n")
    endforeach()
    file(WRITE "${root}/gpuinit/SHA256SUMS" "${sums_gpuinit}")
    file(WRITE "${root}/dcn-stream/SHA256SUMS" "${sums_dcn-stream}")
endfunction()
