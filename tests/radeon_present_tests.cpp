/*
Module: rx7900.tests.radeon_present
Purpose: Check radeondisp's RadeonGL process-escape admission and clipped present batching.
Created: 2026-10-23
Architecture: Runs the exact radeon_present.hpp policy with modelled CLIPOBJ kinds and IOCTLs.
*/
#include <stdio.h>
#include <vector>

#include "radeon_present.hpp"

namespace {

namespace present = rx7900::reactos::radeon_present;
namespace gabi = rx7900::reactos::graphics_abi;
namespace pabi = rx7900::reactos::process_abi;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", contract);
    }
}

struct Sent final {
    std::vector<std::vector<gabi::PresentRect>> batches;
    int32_t window_left = 0;
    int32_t window_top = 0;
    uint32_t fail_ioctl_at = UINT32_MAX;
    uint32_t outcome_at = UINT32_MAX;
    uint32_t outcome = 0;
};

present::Result run(Sent& sent, present::ClipKind kind, present::Rect window, present::Rect bounds,
                    const std::vector<present::Rect>& complex, bool enum_ok = true) {
    static gabi::PresentRequest request;
    gabi::PresentInput input{gabi::present_input_bytes, gabi::version, 7, 9, 64, 64};
    const present::Rect screen{0, 0, 1920, 1080};
    auto enumerate = [&](auto&& visit) {
        for (const auto& r : complex) visit(r);
        return enum_ok;
    };
    auto send = [&](const gabi::PresentRequest& r, uint32_t& outcome) {
        const uint32_t index = static_cast<uint32_t>(sent.batches.size());
        if (index == sent.fail_ioctl_at) return false;
        sent.batches.emplace_back(r.rects, r.rects + r.rect_count);
        sent.window_left = r.window_left;
        sent.window_top = r.window_top;
        expect(r.input.session == 7 && r.input.buffer == 9 && r.reserved == 0, "request carries the input unchanged");
        uint8_t bytes[gabi::present_request_bytes];
        expect(gabi::encode_present_request(r, bytes, sizeof(bytes)), "every batch encodes");
        outcome = index == sent.outcome_at ? sent.outcome : 0;
        return true;
    };
    return present::present(request, input, window, screen, kind, bounds, enumerate, send);
}

} // namespace

int main() {
    using present::process_escape_admitted;
    expect(process_escape_admitted(64, 48, true, true) &&
               process_escape_admitted(64 + pabi::bulk_bytes, 48 + pabi::bulk_bytes, true, true),
           "process escape admits header-only and full bulk payloads");
    expect(!process_escape_admitted(63, 48, true, true) && !process_escape_admitted(64, 47, true, true) &&
               !process_escape_admitted(65 + pabi::bulk_bytes, 48, true, true) &&
               !process_escape_admitted(64, 49 + pabi::bulk_bytes, true, true) &&
               !process_escape_admitted(64, 48, false, true) && !process_escape_admitted(64, 48, true, false),
           "process escape refuses short, oversized and null buffers");
    expect(present::process_reply_valid(true, 48) && !present::process_reply_valid(true, 47) &&
               !present::process_reply_valid(false, 48),
           "process reply needs a successful IOCTL and a full header");

    const present::Rect window{100, 200, 300, 400};
    {
        Sent sent;
        expect(run(sent, present::ClipKind::missing, window, {}, {}).value == 0 && sent.batches.empty(),
               "missing clip is unsupported (readback path)");
        expect(run(sent, present::ClipKind::unknown, window, {}, {}).value == 0 && sent.batches.empty(),
               "unknown clip complexity is unsupported");
        expect(run(sent, present::ClipKind::empty, window, {}, {}).value == 1 && sent.batches.empty(),
               "DC_TRIVIAL (empty bounds in this engine) presents nothing and succeeds");
        expect(run(sent, present::ClipKind::rect, {2000, 0, 2100, 50}, {2000, 0, 2100, 50}, {}).value == 1 &&
                   sent.batches.empty(),
               "an off-screen window presents nothing");
    }
    {
        Sent sent;
        const auto result = run(sent, present::ClipKind::rect, {-50, 1000, 150, 1200}, {-100, 900, 120, 2000}, {});
        expect(result.value == 1 && sent.batches.size() == 1 && sent.batches[0].size() == 1,
               "DC_RECT sends one rectangle");
        const auto& r = sent.batches[0][0];
        expect(r.left == 0 && r.top == 1000 && r.right == 120 && r.bottom == 1080 && sent.window_left == -50 &&
                   sent.window_top == 1000,
               "rectangle is clip ∩ window ∩ surface, window origin unclipped");
    }
    {
        std::vector<present::Rect> rects;
        for (int i = 0; i < 140; ++i) {
            rects.push_back({100 + i, 200, 101 + i, 210});
            if (i % 10 == 0) rects.push_back({0, 0, 50, 50});   // outside the window: skipped
        }
        Sent sent;
        const auto result = run(sent, present::ClipKind::complex, window, {0, 0, 1920, 1080}, rects);
        expect(result.value == 1 && result.batches == 3 && result.rects == 140 && sent.batches[0].size() == 64 &&
                   sent.batches[1].size() == 64 && sent.batches[2].size() == 12,
               "DC_COMPLEX batches at most 64 rectangles, skipping empties");
        bool ordered = true;
        int expected_left = 100;
        for (const auto& batch : sent.batches) {
            for (const auto& r : batch) ordered = ordered && r.left == expected_left++;
        }
        expect(ordered, "rectangles keep enumeration order across batches");
    }
    {
        std::vector<present::Rect> rects(130, present::Rect{110, 210, 120, 220});
        Sent sent;
        sent.outcome_at = 1;
        sent.outcome = static_cast<uint32_t>(pabi::Outcome::busy);
        const auto result = run(sent, present::ClipKind::complex, window, {}, rects);
        expect(result.value == 1 + static_cast<uint32_t>(pabi::Outcome::busy) && sent.batches.size() == 3,
               "first non-success outcome is returned as 1 + outcome; later batches still go");
        Sent broken;
        broken.fail_ioctl_at = 1;
        expect(run(broken, present::ClipKind::complex, window, {}, rects).value == 0 && broken.batches.size() == 1,
               "an IOCTL failure reports unsupported so the readback path repaints exactly");
        Sent enumeration;
        expect(run(enumeration, present::ClipKind::complex, window, {}, rects, false).value == 0,
               "an enumeration failure reports unsupported");
    }
    printf("radeon_present: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
