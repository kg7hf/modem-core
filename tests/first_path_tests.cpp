// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// first_path_tests - which path acquisition anchors on, on the article's recordings
// =============================================================================
// The Part 5 walkthrough's steps, each checked. The recordings' cases ([recordings])
// observe each recording exactly as first_path_demo does and check it against the
// expectations its manifest carries; they need the recordings, so they are registered
// only when the build is configured with MODEM_ARTICLE_DATA=ON, and a missing or
// unusable package fails them as a setup error. The sidelobe step needs no recording.
// Explained in the article series, Part 5 (the first path).
// =============================================================================

#include "tests/support/receive_chain.hpp"
#include "tests/support/test_waveform.hpp"
#include "tests/walkthrough/first_path_steps.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

using namespace modem_test::first_path_steps;

namespace
{

// The package, or a failed test that says why it is not usable.
Pack pack()
{
    std::string diagnostic;
    auto opened = open_pack(MODEM_FIRST_PATH_DATA, diagnostic);
    INFO("the recordings are not usable: " << diagnostic);
    REQUIRE(opened.has_value());
    return std::move(opened.value());
}

// One recording observed at its phases, checked against its manifest.
void meets_its_manifest(std::string_view id)
{
    const auto opened = pack();
    std::string diagnostic;
    const auto recording = observe_case(opened, id, diagnostic);
    INFO(diagnostic);
    REQUIRE(recording.has_value());
    const auto failures = unmet(recording.value());
    for (const auto& failure : failures)
    {
        UNSCOPED_INFO(failure);
    }
    CHECK(failures.empty());
}

} // namespace

TEST_CASE("The model with the previous segment's tail leaves nothing of a clean path's profile; the prefix alone leaves the most at lag 14", "[first_path][sidelobes][walkthrough]")
{
    const auto step = sidelobe_model();
    REQUIRE(step.has_value());
    const auto& rows = step.value().rows;
    REQUIRE(rows.size() == 15U);
    for (const auto& row : rows)
    {
        CHECK(row.with_tail < 0.001F);
        CHECK(row.prefix_only <= row.measured);
        CHECK(row.measured < 0.125F); // a clean path's own sidelobes stay under the 1/8 floor
    }
    CHECK(std::ranges::max_element(rows, {}, &SidelobeRow::prefix_only)->lag == 14U);
}

TEST_CASE("On one clean path observe() puts the body on the direct path at every arrival phase and the anchoring moves nothing", "[first_path][observe]")
{
    // The "Hi" burst straight from the transmitter, after half a second of silence: by the
    // bench's convention its first symbol is centred on its first sample, so the timeline is
    // known exactly without a recording, and no second path is there to find.
    modem_test::WaveformSpec spec;
    spec.payload = {'H', 'i'};
    const auto made = modem_test::make_test_waveform(spec);
    REQUIRE(made.has_value());
    constexpr std::size_t lead = 24000U;
    std::vector<float> audio(lead, 0.0F);
    audio.insert(audio.end(), made.value().audio.begin(), made.value().audio.end());
    const Timeline timeline{.direct_symbol0_sample = 24000.0, .echo_delay_samples = 240.0, .preamble_symbols = made.value().bursts.front().preamble_symbols};
    const double body = 24000.0 + 20.0 * 11520.0;

    for (const std::size_t phase : {0U, 7U, 19U})
    {
        CAPTURE(phase);
        const auto seen = observe(audio, timeline, phase);
        REQUIRE(seen.has_value());
        REQUIRE(seen.value().groups.size() == 1U);
        CHECK(seen.value().groups.front().path == Path::direct);
        for (const auto* profile : {&seen.value().off, &seen.value().window_7, &seen.value().window_3})
        {
            CHECK(profile->anchor == Path::direct);
            CHECK(profile->moved_by == Move::none);
            CHECK(std::abs(profile->body_sample - body) < 1.0); // the timing conversion, to a sample
        }
        CHECK(seen.value().window_7.diagnostics.precursor_candidates == 0U);
    }
}

TEST_CASE("An unusable package is a setup error that says why", "[first_path][manifest]")
{
    std::string diagnostic;
    const auto opened = open_pack("no-such-folder/first-path-v1", diagnostic);
    CHECK_FALSE(opened.has_value());
    CHECK(diagnostic.find("VERSION") != std::string::npos);
}

TEST_CASE("fp1: two groups with the echo's the strongest; the anchoring moves to the direct path's group", "[first_path][recordings]")
{
    meets_its_manifest("fp1-echo-group");
}

TEST_CASE("fp2: at 2 ms the window from 7 stays on the echo and the window from 3 moves to the direct path", "[first_path][recordings]")
{
    meets_its_manifest("fp2-window");
}

TEST_CASE("fp3: a direct path with no group of its own is found by the precursor search after the walk rejects a segment", "[first_path][recordings]")
{
    meets_its_manifest("fp3-faded-direct");
}

TEST_CASE("fp4: with the short preamble the weak direct path is reached at some arrival phases but not all", "[first_path][recordings]")
{
    meets_its_manifest("fp4-phase-short");
}

TEST_CASE("fp5: a later path's sidelobe peaks 11 symbols ahead and the walk rejects it", "[first_path][recordings]")
{
    meets_its_manifest("fp5-sidelobe-path");
}

TEST_CASE("fp6: with the long preamble the same weak path is reached at every arrival phase", "[first_path][recordings]")
{
    meets_its_manifest("fp6-phase-long");
}

TEST_CASE("The runner's Part 5 line on fp1: the strongest group 12 symbols late and the anchoring moving it to the direct path", "[first_path][recordings][runner]")
{
    const auto opened = pack();
    modem_test::ChainSpec spec;
    spec.wav_in = opened.root + "/" + opened.cases.find("fp1-echo-group")->audio;
    const auto report = modem_test::run_chain(spec);
    REQUIRE(report.has_value());
    print(report.value());
    REQUIRE(report.value().first_path.has_value());
    const auto& path = *report.value().first_path;
    REQUIRE(path.strongest_body.has_value());
    CHECK(*path.strongest_body == path.anchored_body + 12U);
    CHECK_FALSE(path.precursor_search);
}
