// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// first_path_manifest_tests - the recordings' manifest reader, on good and bad input
// =============================================================================
// cases.json tells the Part 5 walkthrough what each recording is and what the receiver
// must do with it, so a manifest the reader misreads would pass or fail tests for the
// wrong reason. These check that a good manifest reads exactly, and that each way a
// manifest can be wrong is refused with a diagnostic naming the field, never a default.
// No recordings are needed.
// =============================================================================

#include "tests/support/manifest_json.hpp"
#include "tests/walkthrough/first_path_cases.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <format>
#include <string>
#include <string_view>

using namespace modem_test::first_path;

namespace
{

std::string timeline(std::size_t preamble_symbols, double echo_delay)
{
    const double direct = 24174.0;
    const double body = direct + 20.0 * static_cast<double>(preamble_symbols);
    return std::format(R"({{"lead_samples": 24000, "tail_samples": 24000, "chain_delay_samples": 174.0, "direct_start_sample": 24174.0,
 "tx_symbol0_center_offset_samples": 0.0, "direct_symbol0_sample": {0}, "echo_delay_samples": {1}, "echo_start_sample": {2},
 "echo_symbol0_sample": {2}, "preamble_symbols": {3}, "direct_body_symbol0_sample": {4}, "echo_body_symbol0_sample": {5},
 "uncertainty_samples": 0.01, "calibration_id": "CAL-1", "calibration_method": "one static path, no noise", "operations": ["lead", "resample"]}})",
                       direct, echo_delay, direct + echo_delay, preamble_symbols, body, body + echo_delay);
}

std::string one_case(std::string_view id, std::string_view mode, std::size_t preamble_symbols, double echo_delay, std::string_view phases, std::string_view expectations)
{
    return std::format(R"({{"id": "{0}", "purpose": "a selected teaching case", "article_steps": [1], "audio": "{0}.wav",
 "clean_reference": "clean/hi-{1}.wav", "provenance": "{0}.json", "mode": "{1}", "payload_hex": "4869",
 "channel": {{"model": "pathsim"}}, "timeline": {2}, "phase_offsets_samples": {3}, "expectations": {4}}})",
                       id, mode, timeline(preamble_symbols, echo_delay), phases, expectations);
}

constexpr std::string_view all_phases = "[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]";

// A manifest the reader must accept: all six cases, as the specification spells them.
std::string good_manifest()
{
    std::string cases;
    cases += one_case("fp1-echo-group", "300L", 11520U, 240.0, "[0]",
                      R"({"groups_present": ["direct", "echo"], "strongest_group": "echo", "strongest_margin_supporters_min": 1,
 "anchor": {"off": "echo", "window_7": "direct", "window_3": "direct"}, "moved_by": "earlier_group"})");
    cases += ",\n" + one_case("fp2-window", "600L", 11520U, 96.0, "[0]",
                              R"({"strongest_group": "echo", "strongest_margin_supporters_min": 1, "anchor": {"off": "echo", "window_7": "echo", "window_3": "direct"}})");
    cases += ",\n" + one_case("fp3-faded-direct", "300L", 11520U, 240.0, "[0]",
                              R"({"groups_absent": ["direct"], "anchor": {"off": "echo", "window_7": "direct"}, "moved_by": "precursor_search",
 "reanchored_lag": 12, "precursor_fraction_min": 0.25, "walk_segments_min": 2, "walk_rejected_total_min": 1, "accepted_correlation_min": 0.45})");
    cases += ",\n" + one_case("fp4-phase-short", "600S", 1440U, 96.0, all_phases,
                              R"({"profile": "window_3", "direct_count_min": 4, "direct_count_max": 16, "others": "echo", "phase_vector": "EEEEEEEDDDDDDDDDDEEE",
 "walk_segments": [1, 1, 1, 1, 1, 1, 1, 6, 1, 1, 1, 1, 1, 9, 9, 3, 3, 3, 3, 3]})");
    cases += ",\n" + one_case("fp5-sidelobe-path", "300L", 11520U, 240.0, "[0]",
                              R"({"anchor": {"off": "direct", "window_7": "direct"}, "precursor_lag": 11, "precursor_fraction_min": 0.125,
 "walk_accepted": false, "rejected_correlation_max": 0.25})");
    cases += ",\n" + one_case("fp6-phase-long", "600L", 11520U, 96.0, all_phases,
                              R"({"profile": "window_3", "direct_count": 20, "phase_vector": "DDDDDDDDDDDDDDDDDDDD",
 "walk_segments": [2, 2, 3, 3, 8, 69, 5, 5, 5, 6, 18, 18, 10, 72, 72, 4, 21, 6, 66, 66]})");
    return std::format(R"({{"schema": "modem.first-path-cases/1", "package_id": "first-path-v1", "cases": [
{}
]}})",
                       cases);
}

// The manifest with its first `from` replaced by `to`.
std::string changed(std::string_view from, std::string_view to)
{
    auto text = good_manifest();
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    return text;
}

// Refused, and the diagnostic names `what`.
void refused(const std::string& text, std::string_view what)
{
    std::string diagnostic;
    const auto set = parse_cases(text, diagnostic);
    CHECK_FALSE(set.has_value());
    INFO(diagnostic);
    CHECK(diagnostic.find(what) != std::string::npos);
}

} // namespace

TEST_CASE("A good manifest reads exactly", "[first_path][manifest]")
{
    std::string diagnostic;
    const auto set = parse_cases(good_manifest(), diagnostic);
    INFO(diagnostic);
    REQUIRE(set.has_value());
    REQUIRE(set.value().cases.size() == 6U);
    const auto* fp3 = set.value().find("fp3-faded-direct");
    REQUIRE(fp3 != nullptr);
    CHECK(fp3->audio == "fp3-faded-direct.wav");
    CHECK(fp3->timeline.direct_symbol0_sample == 24174.0);
    CHECK(fp3->timeline.echo_body_symbol0_sample == 24174.0 + 20.0 * 11520.0 + 240.0);
    CHECK(fp3->expect.moved_by == Move::precursor_search);
    CHECK(fp3->expect.reanchored_lag == 12U);
    CHECK(fp3->expect.walk_rejected_total_min == 1U);
    CHECK(fp3->expect.anchor_off == Path::echo);
    CHECK_FALSE(fp3->expect.anchor_window_3.has_value());
    const auto* fp6 = set.value().find("fp6-phase-long");
    REQUIRE(fp6 != nullptr);
    CHECK(fp6->phase_offsets.size() == 20U);
    CHECK(fp6->expect.walk_segments.size() == 20U);
    CHECK(fp6->expect.phase_vector == "DDDDDDDDDDDDDDDDDDDD");
    const auto* fp5 = set.value().find("fp5-sidelobe-path");
    REQUIRE(fp5 != nullptr);
    CHECK(fp5->expect.walk_accepted == false);
}

TEST_CASE("A missing field or one of the wrong type is refused by name", "[first_path][manifest]")
{
    refused(changed(R"("payload_hex": "4869",)", ""), "\"payload_hex\" is missing");
    refused(changed(R"("reanchored_lag": 12)", R"("reanchored_lag": "12")"), "reanchored_lag");
    refused(changed(R"("strongest_group": "echo", "strongest_margin_supporters_min": 1,
 "anchor")",
                    R"("strongest_margin_supporters_min": 1,
 "anchor")"),
            "\"strongest_group\" is missing");
    refused(changed(R"("lead_samples": 24000)", R"("lead_samples": 24000.5)"), "lead_samples");
}

TEST_CASE("Duplicate keys and truncated input and stray content are refused", "[first_path][manifest]")
{
    refused(changed(R"("package_id": "first-path-v1")", R"("package_id": "first-path-v1", "package_id": "first-path-v1")"), "duplicate key");
    const auto text = good_manifest();
    refused(text.substr(0U, text.size() / 2U), "line");
    refused(text + " x", "content after the top-level value");
}

TEST_CASE("Numbers outside JSON's grammar or not finite are refused", "[first_path][manifest]")
{
    refused(changed(R"("chain_delay_samples": 174.0)", R"("chain_delay_samples": 0174.0)"), "leading zero");
    refused(changed(R"("chain_delay_samples": 174.0)", R"("chain_delay_samples": 174.)"), "fraction needs a digit");
    refused(changed(R"("chain_delay_samples": 174.0)", R"("chain_delay_samples": 1e999)"), "out of range");
    refused(changed(R"("chain_delay_samples": 174.0)", R"("chain_delay_samples": NaN)"), "not a value");
}

TEST_CASE("An unsupported schema version and unknown or repeated or missing cases are refused", "[first_path][manifest]")
{
    refused(changed("modem.first-path-cases/1", "modem.first-path-cases/2"), "unsupported version");
    refused(changed(R"("id": "fp2-window")", R"("id": "fp9-other")"), "not a case of this package");
    refused(changed(R"("id": "fp2-window")", R"("id": "fp1-echo-group")"), "appears twice");
    refused(changed(R"("id": "fp2-window")", R"("id": "fp5-sidelobe-path")"), "expected 300L"); // a case's mode is its own
}

TEST_CASE("Escapes and characters outside the accepted form are refused", "[first_path][manifest]")
{
    refused(changed("a selected teaching case", "a selected \\u0074eaching case"), "\\u escapes");
    refused(changed("a selected teaching case", "a selected t\xC3\xA9"
                                                "aching case"),
            "printable ASCII");
    std::string diagnostic;
    CHECK_FALSE(modem_test::manifest::parse(std::string(40U, '[') + std::string(40U, ']'), diagnostic).has_value());
    CHECK(diagnostic.find("16 deep") != std::string::npos);
    CHECK(modem_test::manifest::parse(R"({"a": "tab\tand \"quote\" and \\ and \/"})", diagnostic).has_value());
}

TEST_CASE("A timeline that breaks its own equations or a wrong mode or a path out of the package is refused", "[first_path][manifest]")
{
    refused(changed(R"("direct_start_sample": 24174.0)", R"("direct_start_sample": 24175.0)"), "disagrees with the timeline's equation");
    refused(changed(R"("mode": "600L")", R"("mode": "600S")"), "mode");
    refused(changed(R"("audio": "fp1-echo-group.wav")", R"("audio": "../fp1-echo-group.wav")"), "not a relative path inside the package");
    refused(changed(R"("audio": "fp1-echo-group.wav")", R"("audio": "C:/fp1.wav")"), "not a relative path inside the package");
    refused(changed(R"("phase_vector": "EEEEEEEDDDDDDDDDDEEE")", R"("phase_vector": "EEEEEEEDDDDDDDDDDEEX")"), "20 letters");
    refused(changed(R"("moved_by": "earlier_group")", R"("moved_by": "earlier_group", "extra": 1)"), "\"extra\" is not a field");
    refused(changed(R"("calibration_id": "CAL-1")", R"("calibration_id": 1)"), "calibration_id");
}
