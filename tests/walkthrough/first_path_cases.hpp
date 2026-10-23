// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "tests/walkthrough/first_path_observe.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// =============================================================================
// first_path_cases.hpp - the Part 5 recordings' manifest, read and checked
// =============================================================================
// cases.json (schema modem.first-path-cases/1, package first-path-v1) says, for each
// recording: its files, mode and payload, where its two paths are (the timeline,
// from how it was made), the arrival phases to run, and what the receiver must do.
// read_cases() reads it through the bench's manifest reader and checks every field
// the walkthrough uses: present, of the right type, in range, and consistent with
// the timeline's own equations. Anything else is a setup error naming the field.
// The provenance sidecars are for inspection; the walkthrough does not read them.
// =============================================================================

namespace modem_test::first_path
{

// The timeline, in recording samples at 48 kHz, as the Stage 1 contract defines it.
struct CaseTimeline
{
    std::size_t lead_samples{};
    std::size_t tail_samples{};
    double chain_delay_samples{};
    double direct_start_sample{};
    double tx_symbol0_center_offset_samples{};
    double direct_symbol0_sample{};
    double echo_delay_samples{};
    double echo_start_sample{};
    double echo_symbol0_sample{};
    std::size_t preamble_symbols{};
    double direct_body_symbol0_sample{};
    double echo_body_symbol0_sample{};
    double uncertainty_samples{};
    // calibration_id, calibration_method and operations are checked for type, not kept.
};

// What the receiver must do with the recording. Only the fields the case's spelling
// names are present.
struct CaseExpectations
{
    std::vector<Path> groups_present{};
    std::vector<Path> groups_absent{};
    std::optional<Path> strongest_group{};
    std::optional<std::size_t> strongest_margin_supporters_min{};
    std::optional<Path> anchor_off{};
    std::optional<Path> anchor_window_7{};
    std::optional<Path> anchor_window_3{};
    std::optional<Move> moved_by{};
    std::optional<std::size_t> reanchored_lag{};
    std::optional<std::size_t> precursor_lag{};
    std::optional<double> precursor_fraction_min{};
    std::optional<std::size_t> walk_segments_min{};
    std::optional<std::size_t> walk_rejected_total_min{};
    std::optional<double> accepted_correlation_min{};
    std::optional<double> rejected_correlation_max{};
    std::optional<bool> walk_accepted{};
    std::optional<std::string> profile{}; // off, window_7 or window_3
    std::optional<std::size_t> direct_count_min{};
    std::optional<std::size_t> direct_count_max{};
    std::optional<std::size_t> direct_count{};
    std::optional<Path> others{};
    std::optional<std::string> phase_vector{}; // 20 letters, D E N or -, phase 0 first
    std::vector<std::size_t> walk_segments{};  // 20 counts, phase 0 first
};

struct Case
{
    std::string id{};
    std::string purpose{};
    std::vector<std::size_t> article_steps{};
    std::string audio{};           // relative to the package root
    std::string clean_reference{}; // relative to the package root
    std::string provenance{};      // relative to the package root
    ::modem::m110a::BodyMode mode{};
    std::string payload_hex{};
    CaseTimeline timeline{};
    std::vector<std::size_t> phase_offsets{};
    CaseExpectations expect{};

    // The timeline as observe() takes it.
    [[nodiscard]] Timeline observed_timeline() const noexcept;
};

struct CaseSet
{
    std::vector<Case> cases{};

    [[nodiscard]] const Case* find(std::string_view id) const noexcept;
};

inline constexpr std::string_view cases_schema = "modem.first-path-cases/1";
inline constexpr std::string_view package_id = "first-path-v1";

// The manifest from text (the tests use this) or from a file. On failure the Status says
// the manifest is not usable and `diagnostic` names the file, the field and what is wrong.
[[nodiscard]] ::modem::common::Result<CaseSet> parse_cases(std::string_view text, std::string& diagnostic);
[[nodiscard]] ::modem::common::Result<CaseSet> read_cases(const std::string& path, std::string& diagnostic);

} // namespace modem_test::first_path
