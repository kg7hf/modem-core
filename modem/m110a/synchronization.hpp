// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"

#include <array>
#include <cstddef>
#include <span>

// =============================================================================
// synchronization.hpp - acquisition
// =============================================================================
// This header declares the receiver's FRONT DOOR: the primitives that locate a
// MIL-STD-188-110B body burst in symbol-rate complex baseband and hand the rest
// of the receiver a locked reference: BodyAcquisitionConfig / BodyAcquisition
// (with the earliest-path anchoring's diagnostics) and acquire_body_preamble(),
// with the candidate-evaluation primitives a streaming receiver can call one
// rolling position at a time (make_body_prefix_reference,
// estimate_body_prefix_frequency_radians, body_prefix_correlation,
// characterize_body_candidate, body_candidate_is_valid) and the reference's own
// sidelobe model (body_precursor_sidelobe_powers).
// Explained in the article series, Part 4 (acquisition: finding the burst) and
// Part 5 (the first path); their unit tests are tests/acquisition_tests.cpp and
// tests/first_path_tests.cpp.
//
// The full teaching walkthrough - pipeline diagram, worked example on the
// demo's own output, equations, fading-failure modes, and debugging signals - is in
//   modem/m110a/synchronization-and-acquisition-explainer.md
// Implementation and its rationale are in synchronization.cpp.
// =============================================================================

namespace modem::m110a
{

using ::modem::common::IQSampleSpan;
using ::modem::common::Result;
using ::modem::common::Status;

struct BodyAcquisitionConfig
{
    float symbol_rate_hz{2400.0F};
    float minimum_correlation{0.75F};          // the correlation floor
    float maximum_frequency_offset_hz{120.0F}; // the carrier gate
    std::size_t maximum_start_symbol{};        // the last start searched; 0 searches them all
    // Earliest-path anchoring (on by default; false returns the strongest group
    // as the scan leaves it). On a multipath channel every propagation path
    // produces its own consensus group, offset by the path delay and all
    // describing the same transmission. The strongest group, the one with the
    // most supporting segments, is the ECHO whenever the direct path was the
    // weaker through the segments the scan covers. The direct path then sits at
    // a negative lag the equalizer's precursor span cannot reach, and the
    // countdown decoded on that anchor can be off by a segment or two. With this
    // flag the winner is re-anchored to the EARLIEST path of the same
    // transmission (consensus group, then a multi-segment negative-lag prefix
    // search), the designators and countdown are re-decoded on it, and the body
    // boundary is the majority over every preamble segment.
    bool anchor_earliest_path{true};
    std::size_t anchor_maximum_path_delay_symbols{15U};
    // Re-anchor only when the earlier path lies BEYOND the adaptive equalizer's
    // precursor reach (its short geometry: 24 T/2 taps, cursor 12 = 6 symbols).
    // Inside that reach the adaptive equalizer already handles a precursor path
    // (the 2 ms rows of Table XX, 4.8 symbols), and moving the anchor there only
    // re-rolls a marginal equalizer. The 5 ms rows sit at 12 symbols, far beyond
    // it. The host's whole-burst turbo decoder needs the first path even at 2 ms,
    // and runs with 3.
    std::size_t anchor_minimum_path_delay_symbols{7U};
};

// What the earliest-path anchoring saw and decided, kept
// on the acquisition for the host's diagnostics: the precursor search's
// earliest significant lag and its power fraction of lag 0, the segments it
// accumulated, and how the re-characterization walk fared (segments tried
// and the count rejected by each test) - so a burst that keeps a later-path
// anchor can be traced to the test that kept it.
struct BodyAnchorDiagnostics
{
    // Sidelobe-corrected precursor power at lags 0..15 as a fraction of lag 0.
    std::array<float, 16U> precursor_powers{};
    // Qualifying peaks (earliest first) and the earliest one.
    std::size_t precursor_candidates{};
    std::size_t precursor_lag{};
    float precursor_fraction{};
    std::size_t precursor_segments{};
    std::size_t walk_segments{};
    std::size_t walk_rejected_frequency{};
    std::size_t walk_rejected_correlation{};
    std::size_t walk_rejected_invalid{};
    std::size_t walk_rejected_mode{};
    std::size_t reanchored_lag{};
    bool reanchored{};
};

struct BodyAcquisition
{
    std::size_t first_preamble_symbol{};
    std::size_t detected_segment_symbol{};
    std::size_t first_body_symbol{};
    std::uint8_t preamble_countdown{};
    bool preamble_start_in_buffer{};
    float symbol_rate_hz{body_symbol_rate.per_second_real()};
    float frequency_offset_hz{};
    float carrier_phase_radians{};
    float normalized_correlation{};
    BodyModeRecognition recognition{};
    BodyAnchorDiagnostics anchor{};
};

[[nodiscard]] Result<BodyAcquisition> acquire_body_preamble(IQSampleSpan symbol_rate_input, const BodyAcquisitionConfig& config) noexcept;

// Candidate-evaluation primitives of acquire_body_preamble, exposed so a
// streaming receiver can evaluate one rolling candidate position per step
// instead of re-scanning a whole window every codec block. All operate on
// symbol-rate complex baseband and share the batch scanner's conventions.
//
// The 288-chip sync-scrambled acquisition-prefix reference, written into
// exactly body_preamble_acquisition_prefix_symbols symbols. Constant per
// build - compute once into persistent storage.
void make_body_prefix_reference(std::span<::modem::common::IQSample, body_preamble_acquisition_prefix_symbols> reference) noexcept;
// Differential carrier estimate over the prefix at `start`, radians/symbol.
[[nodiscard]] float estimate_body_prefix_frequency_radians(IQSampleSpan input, std::size_t start, IQSampleSpan reference) noexcept;
// Normalized noncoherent prefix correlation in [0, 1]; carrier_phase_radians
// receives the coherent phase estimate at `start`.
[[nodiscard]] float body_prefix_correlation(IQSampleSpan input, std::size_t start, IQSampleSpan reference, float frequency_radians, float& carrier_phase_radians) noexcept;
// Decodes D1/D2/countdown at the seeded candidate (first_preamble_symbol,
// frequency_offset_hz, carrier_phase_radians, normalized_correlation filled
// in) and resolves mode recognition and the body boundary. The candidate
// start must have at least one whole 480-symbol segment of input ahead.
[[nodiscard]] BodyAcquisition characterize_body_candidate(IQSampleSpan input, BodyAcquisition candidate, const BodyAcquisitionConfig& config) noexcept;
// True when the characterized candidate names a supported mode with an
// internally consistent countdown.
[[nodiscard]] bool body_candidate_is_valid(const BodyAcquisition& candidate) noexcept;
// The prefix reference's own structural power at negative lags 0..maximum_lag
// (at most 15): the reference shifted into the previous segment's known tail,
// scored as a path is (the noncoherent sum of its nine 32-chip block
// correlations; lag 0 is the main path's own power). A measured precursor
// profile subtracts (sidelobe[lag] / sidelobe[0]) x power[0] so the main
// path's sidelobes are not read as an earlier path.
[[nodiscard]] std::array<float, 16U> body_precursor_sidelobe_powers(IQSampleSpan reference, std::size_t maximum_lag) noexcept;

} // namespace modem::m110a
