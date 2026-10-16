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
// and acquire_body_preamble(), with the candidate-evaluation primitives a
// streaming receiver can call one rolling position at a time
// (make_body_prefix_reference, estimate_body_prefix_frequency_radians,
// body_prefix_correlation, characterize_body_candidate,
// body_candidate_is_valid).
// Explained in the article series, Part 4 (acquisition: finding the burst); its
// unit tests are tests/acquisition_tests.cpp.
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

} // namespace modem::m110a
