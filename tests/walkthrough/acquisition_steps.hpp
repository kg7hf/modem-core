// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

// =============================================================================
// acquisition_steps.hpp - the Part 4 walkthrough, one function per step
// =============================================================================
// Each step runs one piece of acquisition on known data and returns the numbers it
// measured; print() writes them the way the article quotes them. The same steps serve
// two callers: acquisition_demo (a main that runs them in article order, like tx_demo
// for Part 2) and acquisition_tests (Catch2, which checks what each step returns). Set
// a breakpoint in a step and either caller stops there.
// Explained in the article series, Part 4 (acquisition: finding the burst).
// =============================================================================

namespace modem_test::acquisition_steps
{

using ::modem::common::IQSample;
using ::modem::common::Result;

// --- The preamble ---

struct SegmentRow
{
    std::size_t segment{};
    std::array<std::uint8_t, 15> channel_symbols{}; // read back from the transmitted chips
    std::uint8_t countdown{};
};

struct PreambleSegments
{
    std::vector<SegmentRow> shown{}; // the first two segments and the last two
    std::size_t segments{};
    bool prefix_on_every_segment{}; // 0 1 3 0 1 3 1 2 0 at the start of each
    bool countdown_runs_down{};     // 23, 22, ... 0
    bool every_chip_is_a_code{};    // each 32-chip block is exactly one of the eight codes
    std::uint8_t d1{};
    std::uint8_t d2{};
    ::modem::m110a::BodyModeRecognition recognition{};
};

[[nodiscard]] Result<PreambleSegments> preamble_segments();
void print(const PreambleSegments& step);

// --- The reference ---

struct ReferenceChip
{
    std::size_t chip{};
    std::uint8_t code{};       // channel symbol 0's chip
    std::uint8_t randomizer{}; // the sync randomizer's tribit
    std::uint8_t tribit{};     // their sum, mod 8
    IQSample point{};
};

struct PrefixReference
{
    std::vector<ReferenceChip> shown{}; // the first eight chips
    std::size_t chips{};
    bool matches_every_segment{}; // bit for bit, against the transmitter's symbols
};

[[nodiscard]] Result<PrefixReference> prefix_reference();
void print(const PrefixReference& step);

// --- Correlating so a fade does not hide it ---

struct FadingPrefix
{
    float clean_coherent{};   // one correlation over all 288 chips
    float clean_blocks{};     // body_prefix_correlation: nine 32-chip blocks, magnitudes summed
    float flipped_coherent{}; // the same, with the last 128 chips turned 180 degrees by a fade
    float flipped_blocks{};
};

[[nodiscard]] FadingPrefix fading_prefix();
void print(const FadingPrefix& step);

struct CombPeak
{
    std::ptrdiff_t start{}; // symbols from the burst's first symbol
    float correlation{};
    float frequency_hz{};
};

struct CorrelationComb
{
    float snr_db{};
    std::int32_t carrier_offset_hz{};
    std::vector<CombPeak> peaks{}; // every start through the carrier gate whose correlation reaches the 0.35 floor
    // The largest correlation more than two symbols from a peak, among the starts whose
    // carrier estimate is inside the 120 Hz gate, as the scan sees them.
    float largest_elsewhere{};
    std::ptrdiff_t largest_elsewhere_start{};
};

[[nodiscard]] Result<CorrelationComb> correlation_comb();
void print(const CorrelationComb& step);

// --- The carrier offset ---

struct OffsetRow
{
    float true_hz{};
    float estimated_hz{};
    bool accepted{}; // within the acquisition's 120 Hz gate
};

struct CarrierRange
{
    std::vector<OffsetRow> rows{};
};

[[nodiscard]] Result<CarrierRange> carrier_range();
void print(const CarrierRange& step);

// --- Reading the mode and the countdown ---

struct RejectedStart
{
    std::size_t start{};
    float correlation{};
    float frequency_hz{};
};

struct ReadSegment
{
    std::vector<RejectedStart> rejected{}; // passed both gates, but did not read as a valid segment
    std::size_t start{};                   // the first start that read as a valid segment, stream symbols
    std::size_t truth_preamble{};          // where the burst's first symbol really is
    float correlation{};
    float frequency_hz{};
    std::uint8_t d1{};
    std::uint8_t d2{};
    std::uint8_t countdown{};
    ::modem::m110a::BodyModeRecognition recognition{};
    std::size_t first_body_symbol{};
    std::size_t truth_body{};
    std::size_t first_preamble_symbol{};
};

[[nodiscard]] Result<ReadSegment> read_segment();
void print(const ReadSegment& step);

struct WakeMidPreamble
{
    std::size_t symbols_missed{}; // the receiver starts this far into the burst
    std::size_t segment_read{};   // the burst segment acquisition read the countdown from
    std::uint8_t countdown{};
    std::size_t first_body_symbol{}; // in the receiver's own numbering
    std::size_t truth_body{};
    bool preamble_start_in_buffer{};
};

[[nodiscard]] Result<WakeMidPreamble> wake_mid_preamble();
void print(const WakeMidPreamble& step);

struct UnsupportedMode
{
    ::modem::m110a::BodyDesignatorState as_sent{}; // D1, D2 = 6, 6: 600S
    bool as_sent_acquired{};
    ::modem::m110a::BodyDesignatorState patched{}; // D1, D2 = 7, 7: a mode this modem does not implement
    bool patched_acquired{};
};

[[nodiscard]] Result<UnsupportedMode> unsupported_mode();
void print(const UnsupportedMode& step);

// --- One segment, or all of them ---

struct VoteRow
{
    std::size_t segment{};
    float correlation{};
    bool valid{};
    std::uint8_t countdown{};
    ::modem::common::DataRate rate{};
    std::size_t first_body_symbol{};
    bool right{}; // the mode and body boundary the burst really has
};

struct VoteGroup
{
    std::size_t first_body_symbol{};
    ::modem::common::DataRate rate{};
    std::size_t supporters{};
};

struct SegmentVote
{
    float snr_db{};
    std::vector<VoteRow> rows{};     // every segment, read on its own
    std::vector<VoteGroup> groups{}; // the valid reads, grouped by the burst they describe
    std::size_t right{};
    std::size_t wrong{};
    std::size_t invalid{};
    bool acquired{};
    bool acquisition_right{};
};

[[nodiscard]] Result<SegmentVote> segment_vote(float snr_db);
void print(const SegmentVote& step);

// --- No burst at all ---

struct NoiseOnly
{
    float seconds{};
    float largest_correlation{}; // over every start the carrier gate lets through
    std::size_t over_floor{};    // starts through both gates
    std::size_t valid{};         // of those, the ones that read as a valid segment
    bool acquired{};
};

[[nodiscard]] Result<NoiseOnly> noise_only();
void print(const NoiseOnly& step);

// --- Part 3's hang-up, fixed while acquiring ---

struct PhaseStart
{
    std::size_t lead{};                                 // samples of silence before the burst: where in a symbol the loop starts
    std::optional<std::size_t> plain_pulled_in_at{};    // burst symbol; empty: not within the preamble
    std::optional<std::size_t> searched_pulled_in_at{}; // with the phase search
    std::uint32_t searched_moves{};                     // half-symbol moves the search made
};

struct PhaseSearch
{
    float snr_db{};
    std::size_t start_phases{};  // every sample phase of a symbol
    std::size_t plain_in_time{}; // pulled in before burst symbol 1200 (of the 1440-symbol preamble)
    std::size_t searched_in_time{};
    std::vector<PhaseStart> late_starts{}; // the starts the plain loop did not pull in from in time
};

[[nodiscard]] Result<PhaseSearch> phase_search();
void print(const PhaseSearch& step);

// --- The clock-offset loop ---

struct ClockRow
{
    float clock_ppm{};
    ::modem::m110a::BodyMode mode{};
    bool type_2{};                          // the rate integrator running from the first candidate
    double body_offset{};                   // the loop's mean distance from the symbol peaks through the body, samples
    double body_wander{};                   // the largest distance from that mean
    std::size_t slips{};                    // whole symbols the loop gained or lost in the body
    std::optional<float> learned_ppm_off{}; // type 2: how far the integrator's rate is from the clock's at the end
};

struct ClockOffset
{
    float snr_db{};
    std::vector<ClockRow> rows{};
};

[[nodiscard]] Result<ClockOffset> clock_offset();
void print(const ClockOffset& step);

struct LearningRow
{
    float snr_db{};
    std::vector<float> error_at_window_ppm{};             // per seed: the integrator's error 2880 symbols after the first candidate
    std::vector<std::optional<std::size_t>> settled_at{}; // per seed: symbols after the first candidate until it stays within 100 ppm
};

struct LearningTime
{
    ::modem::m110a::BodyMode mode{};
    float clock_ppm{};
    std::vector<LearningRow> rows{};
};

[[nodiscard]] Result<LearningTime> learning_time();
void print(const LearningTime& step);

struct RuleRow
{
    std::uint64_t seed{};
    float clock_ppm{};
    double largest_rate_ppm{}; // the integrator's largest momentary rate from 5 s to 20 s
    double judged_mean_ppm{};  // its mean over the same span: what the receiver judges at 20 s
    bool retired{};            // below the 20 s threshold: held at zero for the rest of the burst
};

struct LongBurstRule
{
    float snr_db{};
    float delay_ms{};
    float doppler_spread_hz{};
    std::size_t data_seconds{}; // the body is longer, padded to whole 4.8 s interleaver blocks
    double threshold_ppm{};     // at the first judgement
    std::vector<RuleRow> rows{};
};

[[nodiscard]] Result<LongBurstRule> long_burst_rule();
void print(const LongBurstRule& step);

} // namespace modem_test::acquisition_steps
