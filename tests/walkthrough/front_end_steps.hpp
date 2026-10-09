// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

// =============================================================================
// front_end_steps.hpp - the Part 3 walkthrough, one function per step
// =============================================================================
// Each step runs one piece of the receive front end on known data and returns the
// numbers it measured; print() writes them the way the article quotes them. The same
// steps serve two callers: front_end_demo (a main that runs them in article order,
// like tx_demo for Part 2) and front_end_tests (Catch2, which checks what each step
// returns). Set a breakpoint in a step and either caller stops there.
// Explained in the article series, Part 3 (the audio front end).
// =============================================================================

namespace modem_test::front_end_steps
{

using ::modem::common::IQSample;
using ::modem::common::Result;

// --- Mixing down to baseband ---

struct CarrierRow
{
    std::size_t n{};
    double degrees{};
    IQSample entry{};
};

struct CarrierTable
{
    std::vector<CarrierRow> shown{}; // the first eight entries and the quadrants
    double worst_error{};            // largest distance of any entry from the exact phasor
    bool quadrants_exact{};          // entries 0, 20, 40 and 60 are exactly 1, -j, -1 and j
};

[[nodiscard]] CarrierTable carrier_table();
void print(const CarrierTable& step);

struct ToneMix
{
    std::uint32_t tone_hz{};
    float smallest{}; // on-time magnitude, after the filter fills
    float largest{};
    float step{};          // average phase step, radians per symbol
    float expected_step{}; // 2 pi (tone - 1800) / 2400
};

[[nodiscard]] Result<ToneMix> tone_mix(std::uint32_t tone_hz);
void print(const ToneMix& step);

struct MixRow
{
    std::size_t sample{};
    float audio{};
    float carrier_degrees{};
    IQSample mixed{};
};

struct Tribit6Mix
{
    std::vector<MixRow> rows{}; // the samples around the symbol's peak
    IQSample on_time{};         // the front end's output for that symbol
    float degrees{};
    std::uint8_t tribit{};
};

[[nodiscard]] Result<Tribit6Mix> tribit6_mix();
void print(const Tribit6Mix& step);

// --- The matched filter ---

struct ReceiveFilter
{
    std::size_t taps{};
    float energy{};
    float peak{};
    std::size_t middle{};
    bool peak_in_middle{};
    float worst_asymmetry{}; // largest difference between mirrored taps
};

[[nodiscard]] Result<ReceiveFilter> receive_filter();
void print(const ReceiveFilter& step);

struct PulseNeighbors
{
    float receive_rolloff{};
    float peak{};
    float largest_neighbor{}; // at any other symbol's instant
    float decibels_below{};
};

[[nodiscard]] Result<PulseNeighbors> pulse_neighbors(float receive_rolloff);
void print(const PulseNeighbors& step);

// --- Finding the sampling instant ---

struct TimingError
{
    float on_peak{};
    float early{};        // on-time point 4 samples before the peak
    float late{};         // 4 samples after
    float early_turned{}; // early, with the carrier turned by 1 rad
};

[[nodiscard]] Result<TimingError> timing_error();
void print(const TimingError& step);

// --- Keeping time for hours ---

struct FloatPosition
{
    float position{}; // 2^26 + 20 in float
    float earlier{};  // 2^24 + 20.5 in float
    std::uint64_t whole{};
    float fraction{};
};

[[nodiscard]] FloatPosition float_position();
void print(const FloatPosition& step);

// --- Any block size, the same symbols ---

struct BlockSizes
{
    std::size_t samples{};
    std::size_t symbols{};
    std::size_t identical{}; // symbols identical bit for bit in both runs
};

[[nodiscard]] Result<BlockSizes> block_sizes();
void print(const BlockSizes& step);

// --- Carrier tracking, part one ---

struct CarrierTracking
{
    float learned_phase{}; // radians, for a fixed 0.5 rad offset
    float phase_residual{};
    float learned_hz{}; // for a 5 Hz offset
    float frequency_residual{};
};

[[nodiscard]] Result<CarrierTracking> carrier_tracking();
void print(const CarrierTracking& step);

// --- Walk the symbols back out ---

struct WalkRow
{
    std::size_t body_symbol{};
    IQSample on_time{};
    float degrees{};
    std::uint8_t received{};
    std::uint8_t sent{};
    std::uint8_t randomizer{};
    std::uint8_t data{}; // received minus randomizer: 0 or 4 on a data symbol
};

struct Walk
{
    std::vector<WalkRow> shown{}; // the first six body symbols, then the first coded 1
    bool every_tribit_matches{};  // over every body symbol walked
    bool data_is_bpsk{};          // every data symbol came back as 0 or 4
};

[[nodiscard]] Result<Walk> walk();
void print(const Walk& step);

} // namespace modem_test::front_end_steps
