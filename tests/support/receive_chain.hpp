// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/m110a/synchronization.hpp"
#include "modem/m110a/transmitter.hpp"
#include "tests/support/test_channel.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

// =============================================================================
// receive_chain.hpp - the receive chain end to end, for the runner
// =============================================================================
// Each article explains one stage; the runner runs them all, in order, on a message of
// your own or on a recording. Part 2's transmitter makes the burst and the test channel
// adds what the air does to it; Part 3's front end turns the audio into symbols; Part 4's
// acquisition finds the burst in them with the phase search on, reads its mode and
// countdown, and from the first candidate the receiver turns the search off and lets the
// front end's rate integrator learn the clock. Each article adds its stage here.
// run_chain() returns what every stage found, print() shows it, and runner_tests checks
// the same report.
// =============================================================================

namespace modem_test
{

struct ChainSpec
{
    std::string message{"Hi"};
    modem::m110a::BodyMode mode{modem::common::DataRate::bps600, modem::common::BodyInterleave::long_block};
    ChannelSpec channel{};
    std::optional<std::string> wav_in{};  // run the receive stages on this recording instead
    std::optional<std::string> wav_out{}; // save the audio the receive stages are given
};

// Part 2: what the transmitter sent (none for a recording).
struct TransmitReport
{
    std::size_t bytes{};
    std::size_t preamble_symbols{};
    std::size_t body_symbols{};
    std::size_t audio_samples{};
};

// Where the bench measured the timing loop through the body, against the transmitter's
// symbol grid: its mean distance from the symbol peaks, and how many symbols it slipped.
struct GridReport
{
    double mean_offset_samples{};
    std::int64_t slipped_symbols{};
};

// Part 3: the front end exactly as Part 3 has it, the proportional loop and nothing else.
struct FrontEndReport
{
    std::size_t audio_samples{};
    std::uint32_t sample_rate_hz{};
    std::size_t symbols{};
    std::optional<GridReport> grid{}; // a generated burst only
};

// Part 4: the front end with the phase search on and acquisition at the walkthrough's 0.35
// floor; then the receiver committed at the first candidate, a segment into the preamble: the
// search off from there, and the rate integrator tracking.
struct AcquisitionReport
{
    std::uint32_t phase_search_moves{};
    modem::common::Status status{};
    std::optional<modem::m110a::BodyAcquisition> acquired{};
    std::optional<std::size_t> true_first_preamble{}; // the bench's own answer, for a generated burst
    std::optional<std::size_t> true_first_body{};
    std::size_t committed_at_symbol{};
    double learned_clock_ppm{};       // the integrator's mean rate through the body, as a clock offset
    std::optional<GridReport> grid{}; // the committed loop, a generated burst only
};

struct ChainReport
{
    ChainSpec spec{};
    std::optional<TransmitReport> transmit{};
    std::size_t lead_samples{}; // line before the burst, for a generated burst
    FrontEndReport front_end{};
    AcquisitionReport acquisition{};
};

[[nodiscard]] modem::common::Result<ChainReport> run_chain(const ChainSpec& spec);
void print(const ChainReport& report);

// "600L", "75S", "4800" and the like; nothing for a mode this modem does not implement.
[[nodiscard]] std::optional<modem::m110a::BodyMode> parse_mode(const std::string& text);
[[nodiscard]] std::string mode_text(modem::m110a::BodyMode mode);

} // namespace modem_test
