// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/synchronization.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// =============================================================================
// first_path_observe.hpp - what acquisition does with a two-path recording
// =============================================================================
// The Part 5 walkthrough runs on recordings: the "Hi" burst through Table XX's
// two-path channels, rendered outside the bench. observe() runs one recording
// through Part 3's front end (the phase search on, as a receiver runs it while it
// looks for a preamble) and acquire_body_preamble() three ways (the anchoring off,
// and on with the window from 7 and from 3), and reports what it found against
// where the recording's two paths really are. The recording's timeline, which says
// where they are, comes from how it was made, never from this receiver.
// The walkthrough steps and their tests observe through this one function, and so did
// the tool the recordings were selected with.
// Explained in the article series, Part 5 (the first path).
// =============================================================================

namespace modem_test::first_path
{

// Where the recording's paths are: input samples at 48 kHz, counted from the
// recording's first sample, fractions kept. Symbol centres are nominal (the
// transmitter's pulse peaks); the body's first symbol is preamble_symbols later.
struct Timeline
{
    double direct_symbol0_sample{}; // the direct path's first preamble symbol
    double echo_delay_samples{};    // the second path's delay behind it: 240 for 5 ms, 96 for 2 ms
    std::size_t preamble_symbols{};
};

enum class Path : std::uint8_t
{
    direct,
    echo,
    neither,
    none, // nothing acquired
};

[[nodiscard]] const char* path_name(Path path) noexcept;

// How the anchoring moved the burst from the strongest group, if it did.
enum class Move : std::uint8_t
{
    none,
    earlier_group,    // to a group the scan formed
    precursor_search, // to a path the precursor search found
};

[[nodiscard]] const char* move_name(Move move) noexcept;

// One consensus group of the scan: the starts that agree on a body boundary and a mode.
struct GroupObservation
{
    double body_sample{}; // the centre of its first body symbol, recording samples
    Path path{};
    std::size_t supporters{};
    float correlation_sum{};
    float best_correlation{};
};

// acquire_body_preamble() with one setting of the anchoring.
struct ProfileObservation
{
    Path anchor{};
    double body_sample{};
    Move moved_by{};
    ::modem::m110a::BodyAnchorDiagnostics diagnostics{};
    float anchor_correlation{}; // the prefix correlation at the anchor's segment, its own carrier estimate
    // Where the walk tested the earliest precursor peak: the largest prefix correlation over
    // the segment starts that lag reaches back to (0 when there was no peak).
    float walk_lag_correlation_max{};
};

struct Observation
{
    std::size_t phase{};                  // zero samples put in front of the recording
    std::size_t direct_preamble_symbol{}; // the stream symbol the direct path's preamble starts at
    std::size_t scan_last_start{};
    std::vector<GroupObservation> groups{};
    std::size_t strongest{};        // index into groups
    std::size_t strongest_margin{}; // its supporters beyond the next group's
    ProfileObservation off{};       // anchor_earliest_path false
    ProfileObservation window_7{};  // on, anchor_minimum_path_delay_symbols 7
    ProfileObservation window_3{};  // on, from 3
};

// The scan tests starts in the burst's first 1200 symbols, as the receiver that decodes a
// recorded burst tests the first 1200 symbols it is given.
inline constexpr std::size_t scan_symbols = 1200U;

// A recording: 48 kHz, mono (or the first channel), PCM16 or float.
[[nodiscard]] ::modem::common::Result<std::vector<float>> read_recording(const std::string& path);

// One recording at one arrival phase: `phase` zero samples go in front of the whole
// recording, and every position in the timeline moves with them.
[[nodiscard]] ::modem::common::Result<Observation> observe(const std::vector<float>& recording, const Timeline& timeline, std::size_t phase = 0U);

} // namespace modem_test::first_path
