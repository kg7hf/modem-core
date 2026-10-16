// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_audio_stream_frontend.hpp"
#include "tests/support/test_waveform.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// =============================================================================
// front_end_harness.hpp - run audio through the receive front end and score it
// =============================================================================
// FrontEndRunner feeds audio to a BodyAudioStreamFrontend in whatever block sizes a
// test asks for and keeps every recovered symbol, numbered from the start of the
// stream. score_burst() lines those symbols up with what the transmitter sent and
// counts the decisions that match once the timing loop has locked. The front end
// does not track the carrier (a later stage does), so the score removes one fixed
// phase rotation per burst before deciding. The runner also keeps the loop's timing
// position after every block, and grid_offsets() measures that against the symbol
// grid the transmitter's clock laid down, so a test can watch the loop itself.
// =============================================================================

namespace modem_test
{

// A change of what the type-2 rate integrator does, landing on one stream sample. With a
// rate, the integrator is frozen at that rate (freeze_timing_rate) whatever the mode says:
// the oracle, a loop that knows the clock.
struct TimingRateChange
{
    std::uint64_t at_sample{};
    modem::m110a::TimingRateMode mode{modem::m110a::TimingRateMode::held};
    std::optional<float> rate{};
};

struct FrontEndOptions
{
    std::vector<std::size_t> block_sizes{480U}; // used in turn, repeating
    float receive_rolloff{modem::m110a::BodyAudioStreamFrontend::matched_filter_rolloff};
    // The type-2 rate tracker: off (the type-1 loop) or on with this integral gain.
    std::optional<float> timing_integral_gain{};
    // The phase search, as the receiver runs it while it looks for a preamble.
    bool phase_search{};
    // What the integrator does from each stream sample on, in order of at_sample; held
    // before the first. A block is cut at each at_sample, so the change lands on that
    // very sample (the symbols are the same however the audio is cut).
    std::vector<TimingRateChange> timing_rate_changes{};
};

// Where the timing loop stood after one block: the position (whole stream sample and
// fraction) at which it will read stream symbol `symbol`, and the rate it had learned.
struct TimingPoint
{
    std::uint64_t symbol{};
    std::uint64_t position_sample{};
    float position_fraction{};
    float timing_rate{};
};

class FrontEndRunner
{
public:
    [[nodiscard]] modem::common::Status initialize(const FrontEndOptions& options = {});
    [[nodiscard]] modem::common::Status feed(std::span<const float> audio);

    // Symbols kept so far; symbols()[k] is stream symbol symbol_base() + k.
    [[nodiscard]] const std::vector<modem::m110a::BodyFrontendSymbol>& symbols() const noexcept { return mSymbols; }

    [[nodiscard]] std::uint64_t symbol_base() const noexcept { return mSymbolBase; }

    [[nodiscard]] std::uint64_t samples_fed() const noexcept { return mSamplesFed; }

    // The loop's position after every block that produced a symbol, oldest first. Feed
    // 20-sample blocks to see every symbol.
    [[nodiscard]] const std::vector<TimingPoint>& timing_points() const noexcept { return mTimingPoints; }

    [[nodiscard]] float timing_rate() const noexcept { return mFrontEnd.timing_rate(); }

    [[nodiscard]] std::uint32_t phase_search_moves() const noexcept { return mFrontEnd.phase_search_moves(); }

    // Drops the kept symbols and timing points (a long run scores each burst, then
    // discards it); the numbering carries on.
    void discard_symbols() noexcept;

private:
    modem::m110a::BodyAudioStreamFrontend mFrontEnd{};
    FrontEndOptions mOptions{};
    std::vector<modem::m110a::BodyFrontendSymbol> mSymbols{};
    std::vector<TimingPoint> mTimingPoints{};
    std::uint64_t mSymbolBase{};
    std::uint64_t mSamplesFed{};
    std::size_t mNextBlock{};
    std::size_t mNextChange{};
};

struct BurstScore
{
    bool locked{};
    std::size_t locked_from{}; // burst symbol where the comparison starts
    std::int64_t offset{};     // stream symbol = burst symbol + offset
    std::size_t compared{};
    std::size_t correct{};
    float rotation_degrees{}; // the fixed carrier rotation removed
    float rms_phase_error_degrees{};
};

// Scores one burst whose first sample sits at stream sample burst_first_sample.
[[nodiscard]] BurstScore score_burst(const FrontEndRunner& runner, const BurstTruth& burst, std::uint64_t burst_first_sample);

// The loop against the transmitter's symbol grid at one timing point: which symbol of the
// burst the loop was reading (a whole number when it sat on a peak) and how far off that
// peak it read, in receive samples, wrapped to (-10, 10].
struct GridOffset
{
    std::uint64_t symbol{};  // stream symbol
    double burst_symbol{};   // the burst's symbol coordinate at the loop's position
    double offset_samples{}; // the loop's position minus the nearest peak
    float timing_rate{};     // the integrator as it stood
};

// Every timing point inside one burst, measured against the grid. The waveform's sample
// 0 was fed at stream sample waveform_start_sample; burst.first_sample counts transmit
// samples from there, and clock_ppm is the channel's clock (the grid stretches with it).
// The receive filter's 80-sample delay is taken out, so a loop on the peaks reads zero.
[[nodiscard]] std::vector<GridOffset> grid_offsets(const FrontEndRunner& runner, const BurstTruth& burst, std::uint64_t waveform_start_sample, float clock_ppm = 0.0F);

// The samples per symbol beyond 20 that a clock offset adds to the received grid: what a
// rate integrator that has learned the clock holds.
[[nodiscard]] float grid_rate(float clock_ppm) noexcept;

// The nearest of the eight phases, as a tribit (tribit t sits at t x 45 degrees).
[[nodiscard]] std::uint8_t nearest_tribit(modem::common::IQSample symbol) noexcept;

// The kept on-time samples, one per symbol: what acquisition searches.
[[nodiscard]] std::vector<modem::common::IQSample> on_time_symbols(const FrontEndRunner& runner);

// Where burst symbol 0 landed in the stream (stream symbol numbering), for a burst whose first
// sample sits at stream sample burst_first_sample. Unlike score_burst it holds with the carrier
// off frequency: the burst's last 2000 symbols are lined up with what was sent eight symbols at
// a time, so a turning carrier cannot cancel the sum. It is the bench's own answer, from what
// was sent, for acquisition to be checked against.
[[nodiscard]] std::uint64_t locate_burst(const FrontEndRunner& runner, const BurstTruth& burst, std::uint64_t burst_first_sample);

} // namespace modem_test
