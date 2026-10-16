// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>

// =============================================================================
// body_audio_stream_frontend.hpp - streaming matched filter + early-late timing
// =============================================================================
// The streaming front end: 1800 Hz mix, RRC matched filter, fractional timing,
// block-split invariant. Emits BodyFrontendSymbol {early, on_time} T/2 pairs.
// Explained in the article series, Part 3 (the audio front end), with the phase
// search and the rate integrator in Part 4 (acquisition); its unit tests are
// tests/front_end_tests.cpp and tests/acquisition_tests.cpp.
// Teaching walkthrough: modem/m110a/audio-front-end-explainer.md
// =============================================================================

namespace modem::m110a
{

using ::modem::common::IQSample;
using ::modem::common::SampleSpan;
using ::modem::common::Status;

// One recovered channel symbol from the streaming front end: the half-symbol
// sample T/2 before the decision point and the on-time decision sample.
struct BodyFrontendSymbol
{
    IQSample early{};
    IQSample on_time{};
};

// The early-late timing error for one symbol: the samples half a symbol late and
// half a symbol early, differenced and weighted by the on-time sample. Positive
// when the pulse peak lies after the on-time sample, so the loop samples later.
// Multiplying by the conjugate makes the result independent of the carrier phase.
[[nodiscard]] inline float early_late_timing_error(IQSample early, IQSample on_time, IQSample late) noexcept
{
    return std::real((late - early) * std::conj(on_time));
}

// What the type-2 rate integrator does with each symbol's timing error
// (set_timing_rate_mode). held: it is zero, and the loop is the type-1 loop. tracking: it
// integrates, learning the clock's rate and carrying it. frozen: it stands at the rate it
// has (freeze_timing_rate; a test's oracle, a loop that knows the clock). The receiver holds
// it while it searches, tracks from the first preamble candidate, and once the clock has
// had time to be learned either retires it (held, when the rate it found is one the
// proportional loop carries alone) or keeps it tracking to the end of the burst.
enum class TimingRateMode : std::uint8_t
{
    held,
    tracking,
    frozen,
};

// Streaming receive front end: complex mix at the 1800 Hz carrier,
// root-raised-cosine matched filter, and early-late fractional timing,
// consuming capture blocks of any size with all state retained across calls.
// The output for a given sample stream is invariant to how that stream is
// split into blocks. The timing position is held as a whole sample count plus a
// fraction, so precision does not degrade over a long capture the way a single
// float position does. There is no initial phase search: the tracking loop
// pulls in during the preamble (0.6 s with the short interleaver, 4.8 s with the
// long one). Started near half a symbol off the peaks, though, it hangs on the
// detector's unstable zero for about 2000 symbols, longer than the short
// interleaver's 1440-symbol preamble (the 600S tests in tests/front_end_tests.cpp).
// The matched filter is evaluated only at the interpolation points the loop reads.
class BodyAudioStreamFrontend
{
public:
    // Eight symbols at 20 samples per symbol = 161 taps, shorter than the
    // transmitter's 641-tap shaping filter.
    static constexpr std::size_t matched_filter_span_symbols = 8U;
    static constexpr std::size_t matched_filter_taps = matched_filter_span_symbols * body_audio_samples_per_symbol + 1U;
    // The default receive-filter rolloff; initialize() takes another (0.25 matches the
    // transmitter's pulse). The tap count does not depend on it.
    static constexpr float matched_filter_rolloff = 0.35F;
    static constexpr float default_timing_loop_gain = 0.08F;
    static constexpr float default_maximum_timing_step = 0.20F;
    static constexpr std::size_t maximum_block_samples = 480U;
    // Timing corrections can slide one extra symbol into a block.
    static constexpr std::size_t maximum_block_symbols = maximum_block_samples / body_audio_samples_per_symbol + 2U;

    [[nodiscard]] Status initialize(float timing_loop_gain = default_timing_loop_gain, float maximum_timing_step = default_maximum_timing_step,
                                    float receive_rolloff = matched_filter_rolloff) noexcept;
    void reset() noexcept;

    [[nodiscard]] Status process(SampleSpan captured, std::span<BodyFrontendSymbol> output, std::size_t& produced) noexcept;

    [[nodiscard]] std::uint64_t symbols_produced() const noexcept { return mSymbolsProduced; }

    [[nodiscard]] std::uint32_t clipped_samples() const noexcept { return mClippedSamples; }

    // The type-2 rate integrator's default gain, a fortieth of the proportional gain's 0.08:
    // gentler than the carrier tracker's ratio, because the timing error carries a fading
    // bias the carrier's phase error does not.
    static constexpr float default_timing_integral_gain = 0.0002F;

    // Adds a rate integrator to the timing loop (off by default), the same structure as the
    // carrier tracker's second loop. The type-1 loop is proportional only, so under a clock
    // offset it settles a standing distance off the symbol peaks, the drift divided by its
    // gain, and at 1000 ppm the distance it would need is past the peak of the detector's
    // output, and it slips. The integrator carries the clock's rate, so the proportional term
    // has only the jitter left. integral_gain 0 (or enabled false) leaves the loop the type-1
    // loop. The setting survives reset(); the integrator itself only integrates while
    // set_timing_rate_mode() has it tracking. Call after initialize().
    void configure_timing_tracker(bool enabled, float integral_gain = default_timing_integral_gain) noexcept;

    // What the integrator does from the next process() call; nothing, unless the tracker is
    // configured. held zeroes it, and the loop is the type-1 loop. tracking integrates, from
    // zero after held. frozen keeps the rate as it stands. The receiver holds it while it
    // searches (on a line with no signal the timing error is not zero on average, and an
    // integrator left running walks to rates the next preamble cannot pull in from). Every
    // change of mode starts the running mean again. reset() holds it.
    void set_timing_rate_mode(TimingRateMode mode) noexcept
    {
        if (mode == TimingRateMode::held)
        {
            mTimingRate = 0.0F;
        }

        mTimingRateMode = mode;
        mark_timing_rate_mean();
    }

    // The mean of the integrator's rate over the symbols produced since the last mark. Through
    // fading the momentary rate swings as the strongest path moves, while its mean over tens
    // of seconds stays near the clock's offset, so this, not timing_rate(), is what the
    // receiver judges. 0 with no symbol since the mark.
    void mark_timing_rate_mean() noexcept
    {
        mTimingRateSum = 0;
        mTimingRateSamples = 0U;
    }

    [[nodiscard]] float timing_rate_mean() const noexcept;

    // Freezes the integrator at a given rate, in samples per symbol beyond the nominal 20 (a
    // test's oracle: a loop that knows the clock; the receiver never freezes), clamped to
    // the step limit like the integrator. With the tracker off only the mode changes, so the
    // loop stays the type-1 loop.
    void freeze_timing_rate(float rate) noexcept
    {
        if (mTimingIntegralGain > 0.0F)
        {
            mTimingRate = std::clamp(rate, -mMaximumTimingStep, mMaximumTimingStep);
        }

        mTimingRateMode = TimingRateMode::frozen;
    }

    [[nodiscard]] TimingRateMode timing_rate_mode() const noexcept { return mTimingRateMode; }

    // The phase search, made with the three points the loop already reads. Over every
    // phase_search_window_symbols symbols it sums the on-time energy and the energy half a
    // symbol off it (early and late). A loop on the peaks reads more energy on time (a
    // raised-cosine pulse at rolloff 0.35 has 0.83 of its peak energy half a symbol off); one
    // hung half a symbol off the peaks reads more off time, so when the off-time sum exceeds
    // the on-time sum by phase_search_margin in two windows running, the loop is moved half a
    // symbol forward, onto the peaks, and pulls in from there within a few hundred symbols
    // instead of sliding off the detector's unstable zero for up to 2000. (Two windows, so
    // noise alone, whose sums differ by a few per cent, never moves a locked loop.) The same
    // comparison sets the gain: a window whose off-time energy came within
    // phase_search_fast_gain_ratio of the on-time energy (the loop more than about a sixth of a
    // symbol off the peaks, where the error is small and the pull-in slow) runs the next
    // window at phase_search_loop_gain, three times the tracking gain; any other window runs
    // at the tracking gain, so a loop already on the peaks is not jittered. The receiver turns
    // the search on while it looks for and acquires a preamble, and off once a burst is
    // committed, so a locked body is never moved. Keyed to the symbol count, so the output
    // stays the same however the audio is cut into blocks. Off by default and after reset().
    static constexpr std::uint32_t phase_search_window_symbols = 120U;
    static constexpr float phase_search_margin = 0.06F;
    static constexpr float phase_search_loop_gain = 0.24F;
    static constexpr float phase_search_fast_gain_ratio = 0.92F;

    void set_phase_search(bool enabled) noexcept
    {
        if (enabled != mPhaseSearch)
        {
            mPhaseSearch = enabled;
            mPhaseOnEnergy = 0.0F;
            mPhaseOffEnergy = 0.0F;
            mPhaseWindowSymbols = 0U;
            mPhaseWindowsOff = 0U;
            mPhaseFastGain = false;
        }
    }

    [[nodiscard]] bool phase_search() const noexcept { return mPhaseSearch; }

    // How many times the phase search has moved the loop since initialize().
    [[nodiscard]] std::uint32_t phase_search_moves() const noexcept { return mPhaseSearchMoves; }

    // The integrator's rate, samples per symbol beyond 20 (zero while held or off).
    [[nodiscard]] float timing_rate() const noexcept { return mTimingRate; }

    // Where the loop will read its next on-time sample: the stream sample (counted from the
    // first sample given to process()) and the fraction of a sample past it. Together they
    // are the timing position of symbol symbols_produced(), so a test can measure the loop
    // against the transmitter's symbol grid directly.
    [[nodiscard]] std::uint64_t timing_position_sample() const noexcept { return mPositionSample; }
    [[nodiscard]] float timing_position_fraction() const noexcept { return mPositionFraction; }

private:
    // The deepest read of one produced symbol is the early interpolation
    // point's oldest filter tap: position - half symbol - (taps - 1). The
    // produce loop can trail the write head by up to eleven samples, and a
    // whole maximum-size block is written before production runs, so the
    // ring must retain (taps - 1) + half + 11 + maximum_block_samples = 661
    // samples of history. 1024 holds that with margin.
    static constexpr std::size_t raw_ring_size = 1024U;
    static_assert(raw_ring_size >= matched_filter_taps - 1U + body_audio_samples_per_symbol / 2U + 11U + maximum_block_samples);
    static constexpr std::size_t half_symbol_samples = body_audio_samples_per_symbol / 2U;
    // The first symbol's placement (filter group delay) plus half a symbol, so
    // the early interpolation point stays inside history.
    static constexpr std::uint64_t timing_start_sample = matched_filter_taps - 1U + half_symbol_samples;

    [[nodiscard]] IQSample filtered_at(std::uint64_t sample_index) const noexcept;
    [[nodiscard]] IQSample interpolate_at(std::uint64_t sample_index, float fraction) const noexcept;

    std::array<float, matched_filter_taps> mTaps{};
    std::array<IQSample, raw_ring_size> mBasebandRing{};
    std::uint64_t mWrittenSamples{};
    std::uint64_t mSymbolsProduced{};
    std::uint64_t mPositionSample{};
    float mPositionFraction{};
    float mTimingLoopGain{};
    float mMaximumTimingStep{};
    std::uint32_t mClippedSamples{};
    std::size_t mCarrierIndex{}; // sample number mod body_carrier_period_samples
    // The type-2 rate integrator (configure_timing_tracker): its gain (0, the type-1 loop),
    // its rate, what it does now (set_timing_rate_mode), and its rate summed since the last
    // mark (mark_timing_rate_mean), in fixed point (rate x 2^20 per symbol) so a window of
    // 10^5 symbols loses nothing to float rounding.
    float mTimingIntegralGain{};
    float mTimingRate{};
    static constexpr float timing_rate_mean_scale = 1048576.0F;
    std::int64_t mTimingRateSum{};
    std::uint32_t mTimingRateSamples{};
    TimingRateMode mTimingRateMode{TimingRateMode::held};
    // The phase search (set_phase_search): the window's on-time and off-time energy sums, the
    // symbols in the window so far, the windows running that read more energy off time, the
    // fast gain for the next window, and the moves made.
    bool mPhaseSearch{};
    float mPhaseOnEnergy{};
    float mPhaseOffEnergy{};
    std::uint32_t mPhaseWindowSymbols{};
    std::uint32_t mPhaseWindowsOff{};
    bool mPhaseFastGain{};
    std::uint32_t mPhaseSearchMoves{};
    bool mInitialized{};
};

} // namespace modem::m110a
