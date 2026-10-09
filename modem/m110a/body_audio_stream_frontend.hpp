// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"

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
// Explained in the article series, Part 3 (the audio front end); its unit tests
// are tests/front_end_tests.cpp.
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
    bool mInitialized{};
};

} // namespace modem::m110a
