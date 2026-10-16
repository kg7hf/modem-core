// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
body_audio_stream_frontend.cpp - streaming matched filter + early-late timing
================================================================================
Complex-mix each 48 kHz sample against the 1800 Hz carrier into a ring,
RRC matched-filter it (161 taps, rolloff 0.35), and run an early-late timing
loop. All state is retained across calls, so the recovered symbols are
INVARIANT to how the sample stream is split into blocks. The timing position is
a whole sample count plus a fraction (precision over a long capture). There is
no whole-buffer phase search: the loop pulls in during the preamble, and the phase
search (set_phase_search) moves it half a symbol when it starts hung between the
peaks. An optional rate integrator (configure_timing_tracker) carries a clock offset.
  INPUT  : SampleSpan captured audio.
  OUTPUT : BodyFrontendSymbol {early, on_time} per symbol (T/2 pair).
================================================================================
*/

#include "modem/m110a/body_audio_stream_frontend.hpp"

#include "modem/common/convert.hpp"
#include "modem/common/dsp/timing_recovery.hpp"
#include "modem/common/status.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>

namespace modem::m110a
{

using ::modem::common::approx_real;
using ::modem::common::as_real;
using ::modem::common::clamp_to_size;
using ::modem::common::floor_to;
using ::modem::common::IQSample;
using ::modem::common::make_root_raised_cosine_taps;
using ::modem::common::ring_slot;
using ::modem::common::round_to;
using ::modem::common::SampleSpan;
using ::modem::common::Status;
using ::modem::common::StatusCode;

constexpr float clip_threshold = 0.999F;
// The phase search averages the energy of the two off-time points, early and late.
constexpr float off_time_points = 2.0F;

Status BodyAudioStreamFrontend::initialize(float timing_loop_gain, float maximum_timing_step, float receive_rolloff) noexcept
{
    if (!(timing_loop_gain >= 0.0F) || !(maximum_timing_step > 0.0F))
    {
        return {StatusCode::invalid_argument, "invalid streaming front-end timing configuration"};
    }

    const auto taps_status = make_root_raised_cosine_taps(body_audio_samples_per_symbol, receive_rolloff, mTaps);

    if (!taps_status.is_ok())
    {
        return taps_status;
    }

    mTimingLoopGain = timing_loop_gain;
    mMaximumTimingStep = maximum_timing_step;
    mInitialized = true;
    reset();
    return Status::success();
}

void BodyAudioStreamFrontend::reset() noexcept
{
    mBasebandRing.fill(IQSample{});
    mWrittenSamples = 0U;
    mCarrierIndex = 0U;
    mSymbolsProduced = 0U;
    mPositionSample = timing_start_sample;
    mPositionFraction = 0.0F;
    // The rate integrator is held until the receiver has it track a preamble; its gain
    // stays. The phase search is off until the receiver turns it on.
    set_timing_rate_mode(TimingRateMode::held);
    mPhaseSearch = false;
    mPhaseOnEnergy = 0.0F;
    mPhaseOffEnergy = 0.0F;
    mPhaseWindowSymbols = 0U;
    mPhaseWindowsOff = 0U;
    mPhaseFastGain = false;
    mClippedSamples = 0U;
}

void BodyAudioStreamFrontend::configure_timing_tracker(bool enabled, float integral_gain) noexcept
{
    mTimingIntegralGain = (enabled && integral_gain > 0.0F) ? integral_gain : 0.0F;
    mTimingRate = 0.0F;
}

float BodyAudioStreamFrontend::timing_rate_mean() const noexcept
{
    // The sum is at most 2^20 x 0.2 x 2^32, and the rounding of its float image (past 2^24)
    // is far below the rate's own resolution.
    return mTimingRateSamples == 0U ? 0.0F : approx_real(mTimingRateSum) / (timing_rate_mean_scale * as_real(mTimingRateSamples));
}

IQSample BodyAudioStreamFrontend::filtered_at(std::uint64_t sample_index) const noexcept
{
    // Causal convolution: taps reaching before sample zero are skipped.
    const auto available = clamp_to_size(sample_index + 1U, matched_filter_taps);
    IQSample sum{};

    for (std::size_t tap = 0U; tap < available; ++tap)
    {
        sum += mTaps[tap] * mBasebandRing[ring_slot<raw_ring_size>(sample_index - tap)];
    }

    return sum;
}

IQSample BodyAudioStreamFrontend::interpolate_at(std::uint64_t sample_index, float fraction) const noexcept
{
    const auto lower = filtered_at(sample_index);
    const auto upper = filtered_at(sample_index + 1U);
    return lower + fraction * (upper - lower);
}

Status BodyAudioStreamFrontend::process(SampleSpan captured, std::span<BodyFrontendSymbol> output, std::size_t& produced) noexcept
{
    produced = 0U;

    if (!mInitialized)
    {
        return {StatusCode::unavailable, "streaming front end is not initialized"};
    }

    if (captured.size() > maximum_block_samples || output.size() < captured.size() / body_audio_samples_per_symbol + 2U)
    {
        return {StatusCode::invalid_argument, "streaming front-end block buffers are invalid"};
    }

    for (const auto sample : captured)
    {
        if (std::fabs(sample) >= clip_threshold)
        {
            ++mClippedSamples;
        }

        // Mix down with the conjugate carrier: the table entry for this sample number mod 80.
        mBasebandRing[ring_slot<raw_ring_size>(mWrittenSamples)] = 2.0F * sample * std::conj(body_carrier_table[mCarrierIndex]);
        ++mWrittenSamples;

        // The carrier index is the sample number mod 80, kept as a wrapping counter: a 64-bit
        // remainder per sample would be a library call on a part without a 64-bit divider.
        if (++mCarrierIndex == body_carrier_period_samples)
        {
            mCarrierIndex = 0U;
        }
    }

    // The late interpolation point reads filtered samples up to
    // position + half + 1, so a symbol is ready once that lookahead exists.
    while (mPositionSample + half_symbol_samples + 2U <= mWrittenSamples && produced < output.size())
    {
        const auto early = interpolate_at(mPositionSample - half_symbol_samples, mPositionFraction);
        const auto on_time = interpolate_at(mPositionSample, mPositionFraction);
        const auto late = interpolate_at(mPositionSample + half_symbol_samples, mPositionFraction);
        output[produced].early = early;
        output[produced].on_time = on_time;
        ++produced;
        ++mSymbolsProduced;

        // The error is divided by the energy of the three samples, so a loud signal and a
        // quiet one move the loop by the same amount, and each step is limited.
        const auto timing_error = early_late_timing_error(early, on_time, late);
        const auto energy = std::norm(early) + std::norm(on_time) + std::norm(late);
        const auto normalization = std::max(energy, 1.0e-6F);

        // The phase search: more energy half a symbol off the loop's points than on them,
        // summed over a window, is the sign of a loop hung half a symbol off the peaks; it is
        // moved half a symbol forward, onto them.
        float phase_move = 0.0F;
        if (mPhaseSearch)
        {
            mPhaseOnEnergy += std::norm(on_time);
            mPhaseOffEnergy += (std::norm(early) + std::norm(late)) / off_time_points;

            if (++mPhaseWindowSymbols >= phase_search_window_symbols)
            {
                mPhaseWindowsOff = mPhaseOffEnergy > mPhaseOnEnergy * (1.0F + phase_search_margin) ? mPhaseWindowsOff + 1U : 0U;
                mPhaseFastGain = mPhaseOffEnergy > mPhaseOnEnergy * phase_search_fast_gain_ratio;

                if (mPhaseWindowsOff >= 2U)
                {
                    phase_move = as_real(half_symbol_samples);
                    ++mPhaseSearchMoves;
                    mPhaseWindowsOff = 0U;
                }

                mPhaseOnEnergy = 0.0F;
                mPhaseOffEnergy = 0.0F;
                mPhaseWindowSymbols = 0U;
            }
        }

        const auto loop_gain = mPhaseSearch && mPhaseFastGain ? phase_search_loop_gain : mTimingLoopGain;
        float correction = 0.0F;
        if (mTimingIntegralGain > 0.0F && mTimingRateMode != TimingRateMode::held)
        {
            // Type 2: the integrator carries the clock's rate, and the proportional term only
            // the jitter. It integrates while tracking; frozen, the rate stands. Both are
            // limited to the step limit, which is far beyond any real clock offset.
            const auto normalized_error = timing_error / normalization;

            if (mTimingRateMode == TimingRateMode::tracking)
            {
                mTimingRate = std::clamp(mTimingRate + mTimingIntegralGain * normalized_error, -mMaximumTimingStep, mMaximumTimingStep);
            }

            mTimingRateSum += round_to<std::int64_t>(mTimingRate * timing_rate_mean_scale);
            ++mTimingRateSamples;

            correction = std::clamp(loop_gain * normalized_error + mTimingRate, -mMaximumTimingStep, mMaximumTimingStep);
        }
        else
        {
            // Type 1, the default, and the tracker while held: the proportional loop.
            correction = std::clamp(loop_gain * timing_error / normalization, -mMaximumTimingStep, mMaximumTimingStep);
        }
        mPositionFraction += as_real(body_audio_samples_per_symbol) + correction + phase_move;
        const auto whole = std::floor(mPositionFraction);
        mPositionSample += floor_to<std::uint64_t>(whole); // whole is already integral and positive
        mPositionFraction -= whole;
    }

    return Status::success();
}

} // namespace modem::m110a
