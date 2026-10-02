// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"

#include <cstddef>
#include <span>

// =============================================================================
// timing_recovery.hpp - RRC taps, matched filter + symbol timing, carrier loop
// =============================================================================
// The waveform-neutral demodulation front end: design the root-raised-cosine
// taps, run the matched filter and recover symbol timing from the sample stream,
// and track residual carrier phase/frequency with a second-order loop.
//
// Nothing here knows a waveform; any waveform on the same 1800 Hz / 2400 Bd
// carrier can share it. The 110A body's PREAMBLE ACQUISITION (every body_*
// symbol) is waveform-specific and belongs to the receiver.
//
// The transmitter uses make_root_raised_cosine_taps. The matched filter, timing
// recovery and CarrierTracker are declared here; they arrive with the receiver series.
// =============================================================================

namespace modem::common
{

struct TimingRecoveryConfig
{
    std::size_t samples_per_symbol{2U};
    std::size_t first_symbol_sample{};
    std::size_t phase_search_symbols{64U};
    float loop_gain{0.02F};
    float maximum_step_correction{0.25F};
};

struct TimingRecoveryProgress
{
    std::size_t symbols_written{};
    float final_sample_position{};
    float final_step_correction{};
    std::size_t selected_integer_phase{};
};

[[nodiscard]] Status make_root_raised_cosine_taps(std::size_t samples_per_symbol, float rolloff, std::span<float> taps) noexcept;
[[nodiscard]] Result<TimingRecoveryProgress> matched_filter_and_recover_timing(IQSampleSpan input, std::span<const float> matched_filter_taps, const TimingRecoveryConfig& config,
                                                                               MutableIQSampleSpan filtered_scratch, MutableIQSampleSpan recovered_symbols,
                                                                               MutableIQSampleSpan recovered_half_symbols = {}) noexcept;

class CarrierTracker
{
public:
    static constexpr float default_maximum_frequency_radians_per_symbol = 0.31415926535897932385F;

    [[nodiscard]] Status configure(float proportional_gain, float integral_gain, float maximum_frequency_radians_per_symbol = default_maximum_frequency_radians_per_symbol) noexcept;
    void reset(float phase_radians = 0.0F, float frequency_radians_per_symbol = 0.0F) noexcept;
    // Restore a previously observed (phase, frequency) verbatim, without the
    // wrap/clamp that reset() applies. The caller supplies values that are
    // already the tracker's own post-update output, so re-wrapping is skipped to
    // keep a restored state bit-identical to the state it was captured from.
    void restore(float phase_radians, float frequency_radians_per_symbol) noexcept;
    [[nodiscard]] IQSample update(IQSample input, IQSample decision) noexcept;
    void advance() noexcept;

    [[nodiscard]] float phase_radians() const noexcept { return mPhaseRadians; }

    [[nodiscard]] float frequency_radians_per_symbol() const noexcept { return mFrequencyRadiansPerSymbol; }

    [[nodiscard]] float maximum_frequency_radians_per_symbol() const noexcept { return mMaximumFrequencyRadiansPerSymbol; }

private:
    float mProportionalGain{};
    float mIntegralGain{};
    float mPhaseRadians{};
    float mFrequencyRadiansPerSymbol{};
    float mMaximumFrequencyRadiansPerSymbol{default_maximum_frequency_radians_per_symbol};
};

} // namespace modem::common
