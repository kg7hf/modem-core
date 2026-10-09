// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
timing_recovery.cpp - the waveform-neutral demodulation front end
================================================================================
WHAT THIS IS
------------
make_root_raised_cosine_taps  - the energy-normalized RRC pulse the transmitter
                                shapes with and the receiver matches against.
matched_filter_and_recover_timing - matched filter plus symbol-timing recovery
                                over a sample stream; returns the symbol decisions
                                and where the loop left the timing phase.
CarrierTracker                - second-order phase/frequency loop closing on the
                                residual carrier after timing is locked.

In this repository make_root_raised_cosine_taps (the transmitter shapes every
symbol with it) and CarrierTracker are implemented. The whole-buffer
matched_filter_and_recover_timing is declared in the header and arrives later; the
streaming front end used for live audio is m110a/body_audio_stream_frontend.

The 110A body preamble acquisition (every body_* symbol) is waveform-specific and
belongs to the receiver. This file mentions no Body* type at all.
================================================================================
*/

#include "modem/common/dsp/timing_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <span>

namespace modem::common
{

constexpr float pi = 3.14159265358979323846F;
constexpr float two_pi = 2.0F * pi;

// Wraps a phase to (-pi, pi]. The loop form handles any finite input.
static float wrap_phase(float phase) noexcept
{
    while (phase > pi)
    {
        phase -= two_pi;
    }

    while (phase < -pi)
    {
        phase += two_pi;
    }

    return phase;
}

Status make_root_raised_cosine_taps(std::size_t samples_per_symbol, float rolloff, std::span<float> taps) noexcept
{
    if (samples_per_symbol < 2U || rolloff <= 0.0F || rolloff > 1.0F || taps.size() < 3U || (taps.size() & 1U) == 0U)
    {
        return {StatusCode::invalid_argument, "invalid root-raised-cosine filter request"};
    }

    const auto center = static_cast<float>(taps.size() - 1U) * 0.5F;
    float energy{};

    for (std::size_t index = 0U; index < taps.size(); ++index)
    {
        const auto time = (static_cast<float>(index) - center) / static_cast<float>(samples_per_symbol);
        float value{};

        if (std::fabs(time) < 1.0e-6F)
        {
            value = 1.0F + rolloff * (4.0F / pi - 1.0F);
        }
        else if (std::fabs(std::fabs(4.0F * rolloff * time) - 1.0F) < 1.0e-5F)
        {
            const auto angle = pi / (4.0F * rolloff);
            value = (rolloff / rmath::sqrt(2.0F)) * ((1.0F + 2.0F / pi) * rmath::sin(angle) + (1.0F - 2.0F / pi) * rmath::cos(angle));
        }
        else
        {
            const auto numerator = rmath::sin(pi * time * (1.0F - rolloff)) + 4.0F * rolloff * time * rmath::cos(pi * time * (1.0F + rolloff));
            const auto denominator = pi * time * (1.0F - 16.0F * rolloff * rolloff * time * time);
            value = numerator / denominator;
        }

        taps[index] = value;
        energy += value * value;
    }

    if (energy <= 0.0F)
    {
        return {StatusCode::internal_error, "root-raised-cosine filter has no energy"};
    }

    const auto normalization = 1.0F / rmath::sqrt(energy);

    for (auto& tap : taps)
    {
        tap *= normalization;
    }

    return Status::success();
}

Status CarrierTracker::configure(float proportional_gain, float integral_gain, float maximum_frequency_radians_per_symbol) noexcept
{
    if (!std::isfinite(proportional_gain) || !std::isfinite(integral_gain) || !std::isfinite(maximum_frequency_radians_per_symbol) || proportional_gain < 0.0F || integral_gain < 0.0F ||
        maximum_frequency_radians_per_symbol <= 0.0F || maximum_frequency_radians_per_symbol > pi)
    {
        return {StatusCode::invalid_argument, "carrier tracker configuration is invalid"};
    }

    mProportionalGain = proportional_gain;
    mIntegralGain = integral_gain;
    mMaximumFrequencyRadiansPerSymbol = maximum_frequency_radians_per_symbol;
    mFrequencyRadiansPerSymbol = std::clamp(mFrequencyRadiansPerSymbol, -mMaximumFrequencyRadiansPerSymbol, mMaximumFrequencyRadiansPerSymbol);
    return Status::success();
}

void CarrierTracker::reset(float phase_radians, float frequency_radians_per_symbol) noexcept
{
    mPhaseRadians = std::isfinite(phase_radians) ? wrap_phase(phase_radians) : 0.0F;
    mFrequencyRadiansPerSymbol = std::isfinite(frequency_radians_per_symbol) ? std::clamp(frequency_radians_per_symbol, -mMaximumFrequencyRadiansPerSymbol, mMaximumFrequencyRadiansPerSymbol) : 0.0F;
}

void CarrierTracker::restore(float phase_radians, float frequency_radians_per_symbol) noexcept
{
    mPhaseRadians = phase_radians;
    mFrequencyRadiansPerSymbol = frequency_radians_per_symbol;
}

// -----------------------------------------------------------------------------
// CarrierTracker::update  (decision-directed 2nd-order carrier loop)
// -----------------------------------------------------------------------------
// 50K view: Remove residual carrier phase/frequency during demodulation, learning
//   from each symbol decision.
// Detailed view: Derotate the input by the current phase; form the phase error
//   arg(corrected*conj(decision)); integrate it into the frequency estimate
//   (clamped to +/- max rad/symbol); advance the phase by frequency +
//   proportional*error. advance() free-wheels the phase for known/skipped
//   symbols; reset() wraps/clamps a fresh state; restore() re-installs a captured
//   state bit-for-bit (no re-wrap) for exact streaming replay.
// 5th-grade view: Keep gently turning a dial so each symbol lands upright; if the
//   dial keeps needing the same turn, learn that drift and turn ahead of it.
// -----------------------------------------------------------------------------
IQSample CarrierTracker::update(IQSample input, IQSample decision) noexcept
{
    const auto corrected = input * std::polar(1.0F, -mPhaseRadians);
    const auto error = std::arg(corrected * std::conj(decision));
    mFrequencyRadiansPerSymbol = std::clamp(mFrequencyRadiansPerSymbol + mIntegralGain * error, -mMaximumFrequencyRadiansPerSymbol, mMaximumFrequencyRadiansPerSymbol);
    mPhaseRadians = wrap_phase(mPhaseRadians + mFrequencyRadiansPerSymbol + mProportionalGain * error);
    return corrected;
}

void CarrierTracker::advance() noexcept
{
    mPhaseRadians = wrap_phase(mPhaseRadians + mFrequencyRadiansPerSymbol);
}

} // namespace modem::common
