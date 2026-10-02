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

In this repository only make_root_raised_cosine_taps is implemented; the
transmitter shapes every symbol with it. The matched filter, timing recovery and
CarrierTracker are declared in the header and arrive with the receiver series.

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

} // namespace modem::common
