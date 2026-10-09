// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/support/impairments.hpp"

#include "modem/common/convert.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>

namespace modem_test
{

using ::modem::common::as_real;
using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::MutableIQSampleSpan;
using ::modem::common::Status;
using ::modem::common::StatusCode;

constexpr float two_pi = 6.2831853071795864769F;

DeterministicRng::DeterministicRng(std::uint64_t seed) noexcept : mState{seed == 0U ? 0x9E3779B97F4A7C15ULL : seed} {}

std::uint64_t DeterministicRng::next_u64() noexcept
{
    auto value = mState;
    value ^= value >> 12U;
    value ^= value << 25U;
    value ^= value >> 27U;
    mState = value;
    return value * 0x2545F4914F6CDD1DULL;
}

float DeterministicRng::uniform() noexcept
{
    // The top 53 bits as a double in [0, 1), rounded once to float. convert.hpp has no
    // helper for a 53-bit integer or for double to float, so the two casts are spelled here.
    constexpr double scale = 0x1.0p-53;
    return static_cast<float>(static_cast<double>(next_u64() >> 11U) * scale);
}

float DeterministicRng::gaussian() noexcept
{
    if (mHasSpare)
    {
        mHasSpare = false;
        return mSpare;
    }

    const auto u1 = std::max(uniform(), 1.0e-12F);
    const auto u2 = uniform();
    const auto magnitude = std::sqrt(-2.0F * std::log(u1));
    const auto angle = two_pi * u2;
    mSpare = magnitude * std::sin(angle);
    mHasSpare = true;
    return magnitude * std::cos(angle);
}

Status WattersonChannel::configure(const WattersonConfig& config) noexcept
{
    if (config.sample_rate_hz <= 0.0F || config.doppler_spread_hz <= 0.0F || config.path_delay_samples >= maximum_delay_samples || config.direct_path_gain < 0.0F || config.delayed_path_gain < 0.0F)
    {
        return {StatusCode::invalid_argument, "invalid Watterson channel configuration"};
    }

    mConfig = config;
    DeterministicRng rng{config.seed};

    for (std::size_t index = 0U; index < oscillator_count; ++index)
    {
        mDirectPhases[index] = two_pi * rng.uniform();
        mDelayedPhases[index] = two_pi * rng.uniform();
        constexpr auto branch_oscillators = oscillator_count / 2U;
        const auto branch_index = index % branch_oscillators;
        const auto direct_angle = 0.5F * 3.14159265358979323846F * (as_real(branch_index) + 0.5F) / as_real(branch_oscillators);
        const auto delayed_angle = 0.5F * 3.14159265358979323846F * (as_real(branch_index) + 0.25F) / as_real(branch_oscillators);
        mDirectSteps[index] = two_pi * config.doppler_spread_hz * std::cos(direct_angle) / config.sample_rate_hz;
        mDelayedSteps[index] = two_pi * config.doppler_spread_hz * std::cos(delayed_angle) / config.sample_rate_hz;
    }

    mConfigured = true;
    mDelayLine.fill(IQSample{});
    mDelayCursor = 0U;
    return Status::success();
}

void WattersonChannel::reset() noexcept
{
    mConfigured = false;
    mDelayLine.fill(IQSample{});
    mDelayCursor = 0U;
}

IQSample WattersonChannel::fading_sample(std::array<float, oscillator_count>& phases, const std::array<float, oscillator_count>& steps) noexcept
{
    constexpr auto branch_oscillators = oscillator_count / 2U;
    float in_phase{};
    float quadrature{};

    for (std::size_t index = 0U; index < oscillator_count; ++index)
    {
        auto& branch = index < branch_oscillators ? in_phase : quadrature;
        branch += std::cos(phases[index]);
        phases[index] += steps[index];

        if (phases[index] >= two_pi)
        {
            phases[index] -= two_pi;
        }
        else if (phases[index] < 0.0F)
        {
            phases[index] += two_pi;
        }
    }

    return IQSample{in_phase, quadrature} / std::sqrt(as_real(branch_oscillators));
}

Status WattersonChannel::process(IQSampleSpan input, MutableIQSampleSpan output) noexcept
{
    if (!mConfigured || output.size() < input.size())
    {
        return {StatusCode::invalid_argument, "Watterson channel is not configured or output is too small"};
    }

    for (std::size_t index = 0U; index < input.size(); ++index)
    {
        mDelayLine[mDelayCursor] = input[index];
        const auto delayed_index = (mDelayCursor + maximum_delay_samples - mConfig.path_delay_samples) % maximum_delay_samples;
        const auto direct_fading = fading_sample(mDirectPhases, mDirectSteps);
        const auto delayed_fading = fading_sample(mDelayedPhases, mDelayedSteps);
        output[index] = mConfig.direct_path_gain * direct_fading * input[index] + mConfig.delayed_path_gain * delayed_fading * mDelayLine[delayed_index];
        mDelayCursor = (mDelayCursor + 1U) % maximum_delay_samples;
    }

    return Status::success();
}

} // namespace modem_test
