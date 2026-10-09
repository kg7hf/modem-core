// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/support/test_channel.hpp"

#include "modem/common/convert.hpp"
#include "modem/common/types.hpp"
#include "tests/support/impairments.hpp"

#include <cmath>
#include <complex>
#include <numbers>

namespace modem_test
{

using ::modem::common::as_real;
using ::modem::common::IQSample;
using ::modem::common::mean;
using ::modem::common::round_to;
using ::modem::common::Status;
using ::modem::common::StatusCode;

namespace
{

constexpr std::uint32_t sample_rate_hz = 48000U;
constexpr float pi = std::numbers::pi_v<float>;

// A real signal's analytic form, x + jH{x}, so a complex channel model can act on it.
// H is a 1025-tap Blackman-windowed Hilbert transformer, centered so there is no delay.
std::vector<IQSample> analytic_signal(const std::vector<float>& audio)
{
    constexpr std::size_t taps = 1025U;
    constexpr std::size_t half = taps / 2U;
    std::vector<float> hilbert(half + 1U); // hilbert[k] multiplies x[n - k] and, negated, x[n + k]

    for (std::size_t k = 1U; k <= half; k += 2U)
    {
        const auto position = as_real(half + k) / as_real(taps - 1U);
        const auto window = 0.42F - 0.5F * std::cos(2.0F * pi * position) + 0.08F * std::cos(4.0F * pi * position);
        hilbert[k] = window * 2.0F / (pi * as_real(k));
    }

    std::vector<IQSample> analytic(audio.size());

    for (std::size_t n = 0U; n < audio.size(); ++n)
    {
        float quadrature{};

        for (std::size_t k = 1U; k <= half; k += 2U)
        {
            const auto before = n >= k ? audio[n - k] : 0.0F;
            const auto after = n + k < audio.size() ? audio[n + k] : 0.0F;
            quadrature += hilbert[k] * (before - after);
        }

        analytic[n] = IQSample{audio[n], quadrature};
    }

    return analytic;
}

Status apply_fading(std::vector<float>& audio, const FadingSpec& fading, std::uint64_t seed)
{
    WattersonChannel channel;
    const WattersonConfig config{
        .sample_rate_hz = as_real(sample_rate_hz),
        .doppler_spread_hz = fading.doppler_spread_hz,
        .path_delay_samples = round_to<std::size_t>(fading.delay_ms * as_real(sample_rate_hz) / 1000.0F),
        .direct_path_gain = 0.70710678F,
        .delayed_path_gain = 0.70710678F,
        .seed = seed,
    };
    auto status = channel.configure(config);

    if (!status.is_ok())
    {
        return status;
    }

    const auto input = analytic_signal(audio);
    std::vector<IQSample> output(input.size());
    status = channel.process(input, output);

    if (!status.is_ok())
    {
        return status;
    }

    for (std::size_t n = 0U; n < audio.size(); ++n)
    {
        audio[n] = output[n].real();
    }

    return Status::success();
}

} // namespace

float signal_rms(const std::vector<float>& audio) noexcept
{
    float power{};
    std::size_t counted{};

    for (const auto sample : audio)
    {
        if (std::fabs(sample) > 0.0F)
        {
            power += sample * sample;
            ++counted;
        }
    }

    return std::sqrt(mean(power, counted));
}

Status apply_channel(std::vector<float>& audio, const ChannelSpec& spec)
{
    const auto rms = signal_rms(audio);

    if (rms <= 0.0F)
    {
        return {StatusCode::invalid_argument, "test channel needs a signal"};
    }

    if (spec.fading)
    {
        const auto status = apply_fading(audio, *spec.fading, spec.seed);

        if (!status.is_ok())
        {
            return status;
        }
    }

    if (spec.cw_db)
    {
        // A tone of amplitude A carries A^2 / 2 of power. The angle is reduced to one
        // cycle in whole numbers first, so the tone stays exact however long the audio.
        const auto amplitude = std::sqrt(2.0F) * rms * std::pow(10.0F, *spec.cw_db / 20.0F);

        for (std::size_t n = 0U; n < audio.size(); ++n)
        {
            const auto cycle_position = (std::uint64_t{spec.cw_hz} * n) % sample_rate_hz;
            audio[n] += amplitude * std::cos(2.0F * pi * as_real(cycle_position) / as_real(sample_rate_hz));
        }
    }

    if (spec.snr_db)
    {
        // White noise spreads evenly over 0 to 24 kHz, so a 3 kHz band holds one eighth
        // of its power: SNR = rms^2 / (sigma^2 / 8).
        const auto sigma = rms * std::sqrt(8.0F) * std::pow(10.0F, -*spec.snr_db / 20.0F);
        DeterministicRng rng{spec.seed ^ 0x5EEDu};

        for (auto& sample : audio)
        {
            sample += sigma * rng.gaussian();
        }
    }

    return Status::success();
}

} // namespace modem_test
