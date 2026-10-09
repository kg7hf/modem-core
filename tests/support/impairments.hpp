// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

// =============================================================================
// impairments.hpp - the test kit's random numbers and fading model
// =============================================================================
// DeterministicRng gives the same numbers on every platform for a given seed, so a
// noisy test repeats exactly. WattersonChannel is a two-path fading model for
// engineering tests. The channel stage (test_channel) applies both to real audio.
// =============================================================================

namespace modem_test
{

using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::MutableIQSampleSpan;
using ::modem::common::Status;

class DeterministicRng
{
public:
    explicit DeterministicRng(std::uint64_t seed = 1U) noexcept;
    [[nodiscard]] std::uint64_t next_u64() noexcept;
    [[nodiscard]] float uniform() noexcept;
    [[nodiscard]] float gaussian() noexcept;

private:
    std::uint64_t mState{};
    bool mHasSpare{};
    float mSpare{};
};

struct WattersonConfig
{
    float sample_rate_hz{48000.0F};
    float doppler_spread_hz{1.0F};
    std::size_t path_delay_samples{96U};
    float direct_path_gain{0.70710678F};
    float delayed_path_gain{0.70710678F};
    std::uint64_t seed{1U};
};

// Deterministic sum-of-sinusoids engineering model with two independent,
// equal-average-power Rayleigh paths. This is not a complete ITU-R F.520-2 mask
// qualification and must not be represented as conformance evidence.
class WattersonChannel
{
public:
    static constexpr std::size_t oscillator_count = 16U;
    static constexpr std::size_t maximum_delay_samples = 512U;

    [[nodiscard]] Status configure(const WattersonConfig& config) noexcept;
    [[nodiscard]] Status process(IQSampleSpan input, MutableIQSampleSpan output) noexcept;
    void reset() noexcept;

private:
    [[nodiscard]] IQSample fading_sample(std::array<float, oscillator_count>& phases, const std::array<float, oscillator_count>& steps) noexcept;

    WattersonConfig mConfig{};
    std::array<float, oscillator_count> mDirectPhases{};
    std::array<float, oscillator_count> mDelayedPhases{};
    std::array<float, oscillator_count> mDirectSteps{};
    std::array<float, oscillator_count> mDelayedSteps{};
    std::array<IQSample, maximum_delay_samples> mDelayLine{};
    std::size_t mDelayCursor{};
    bool mConfigured{};
};

} // namespace modem_test
