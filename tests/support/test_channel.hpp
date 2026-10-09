// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

// =============================================================================
// test_channel.hpp - what the air does to a test signal
// =============================================================================
// The channel stage applies the impairment kit to real 48 kHz audio, with fixed
// seeds so every run repeats. Signal-to-noise is set the way the standard measures
// it: signal power over the noise power in a 3 kHz band. The fading is the kit's
// two-path Watterson model, an engineering model and not a qualified ITU-R F.520
// simulator, so results through it are engineering evidence, not conformance results.
// =============================================================================

namespace modem_test
{

struct FadingSpec
{
    float delay_ms{2.0F};          // delay of the second, equal-power path
    float doppler_spread_hz{1.0F}; // how fast the two paths fade
};

struct ChannelSpec
{
    std::optional<float> snr_db{}; // white noise: signal over noise in a 3 kHz band
    std::optional<float> cw_db{};  // a CW tone: its power relative to the signal
    std::uint32_t cw_hz{1800U};    // whole hertz, so the tone is exact
    std::optional<FadingSpec> fading{};
    std::uint64_t seed{1U};
};

// Applies the channel to the audio in place, in the order a real link does: the
// path, interference, then noise at the receiver.
[[nodiscard]] modem::common::Status apply_channel(std::vector<float>& audio, const ChannelSpec& spec);

// The RMS of the non-silent samples: the signal level the SNR and the CW level are set against.
[[nodiscard]] float signal_rms(const std::vector<float>& audio) noexcept;

} // namespace modem_test
