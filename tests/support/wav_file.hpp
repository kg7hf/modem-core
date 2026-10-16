// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

// =============================================================================
// wav_file.hpp - audio in and out of WAV files
// =============================================================================
// The runner reads a recording and can save the audio its channel made, so the receive
// chain runs on real audio, and the audio it makes can be played or given to another
// modem. Reading takes 16-bit PCM or 32-bit float, mono or the first channel of several.
// Writing is 16-bit PCM, mono. WAV is little-endian whatever the host, so every value is
// put together from its bytes, and taken apart into them.
// =============================================================================

namespace modem_test
{

struct WavAudio
{
    std::uint32_t sample_rate_hz{};
    std::uint16_t channels{};
    std::vector<float> samples{}; // the first channel, full scale is 1
};

[[nodiscard]] modem::common::Result<WavAudio> read_wav(const std::string& path);

// Samples past full scale are clipped to it.
[[nodiscard]] modem::common::Status write_wav(const std::string& path, std::span<const float> samples, std::uint32_t sample_rate_hz);

} // namespace modem_test
