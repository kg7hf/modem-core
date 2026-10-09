// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/m110a/transmitter.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

// =============================================================================
// test_waveform.hpp - test signals from the real transmitter
// =============================================================================
// The transmitter is the reference every receive stage is measured against, so the
// test bench builds its signals with it: a mode, a payload (given, or random from a
// seed so every run repeats), and any number of bursts back to back. Alongside the
// audio it keeps the truth each receive stage is checked against. Later stages add
// their own truth to BurstTruth (coded bits for the decoder, for example).
// =============================================================================

namespace modem_test
{

struct WaveformSpec
{
    modem::m110a::BodyMode mode{modem::common::DataRate::bps600, modem::common::BodyInterleave::long_block};
    std::vector<std::uint8_t> payload{};   // sent as given when not empty
    std::size_t random_payload_bytes{16U}; // otherwise this many random bytes per burst...
    std::uint64_t seed{1U};                // ...drawn from this seed
    std::size_t bursts{1U};
    std::size_t gap_samples{}; // silence between bursts, in 48 kHz samples
};

// What one burst sent, for the receive stages to be checked against.
struct BurstTruth
{
    std::size_t first_sample{};     // the burst's first audio sample
    std::size_t preamble_symbols{}; // symbols before the body starts
    std::vector<std::uint8_t> payload{};
    std::vector<std::uint8_t> tribits{}; // every transmitted symbol, preamble and body
};

struct TestWaveform
{
    std::vector<float> audio{};
    std::vector<BurstTruth> bursts{};
};

[[nodiscard]] modem::common::Result<TestWaveform> make_test_waveform(const WaveformSpec& spec);

} // namespace modem_test
