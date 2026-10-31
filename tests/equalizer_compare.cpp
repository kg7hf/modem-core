// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// equalizer_compare - the two equalizers on the same bursts, row by row
// =============================================================================
// A small comparison you can run yourself, for the Signal Path article "The
// equalizer". For each long-interleave row of the standard's Table 10.1 (the 2 ms /
// 1 Hz channel for 2400, 1200 and 600 bps, the 5 ms / 5 Hz channel for 300 and 150, each
// at the row's signal-to-noise ratio), it sends bursts of random bytes through the
// test bench's two-path channel, one seed per burst, receives each burst once, and
// equalizes it twice: with the adaptive equalizer and with CIR-MMSE. Both engines see
// the same bursts, so the comparison is paired. It counts the data decisions each gets
// wrong before the decoder, in the mode's own alphabet, and how many bursts each engine
// decided better.
//
// Two limits. The bench's channel is an engineering model, not a qualified simulator,
// so these are engineering numbers, not conformance results. And they are decisions
// before the decoder: whether each burst's message comes out is Part 7's measure.
//
//   Build:  cmake -S . -B build && cmake --build build
//   Run:    ./build/equalizer_compare               (20 bursts a row)
//           ./build/equalizer_compare --seeds 100
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/status.hpp"
#include "tests/support/test_channel.hpp"
#include "tests/support/test_waveform.hpp"
#include "tests/walkthrough/equalizer_steps.hpp"

#include <array>
#include <charconv>
#include <cstdio>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

using namespace modem_test::equalizer_steps;
using ::modem::common::as_real;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;

namespace
{

// One row of Table 10.1, long interleave.
struct Row
{
    const char* name{};
    DataRate rate{};
    float delay_ms{};
    float doppler_hz{};
    float snr_db{};
};

constexpr std::array rows{
    Row{"2400L", DataRate::bps2400, 2.0F, 1.0F, 18.0F}, Row{"1200L", DataRate::bps1200, 2.0F, 1.0F, 11.0F}, Row{"600L", DataRate::bps600, 2.0F, 1.0F, 7.0F},
    Row{"300L", DataRate::bps300, 5.0F, 5.0F, 7.0F},    Row{"150L", DataRate::bps150, 5.0F, 5.0F, 5.0F},
};

constexpr std::size_t lead_samples = 24000U; // half a second of line on each side of the burst
constexpr std::size_t payload_bytes = 32U;

// One burst through the channel, both engines: the data decisions each got wrong, or none if
// the burst was not acquired.
struct Pair
{
    bool acquired{};
    std::size_t symbols{};
    std::size_t adaptive_errors{};
    std::size_t cir_errors{};
};

Pair one_burst(const Row& row, std::uint64_t seed)
{
    const BodyMode mode{row.rate, BodyInterleave::long_block};
    const auto wave = ::modem_test::make_test_waveform(::modem_test::WaveformSpec{.mode = mode, .random_payload_bytes = payload_bytes, .seed = seed});
    if (!wave)
    {
        return {};
    }
    std::vector<float> audio(lead_samples, 0.0F);
    audio.insert(audio.end(), wave.value().audio.begin(), wave.value().audio.end());
    audio.resize(audio.size() + lead_samples, 0.0F);
    const ::modem_test::ChannelSpec channel{.snr_db = row.snr_db, .fading = ::modem_test::FadingSpec{.delay_ms = row.delay_ms, .doppler_spread_hz = row.doppler_hz}, .seed = seed};
    if (!::modem_test::apply_channel(audio, channel).is_ok())
    {
        return {};
    }
    const auto burst = receive_audio(row.name, mode, audio, wave.value());
    if (!burst)
    {
        return {};
    }

    Pair pair{.acquired = true};
    for (const auto engine : {BodyDataEqualizer::adaptive, BodyDataEqualizer::cir_mmse})
    {
        auto trained = train(burst.value(), engine);
        const auto body = trained ? walk_body(burst.value(), trained.value()) : Result<Body>{trained.error()};
        if (!body)
        {
            return {};
        }
        pair.symbols = body.value().data_symbols;
        (engine == BodyDataEqualizer::adaptive ? pair.adaptive_errors : pair.cir_errors) = body.value().data_errors;
    }
    return pair;
}

float percent(std::size_t errors, std::size_t symbols) noexcept
{
    return symbols == 0U ? 0.0F : 100.0F * as_real(errors) / as_real(symbols);
}

} // namespace

int main(int argc, char** argv)
{
    std::size_t seeds = 20U;
    const std::span<char*> args(argv, ::modem::common::narrow<std::size_t>(argc));
    if (args.size() == 3U && std::string_view{args[1]} == "--seeds")
    {
        const std::string_view text{args[2]};
        if (std::from_chars(text.data(), text.data() + text.size(), seeds).ec != std::errc{} || seeds == 0U)
        {
            std::println(stderr, "equalizer_compare: --seeds wants a positive whole number");
            return 2;
        }
    }
    else if (args.size() != 1U)
    {
        std::println(stderr, "usage: equalizer_compare [--seeds N]");
        return 2;
    }
    if (!::modem::m110a::body_receiver_engine_available(BodyDataEqualizer::cir_mmse))
    {
        std::println(stderr, "equalizer_compare: this build leaves the CIR-MMSE engine out (M110_BODY_RECEIVER_CIR_ENGINE=OFF)");
        return 2;
    }

    std::println("== The two equalizers on the same bursts: {} a row, the bench's two-path channel ==", seeds);
    std::println("   data decisions wrong before the decoder, in the mode's own alphabet\n");
    std::println("   row     channel      SNR   acquired   adaptive   CIR-MMSE   adaptive better   CIR-MMSE better   same");
    for (const auto& row : rows)
    {
        std::size_t acquired = 0U;
        std::size_t symbols = 0U;
        std::size_t adaptive_errors = 0U;
        std::size_t cir_errors = 0U;
        std::size_t adaptive_better = 0U;
        std::size_t cir_better = 0U;
        for (std::uint64_t seed = 1U; seed <= seeds; ++seed)
        {
            const auto pair = one_burst(row, seed);
            if (!pair.acquired)
            {
                continue;
            }
            ++acquired;
            symbols += pair.symbols;
            adaptive_errors += pair.adaptive_errors;
            cir_errors += pair.cir_errors;
            adaptive_better += pair.adaptive_errors < pair.cir_errors ? 1U : 0U;
            cir_better += pair.cir_errors < pair.adaptive_errors ? 1U : 0U;
        }
        std::println("   {:<6}  {:.0f} ms {:.0f} Hz  {:4.0f} dB   {:3} of {:<3} {:7.2f}%   {:7.2f}%   {:15}   {:15}   {:4}", row.name, row.delay_ms, row.doppler_hz, row.snr_db, acquired, seeds,
                     percent(adaptive_errors, symbols), percent(cir_errors, symbols), adaptive_better, cir_better, acquired - adaptive_better - cir_better);
    }
    return 0;
}
