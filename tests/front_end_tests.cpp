// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// front_end_tests.cpp - the receive front end's tests
// =============================================================================
// Each test feeds known data through one piece of the front end and checks what
// comes out. The signals come from the real transmitter (support/test_waveform),
// the impairments from the channel stage (support/test_channel), and the scoring
// from the front-end harness (support/front_end_harness). Run them from VS Code's
// Testing panel or with ctest; set a breakpoint and debug any one of them. The
// [.long] test runs only when asked for (ctest -L long).
// Explained in the article series, Part 3 (the audio front end).
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/dsp/timing_recovery.hpp"
#include "modem/m110a/body_audio_stream_frontend.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/scrambler.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/test_channel.hpp"
#include "tests/support/test_waveform.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <print>
#include <vector>

namespace
{

using namespace ::modem_test;
using ::modem::common::as_real;
using ::modem::common::CarrierTracker;
using ::modem::common::floor_to;
using ::modem::common::IQSample;
using ::modem::common::make_root_raised_cosine_taps;
using ::modem::common::narrow;
using ::modem::common::widen;
using ::modem::m110a::BodyAudioStreamFrontend;
using ::modem::m110a::early_late_timing_error;

constexpr std::size_t samples_per_symbol = ::modem::m110a::body_audio_samples_per_symbol; // 20 at 48 kHz
constexpr std::size_t half_symbol = samples_per_symbol / 2U;
constexpr float pi = std::numbers::pi_v<float>;

// The two-character message from Part 2, with the truth of what was sent.
TestWaveform hi_waveform()
{
    WaveformSpec spec;
    spec.payload = {'H', 'i'};
    const auto waveform = make_test_waveform(spec);
    REQUIRE(waveform.error().is_ok());
    return waveform.value();
}

FrontEndRunner run(const std::vector<float>& audio, const FrontEndOptions& options = {})
{
    FrontEndRunner runner;
    REQUIRE(runner.initialize(options).is_ok());
    REQUIRE(runner.feed(audio).is_ok());
    return runner;
}

bool same_bits(IQSample a, IQSample b) noexcept
{
    return std::bit_cast<std::uint32_t>(a.real()) == std::bit_cast<std::uint32_t>(b.real()) && std::bit_cast<std::uint32_t>(a.imag()) == std::bit_cast<std::uint32_t>(b.imag());
}

} // namespace

TEST_CASE("The receive filter has unit energy and is symmetric about its peak", "[front_end][pulse]")
{
    std::array<float, BodyAudioStreamFrontend::matched_filter_taps> taps{};
    REQUIRE(make_root_raised_cosine_taps(samples_per_symbol, BodyAudioStreamFrontend::matched_filter_rolloff, taps).is_ok());

    float energy{};
    for (const auto tap : taps)
    {
        energy += tap * tap;
    }

    const std::size_t middle = taps.size() / 2U;
    std::println("receive filter: {} taps, energy {:.6f}, peak {:.4f} at tap {}", taps.size(), energy, taps[middle], middle);
    CHECK(std::fabs(energy - 1.0F) < 1.0e-5F);
    CHECK(std::ranges::max_element(taps) == taps.begin() + narrow<std::ptrdiff_t>(middle));

    for (std::size_t index = 0U; index < middle; ++index)
    {
        CHECK(std::fabs(taps[index] - taps[taps.size() - 1U - index]) < 1.0e-6F);
    }
}

TEST_CASE("Transmit pulse then receive filter leaves the neighbors' instants nearly empty", "[front_end][pulse]")
{
    const auto receive_rolloff = GENERATE(0.25F, BodyAudioStreamFrontend::matched_filter_rolloff);
    std::vector<float> transmit(::modem::m110a::body_audio_shaping_taps);
    std::vector<float> receive(BodyAudioStreamFrontend::matched_filter_taps);
    REQUIRE(make_root_raised_cosine_taps(samples_per_symbol, 0.25F, transmit).is_ok());
    REQUIRE(make_root_raised_cosine_taps(samples_per_symbol, receive_rolloff, receive).is_ok());

    std::vector<float> combined(transmit.size() + receive.size() - 1U);
    for (std::size_t i = 0U; i < transmit.size(); ++i)
    {
        for (std::size_t j = 0U; j < receive.size(); ++j)
        {
            combined[i + j] += transmit[i] * receive[j];
        }
    }

    const std::size_t center = combined.size() / 2U;
    float largest_neighbor{};
    for (std::size_t offset = samples_per_symbol; offset <= center; offset += samples_per_symbol)
    {
        largest_neighbor = std::max({largest_neighbor, std::fabs(combined[center - offset]), std::fabs(combined[center + offset])});
    }

    std::println("transmit 0.25, receive {:.2f}: peak {:.4f}, largest neighbor {:.5f} ({:.1f} dB below)", receive_rolloff, combined[center], largest_neighbor,
                 20.0F * std::log10(combined[center] / largest_neighbor));
    CHECK(largest_neighbor < 0.02F * combined[center]);
}

TEST_CASE("A tone on the carrier mixes down to a constant and one 10 Hz off turns", "[front_end][mixer]")
{
    constexpr std::uint32_t sample_rate = 48000U;
    const auto tone_hz = GENERATE(1800U, 1810U);

    // The angle is reduced to one cycle in whole numbers first, so the tone is exact.
    std::vector<float> tone(sample_rate);
    for (std::uint32_t n = 0U; n < sample_rate; ++n)
    {
        tone[n] = 0.5F * std::cos(2.0F * pi * as_real((tone_hz * n) % sample_rate) / as_real(sample_rate));
    }

    const auto runner = run(tone);
    const auto& symbols = runner.symbols();
    constexpr std::size_t settled = 20U; // skip the filter filling up
    REQUIRE(symbols.size() > settled + 100U);

    float step_sum{};
    float smallest = 1.0e9F;
    float largest{};
    for (std::size_t k = settled + 1U; k < symbols.size(); ++k)
    {
        step_sum += std::arg(symbols[k].on_time * std::conj(symbols[k - 1U].on_time));
        smallest = std::min(smallest, std::abs(symbols[k].on_time));
        largest = std::max(largest, std::abs(symbols[k].on_time));
    }

    const auto step = step_sum / as_real(symbols.size() - settled - 1U);
    const auto expected = 2.0F * pi * as_real(tone_hz - 1800U) / 2400.0F;
    std::println("{} Hz tone: magnitude {:.4f} to {:.4f}, phase step {:+.6f} rad/symbol (expected {:+.6f})", tone_hz, smallest, largest, step, expected);
    CHECK(std::fabs(step - expected) < 1.0e-4F);

    // On the carrier the output is exactly constant. Off it, the mix's mirror image near
    // 3600 Hz is far down the filter's stopband but not gone: a faint ripple.
    if (tone_hz == 1800U)
    {
        CHECK(largest - smallest < 1.0e-4F * largest);
    }
}

TEST_CASE("The 1800 Hz carrier repeats every 80 samples: three whole cycles", "[front_end][carrier]")
{
    using ::modem::m110a::body_carrier_period_samples;
    using ::modem::m110a::body_carrier_table;

    // 1800 cycles a second at 48,000 samples a second is 3 cycles every 80 samples, so 80
    // phasors cover the carrier exactly; entry n turns 3n/80 of a cycle, 13.5 degrees a sample.
    STATIC_REQUIRE(1800U * body_carrier_period_samples == 3U * 48000U);

    std::println("  n   phase (deg)      cos        sin");
    for (std::size_t n = 0U; n < body_carrier_period_samples; ++n)
    {
        const auto cycles = widen<double>(as_real((3U * n) % body_carrier_period_samples)) / 80.0;
        const auto exact = std::polar(1.0, 2.0 * std::numbers::pi * cycles);
        const auto entry = body_carrier_table[n];

        if (n < 8U || n % 20U == 0U)
        {
            std::println("{:3}   {:8.1f}   {:+9.6f}  {:+9.6f}", n, 360.0 * cycles, entry.real(), entry.imag());
        }

        // Each entry is the exact phasor rounded once to float: within half a float step.
        CHECK(std::fabs(widen<double>(entry.real()) - exact.real()) < 6.0e-8);
        CHECK(std::fabs(widen<double>(entry.imag()) - exact.imag()) < 6.0e-8);
    }

    // The quadrants are exact: 0, 270, 180 and 90 degrees.
    CHECK(body_carrier_table[0] == IQSample{1.0F, 0.0F});
    CHECK(body_carrier_table[20] == IQSample{0.0F, -1.0F});
    CHECK(body_carrier_table[40] == IQSample{-1.0F, 0.0F});
    CHECK(body_carrier_table[60] == IQSample{0.0F, 1.0F});
}

TEST_CASE("Mixing Part 2's tribit 6 down from 1800 Hz", "[front_end][carrier]")
{
    using ::modem::m110a::body_carrier_period_samples;
    using ::modem::m110a::body_carrier_table;

    const auto waveform = hi_waveform();
    const auto& burst = waveform.bursts.front();

    // Part 2 ended on the randomizer's sixth value, 6, and its worked symbol was tribit 6, 270
    // degrees. In the Hi burst they meet at body symbol 5: a coded 0 plus randomizer value 6.
    // The transmitter puts each symbol's peak at 20 times its number.
    const auto symbol = burst.preamble_symbols + 5U;
    REQUIRE(burst.tribits[symbol] == 6U);
    const auto peak = burst.first_sample + symbol * samples_per_symbol;

    // Each mixed sample is the symbol plus a copy spinning at 3600 Hz (and a little of the
    // neighboring symbols); the matched filter averages the spin away.
    std::println("   sample     audio   carrier (deg)   mixed I   mixed Q");
    for (auto n = peak - 4U; n <= peak + 4U; ++n)
    {
        const auto carrier = body_carrier_table[n % body_carrier_period_samples];
        const auto mixed = 2.0F * waveform.audio[n] * std::conj(carrier);
        auto degrees = std::arg(carrier) * 180.0F / pi;
        degrees += degrees < 0.0F ? 360.0F : 0.0F;
        std::println("{:9}  {:+8.4f}   {:12.1f}   {:+7.4f}   {:+7.4f}", n, waveform.audio[n], degrees, mixed.real(), mixed.imag());
    }

    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, burst, 0U);
    REQUIRE(score.locked);
    const auto k = narrow<std::int64_t>(symbol) + score.offset - narrow<std::int64_t>(runner.symbol_base());
    const auto on_time = runner.symbols()[narrow<std::size_t>(k)].on_time;
    auto degrees = std::arg(on_time) * 180.0F / pi;
    degrees += degrees < 0.0F ? 360.0F : 0.0F;
    std::println("after the matched filter: on-time ({:+.3f}, {:+.3f}), {:.1f} degrees, tribit {}", on_time.real(), on_time.imag(), degrees, nearest_tribit(on_time));
    CHECK(nearest_tribit(on_time) == 6U);
}

TEST_CASE("The early-late error is zero on the peak and signed off it for any carrier phase", "[front_end][timing]")
{
    std::array<float, BodyAudioStreamFrontend::matched_filter_taps> taps{};
    REQUIRE(make_root_raised_cosine_taps(samples_per_symbol, BodyAudioStreamFrontend::matched_filter_rolloff, taps).is_ok());

    const std::size_t peak = taps.size() / 2U;
    const auto error_at = [&](std::size_t on_time, float carrier_phase)
    {
        const auto turn = std::polar(1.0F, carrier_phase);
        return early_late_timing_error(IQSample{taps[on_time - half_symbol]} * turn, IQSample{taps[on_time]} * turn, IQSample{taps[on_time + half_symbol]} * turn);
    };

    const auto on_peak = error_at(peak, 0.0F);
    const auto early = error_at(peak - 4U, 0.0F);
    const auto late = error_at(peak + 4U, 0.0F);
    const auto early_turned = error_at(peak - 4U, 1.0F);
    std::println("on the peak {:+.6f}, 4 samples early {:+.6f}, 4 samples late {:+.6f}, early and turned by 1 rad {:+.6f}", on_peak, early, late, early_turned);
    CHECK(std::fabs(on_peak) < 1.0e-6F);
    CHECK(early > 0.0F);
    CHECK(late < 0.0F);
    CHECK(std::fabs(early + late) < 1.0e-6F);
    CHECK(std::fabs(early_turned - early) < 1.0e-6F);
}

TEST_CASE("A float timing position falls off the grid; whole samples plus a fraction do not", "[front_end][timing]")
{
    float position = 67108864.0F; // 2^26 samples, about 23 minutes at 48 kHz
    position += 20.0F;
    float earlier = 16777216.0F; // 2^24
    earlier += 20.5F;
    std::println("float: 2^26 + 20 = {:.1f}; 2^24 + 20.5 = {:.1f}", position, earlier);
    CHECK(std::bit_cast<std::uint32_t>(position) == std::bit_cast<std::uint32_t>(67108880.0F));
    CHECK(std::bit_cast<std::uint32_t>(earlier) == std::bit_cast<std::uint32_t>(16777236.0F));

    std::uint64_t whole = 67108864U;
    float fraction = 0.5F;
    fraction += 20.5F;
    const auto step = std::floor(fraction);
    whole += floor_to<std::uint64_t>(step);
    fraction -= step;
    std::println("whole + fraction: {} + {:.1f}", whole, fraction);
    CHECK(whole == 67108885U);
    CHECK(std::fabs(fraction) < 1.0e-6F);
}

TEST_CASE("Any block size gives the same symbols bit for bit", "[front_end][stream]")
{
    const auto waveform = hi_waveform();
    const auto steady = run(waveform.audio, {.block_sizes = {480U}});
    const auto ragged = run(waveform.audio, {.block_sizes = {1U, 7U, 33U, 160U, 480U}});
    REQUIRE(steady.symbols().size() == ragged.symbols().size());

    std::size_t identical{};
    for (std::size_t k = 0U; k < steady.symbols().size(); ++k)
    {
        identical += same_bits(steady.symbols()[k].early, ragged.symbols()[k].early) && same_bits(steady.symbols()[k].on_time, ragged.symbols()[k].on_time) ? 1U : 0U;
    }

    std::println("{} samples, {} symbols, {} identical bit for bit", waveform.audio.size(), steady.symbols().size(), identical);
    CHECK(identical == steady.symbols().size());
}

TEST_CASE("Hi: after the loop locks every decision is the transmitted tribit", "[front_end][end_to_end]")
{
    const auto waveform = hi_waveform();
    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    std::println("locked by symbol {}; {} of {} decisions match", score.locked_from, score.correct, score.compared);
    CHECK(score.locked);
    CHECK(score.compared > 10000U);
    CHECK(score.correct == score.compared);
}

TEST_CASE("Walk: Part 2's randomizer values come back out of the front end", "[front_end][walk]")
{
    const auto waveform = hi_waveform();
    const auto& burst = waveform.bursts.front();
    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, burst, 0U);
    REQUIRE(score.locked);

    ::modem::m110a::BodyDataRandomizer randomizer;
    randomizer.reset();
    std::println("body  on-time I, Q       angle  tribit  sent  randomizer  tribit - randomizer  coded bit");

    // 600L frames are 20 data symbols then 20 probe symbols; only data symbols carry coded
    // bits. Show the first six body symbols, then the first that carries a coded 1.
    bool shown_a_one = false;
    for (std::size_t j = 0U; j < 400U && !shown_a_one; ++j)
    {
        const auto sent = burst.tribits[burst.preamble_symbols + j];
        const auto k = narrow<std::int64_t>(burst.preamble_symbols + j) + score.offset - narrow<std::int64_t>(runner.symbol_base());
        const auto on_time = runner.symbols()[narrow<std::size_t>(k)].on_time;
        const auto received = nearest_tribit(on_time);
        const auto added = randomizer.next_tribit();
        const auto data = narrow<std::uint8_t>((received + 8U - added) % 8U);
        const bool is_data = j % 40U < 20U;

        if (j < 6U || (is_data && data == 4U))
        {
            auto degrees = std::arg(on_time) * 180.0F / pi;
            degrees += degrees < 0.0F ? 360.0F : 0.0F;
            if (j >= 6U)
            {
                std::println(" ...");
                shown_a_one = true;
            }
            std::println("{:4}  {:+7.3f} {:+7.3f}  {:6.1f}  {:6}  {:4}  {:10}  {:19}  {:9}", j, on_time.real(), on_time.imag(), degrees, received, sent, added, data, data == 4U ? 1 : 0);
        }

        CHECK(received == sent);
        CHECK((!is_data || data == 0U || data == 4U));
    }

    CHECK(shown_a_one);
}

TEST_CASE("The carrier tracker learns a fixed phase and then a frequency offset", "[front_end][tracker]")
{
    constexpr float proportional_gain = 0.02F; // the receiver's carrier-loop gains
    constexpr float integral_gain = 0.0002F;
    std::uint32_t state = 1U;
    const auto next_symbol = [&state]
    {
        state = state * 1664525U + 1013904223U; // a small fixed pseudo-random sequence
        return std::polar(1.0F, pi / 4.0F * as_real(state >> 29U));
    };

    CarrierTracker tracker;
    REQUIRE(tracker.configure(proportional_gain, integral_gain).is_ok());

    float residual{};
    tracker.reset();
    for (int k = 0; k < 2000; ++k)
    {
        const auto decision = next_symbol();
        const auto corrected = tracker.update(decision * std::polar(1.0F, 0.5F), decision);
        residual = std::fabs(std::arg(corrected * std::conj(decision)));
    }
    std::println("fixed 0.5 rad offset: learned {:.4f} rad, last symbol {:.5f} rad from its decision", tracker.phase_radians(), residual);
    CHECK(std::fabs(tracker.phase_radians() - 0.5F) < 0.01F);
    CHECK(residual < 0.01F);

    const auto per_symbol = 2.0F * pi * 5.0F / 2400.0F; // 5 Hz at 2400 symbols per second
    float angle{};
    tracker.reset();
    for (int k = 0; k < 5000; ++k)
    {
        const auto decision = next_symbol();
        const auto corrected = tracker.update(decision * std::polar(1.0F, angle), decision);
        residual = std::fabs(std::arg(corrected * std::conj(decision)));
        angle = std::remainder(angle + per_symbol, 2.0F * pi);
    }
    const auto learned_hz = tracker.frequency_radians_per_symbol() * 2400.0F / (2.0F * pi);
    std::println("5 Hz offset: learned {:.3f} Hz, last symbol {:.5f} rad from its decision", learned_hz, residual);
    CHECK(std::fabs(learned_hz - 5.0F) < 0.01F);
    CHECK(residual < 0.01F);
}

TEST_CASE("A train of ten random messages with gaps: every decision after lock matches", "[front_end][bench]")
{
    WaveformSpec spec;
    spec.random_payload_bytes = 32U;
    spec.seed = 7U;
    spec.bursts = 10U;
    spec.gap_samples = 12345U; // not a whole number of symbols, so each burst starts off the old grid
    const auto waveform = make_test_waveform(spec);
    REQUIRE(waveform.error().is_ok());

    const auto runner = run(waveform.value().audio);
    for (const auto& burst : waveform.value().bursts)
    {
        const auto score = score_burst(runner, burst, burst.first_sample);
        std::println("burst at sample {:8}: locked by symbol {:5}, {} of {} decisions match, carrier turned {:+6.1f} degrees", burst.first_sample, score.locked_from, score.correct, score.compared,
                     score.rotation_degrees);
        CHECK(score.locked);
        CHECK(score.correct == score.compared);
    }
}

TEST_CASE("Noise at 10 dB: the loop locks and most decisions are right", "[front_end][channel]")
{
    const auto seed = GENERATE(1U, 2U, 3U);
    auto waveform = hi_waveform();
    REQUIRE(apply_channel(waveform.audio, {.snr_db = 10.0F, .seed = seed}).is_ok());

    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    const auto error_rate = 1.0F - as_real(score.correct) / as_real(std::max<std::size_t>(score.compared, 1U));
    std::println("AWGN, SNR 10 dB in 3 kHz, seed {}: locked by symbol {}, symbol errors {:.2f}%, rms phase error {:.1f} degrees", seed, score.locked_from, 100.0F * error_rate,
                 score.rms_phase_error_degrees);
    CHECK(score.locked);
    CHECK(error_rate < 0.10F);
}

TEST_CASE("A CW tone 10 dB below the signal: the loop locks and most decisions are right", "[front_end][channel]")
{
    auto waveform = hi_waveform();
    REQUIRE(apply_channel(waveform.audio, {.snr_db = 20.0F, .cw_db = -10.0F}).is_ok());

    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    const auto error_rate = 1.0F - as_real(score.correct) / as_real(std::max<std::size_t>(score.compared, 1U));
    std::println("CW on the carrier at -10 dB, SNR 20 dB in 3 kHz: locked by symbol {}, symbol errors {:.2f}%", score.locked_from, 100.0F * error_rate);
    CHECK(score.locked);
    CHECK(error_rate < 0.10F);
}

TEST_CASE("Watterson fading at 2 ms and 1 Hz with 20 dB SNR: what the front end alone delivers", "[front_end][channel][.report]")
{
    auto waveform = hi_waveform();
    REQUIRE(apply_channel(waveform.audio, {.snr_db = 20.0F, .fading = FadingSpec{.delay_ms = 2.0F, .doppler_spread_hz = 1.0F}}).is_ok());

    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    std::println("Watterson 2 ms / 1 Hz, SNR 20 dB in 3 kHz (engineering model, not conformance): locked {}, {} of {} decisions match", score.locked, score.correct, score.compared);
    SUCCEED("reported; the equalizer, in a later article, is what makes fading decodable");
}

TEST_CASE("Thirty minutes of continuous traffic: still exact past 2^26 samples", "[front_end][.long]")
{
    constexpr std::uint64_t thirty_minutes = 30U * 60U * 48000U;
    FrontEndRunner runner;
    REQUIRE(runner.initialize().is_ok());

    std::size_t bursts{};
    std::size_t clean_bursts{};
    WaveformSpec spec;

    while (runner.samples_fed() < thirty_minutes)
    {
        spec.seed = bursts + 1U;
        const auto waveform = make_test_waveform(spec);
        REQUIRE(waveform.error().is_ok());

        const auto first_sample = runner.samples_fed();
        REQUIRE(runner.feed(waveform.value().audio).is_ok());
        const auto score = score_burst(runner, waveform.value().bursts.front(), first_sample);
        clean_bursts += score.locked && score.correct == score.compared ? 1U : 0U;
        CHECK(score.locked);
        CHECK(score.correct == score.compared);
        runner.discard_symbols();
        ++bursts;
    }

    std::println("{} bursts back to back, {} samples ({} s); {} with every decision right", bursts, runner.samples_fed(), runner.samples_fed() / 48000U, clean_bursts);
    CHECK(runner.samples_fed() > (std::uint64_t{1} << 26U));
    CHECK(clean_bursts == bursts);
}
